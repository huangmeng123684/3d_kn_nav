from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCAN_MANAGE = ROOT.parent / 'SCAN-Planner' / 'src' / 'planner' / 'plan_manage'
OPEN3D_LOC = (
    ROOT.parent / 'FAST_LIO_LOCALIZATION_HUMANOID' / 'open3d_loc'
)
PCT_PLANNER = ROOT.parent / 'PCT_planner'


def load(profile, name):
    return yaml.safe_load((ROOT / 'config' / profile / name).read_text())


def test_pct_online_planner_does_not_require_raw_pcd():
    source = (PCT_PLANNER / 'scripts/run_ros2_global_planner.py').read_text()
    for legacy in ('pcd_path', '/global_points', '_publish_pcd', 'import open3d'):
        assert legacy not in source

    config_paths = [
        PCT_PLANNER / 'params/pct_global_planner.yaml',
        *(
            ROOT / 'config' / profile / 'pct_global_planner.yaml'
            for profile in ('A2', 'local', 'unitree_go2', 'unitree_go2w')
        ),
    ]
    for config_path in config_paths:
        params = yaml.safe_load(config_path.read_text())[
            'pct_global_planner'
        ]['ros__parameters']
        assert 'pcd_path' not in params
        assert params['tomo_path']


def test_coordinator_profiles_use_lightweight_mode2_contract():
    expected = {
        'mode': 2,
        'global_frame': 'map',
        'path_topic': '/pct_path',
        'waypoints_topic': '/scan_planner/waypoints',
        'waypoint_spacing': 1.0,
        'waypoint_z_offset': 0.0,
    }
    for profile in ('A2', 'local', 'unitree_go2', 'unitree_go2w'):
        params = load(profile, 'coordinator.yaml')['pct_scan_coordinator']['ros__parameters']
        assert params == expected


def test_scan_profiles_select_dynamic_waypoint_mode():
    unsupported = {
        'body_pose_topic',
        'fsm.minimum_planning_horizon',
        'fsm.planning_horizon_shrink_step',
        'fsm.planning_failures_per_horizon_shrink',
        'fsm.emergency_time_',
        'fsm.path_z_offset',
        'fsm.max_path_odom_z_difference',
        'fsm.local_target_max_z_difference',
        'fsm.local_target_topic',
        'grid_map.cloud_timeout',
    }
    for profile in ('local', 'unitree_go2', 'unitree_go2w'):
        scan = load(profile, 'scan_planner.yaml')
        params = scan['scan_planner_node']['ros__parameters']
        controller = scan['closed_loop_controller']['ros__parameters']
        assert params['fsm.navi_mode'] == 2
        assert params['fsm.emergency_time'] > 0.0
        assert params['grid_map.frame_id'] == 'map'
        assert params['grid_map.cloud_is_world'] is True
        assert params['grid_map.need_extrinsic'] is False
        assert params['grid_map.body_height'] > 0.0
        assert unsupported.isdisjoint(params)
        assert 'body_pose_topic' not in controller
        assert 'odom_timeout' not in controller
        assert 'trajectory_timeout' not in controller
        assert params['fsm.finish_dist'] == 0.15
        assert params['fsm.finish_yaw'] == 0.10
        assert controller['finish_dist'] == 0.15
        assert controller['finish_yaw'] == 0.10


def test_launch_synchronizes_modes_and_current_scan_topics():
    text = (ROOT / 'launch/local_pct_scan_navigation.launch.py').read_text()
    assert "'navigation_mode', default_value='2'" in text
    assert "'fsm.navi_mode': navigation_mode_value" in text
    assert "'mode': navigation_mode_value" in text
    assert "navigation_mode, \"' == '2'" in text
    assert "('body_pose', '/Odometry_open3d')" in text
    assert "('sensor_pose', '/Odometry_open3d')" in text
    assert "('cloud', '/scan_map')" in text
    assert "('move_base_simple/goal', '/goal_pose')" in text
    assert "('waypoints', '/scan_planner/waypoints')" in text
    assert "on_exit=Shutdown" in text


def test_robot_launches_forward_navigation_mode():
    for robot in ('unitree_go2', 'unitree_go2w'):
        text = (ROOT / 'launch' / f'{robot}_pct_scan_navigation.launch.py').read_text()
        assert "DeclareLaunchArgument('navigation_mode', default_value='2')" in text
        assert "'navigation_mode': LaunchConfiguration('navigation_mode')" in text
        assert f"'config_profile': '{robot}'" in text


def test_navigation_service_interfaces_are_minimal_and_direct():
    expected = {
        'Relocalize.srv': [
            'float64 x', 'float64 y', 'float64 z',
            'float64 qx', 'float64 qy', 'float64 qz', 'float64 qw',
            '---', 'bool success', 'string message',
        ],
        'GetPose.srv': [
            '---', 'bool success',
            'float64 x', 'float64 y', 'float64 z',
            'float64 qx', 'float64 qy', 'float64 qz', 'float64 qw',
            'string message',
        ],
        'PublishGoal.srv': [
            'float64 x', 'float64 y', 'float64 z',
            'float64 qx', 'float64 qy', 'float64 qz', 'float64 qw',
            '---', 'bool success', 'string message',
        ],
    }
    for name, fields in expected.items():
        text = (OPEN3D_LOC / 'srv' / name).read_text()
        assert [line.strip() for line in text.splitlines() if line.strip()] == fields

    cmake = (OPEN3D_LOC / 'CMakeLists.txt').read_text()
    for name in expected:
        assert f'"srv/{name}"' in cmake


def test_navigation_service_node_uses_pose_topics_without_confidence():
    source = (OPEN3D_LOC / 'src/localization_service_node.cpp').read_text()
    for service_name in (
        '/open3d_loc/relocalize',
        '/open3d_loc/get_pose',
        '/open3d_loc/publish_goal',
        '/open3d_loc/pose_deviation',
    ):
        assert service_name in source
    assert 'current_pose_valid_ = false' in source
    assert 'pose_update_count_ > start_pose_count' in source
    assert 'goal_message.header.frame_id = "map"' in source
    assert 'confidence_topic' not in source
    assert 'min_confidence' not in source

    for profile in ('A2', 'local', 'unitree_go2', 'unitree_go2w'):
        params = load(profile, 'open3d_loc.yaml')['localization_service_node']['ros__parameters']
        assert set(params) == {
            'initialpose_topic',
            'current_pose_topic',
            'goal_topic',
            'relocalize_timeout_sec',
        }
        assert params['initialpose_topic'] == '/initialpose'
        assert params['current_pose_topic'] == '/Odometry_open3d'
        assert params['goal_topic'] == '/goal_pose'
        assert params['relocalize_timeout_sec'] > 0.0


def test_runtime_management_interfaces_are_explicit_and_minimal():
    source = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    package = (ROOT / 'package.xml').read_text()
    for legacy in (
        'PlanningRequest',
        'PlannerStatus',
        'ControllerCommand',
        'cmd_vel',
    ):
        assert legacy not in source
    assert 'rosidl_generate_interfaces' in cmake
    assert 'rosidl_default_generators' in package
    for rel_path in (
        'msg/LocalizationStatus.msg',
        'msg/NavigationStatus.msg',
        'msg/MapStatus.msg',
        'srv/SwitchMap.srv',
        'srv/RestartNavigation.srv',
    ):
        assert (ROOT / rel_path).exists()


def test_scan_mode2_is_dynamic_and_has_no_mandatory_sequence():
    header = (SCAN_MANAGE / 'include/plan_manage/scan_replan_fsm.h').read_text()
    source = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    cmake = (SCAN_MANAGE / 'CMakeLists.txt').read_text()
    assert 'WAYPOINT_PATH = 2' in header
    assert 'dynamicWaypointCallback' in source
    assert '"waypoints", rclcpp::QoS(1).reliable().transient_local()' in source
    assert 'planGlobalTrajByWaypoints(waypoints)' in source
    assert 'cancelWaypointNavigation' in source
    assert 'PRESET_TARGET' not in source
    assert 'planNextWaypoint' not in source
    assert 'fsm.waypoints' not in source
    assert 'keypoint_recorder.py' not in cmake


def test_coordinator_publishes_each_complete_path_without_odom_rolling():
    source = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    assert 'publishWaypointPath(sampled)' in source
    assert 'signature == path_signature_' in source
    assert 'rclcpp::QoS(1).reliable().transient_local()' in source
    for removed in (
        'nav_msgs/msg/odometry.hpp',
        'odomCallback',
        'publishRemainingWaypoints',
        'consumedWaypointCount',
        'accumulated_distance_',
        'goal_tolerance',
    ):
        assert removed not in source


def test_scan_mode2_preserves_global_reference_and_uses_arc_lookahead():
    source = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    replan = source.split(
        'bool SCANReplanFSM::planFromCurrentTraj()', 1
    )[1].split(
        'void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()', 1
    )[0]
    target = source.split(
        'bool SCANReplanFSM::getLocalTarget()', 1
    )[1]

    assert 'planGlobalTrajWaypoints' not in replan
    assert 'navi_mode_ != NAVI_MODE::WAYPOINT_PATH' in replan
    assert 'global.last_progress_time_ = projection_time' in target
    assert 'progress_arc_length_ += projection_arc' in target
    assert 'std::min(planning_horizon_, planner_manager_->pp_.planning_horizon_)' in target
    assert 'target_arc + segment_length >= lookahead' in target
    assert 'map->isInMap(local_target_pt_)' in target
    assert 'candidate_time -= time_step' in target
    assert 'candidate_time +=' not in target


def test_goal_completion_is_latched_without_new_status_topics():
    coordinator = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    fsm = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    controller = (SCAN_MANAGE / 'src/closed_loop_controller.cpp').read_text()
    assert 'route_active_' in coordinator
    assert 'route_completed_' not in coordinator
    assert 'goalReached()' in fsm
    assert 'position_error > finish_dist_ + no_replan_thresh_' in fsm
    assert 'bspline.yaw_pts.push_back(end_yaw_)' in fsm
    assert 'task_completed_' in controller
    assert 'final_yaw_ - odom_yaw_' in controller


def test_soft_reset_clears_coordinator_route_and_is_idempotent():
    coordinator = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    manager = (ROOT / 'scripts/nav_manager_node.py').read_text()
    fsm = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    assert '"~/reset_route"' in coordinator
    assert 'clearRoute("route reset requested")' in coordinator
    assert 'self.coordinator_reset_cli' in manager
    assert 'self._call_service(self.coordinator_reset_cli, req)' in manager
    assert 'const bool was_active = have_target_ || !active_waypoints_.empty();' in fsm
    assert 'if (was_active && have_odom_)' in fsm


def test_direct_global_path_follow_chain_is_preserved():
    launch_text = (ROOT / 'launch/global_path_follow_test.launch.py').read_text()
    cmake_text = (ROOT / 'CMakeLists.txt').read_text()
    assert "executable='pct_global_path_follow_coordinator'" in launch_text
    assert "package='pure_pursuit_planner'" in launch_text
    assert 'scripts/global_path_follow_coordinator_node.py' in cmake_text
