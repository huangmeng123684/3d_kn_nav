"""Parallel bringup for PCT global planning with SCAN-Planner local navigation."""

import os
import shutil
import time
from pathlib import Path

import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _workspace_log_root():
    override = os.environ.get('KN_NAV_WS_LOG_DIR')
    if override:
        return Path(override).expanduser().resolve()

    share = Path(get_package_share_directory('pct_scan_navigation')).resolve()
    parts = share.parts
    if 'install' in parts:
        workspace = Path(*parts[:parts.index('install')])
        return workspace / 'src' / 'log'
    if share.name == 'pct_scan_navigation' and share.parent.name == 'src':
        return share.parent / 'log'
    return share.parent / 'log'


def _prepare_log_directory(context):
    # launch configures its console log before executing this OpaqueFunction.
    # Preserve that path so stdout/stderr (including std::cout and printf) can
    # be reached from the per-run workspace log directory as well.
    launch_log_file = Path(launch.logging.launch_config.log_dir).resolve() / 'launch.log'

    log_root = _workspace_log_root()
    log_root.mkdir(parents=True, exist_ok=True)

    run_dir = log_root / time.strftime('run_%Y%m%d_%H%M%S')
    suffix = 0
    while run_dir.exists():
        suffix += 1
        run_dir = log_root / f"{time.strftime('run_%Y%m%d_%H%M%S')}_{suffix}"
    run_dir.mkdir(parents=True, exist_ok=True)

    console_log_link = run_dir / 'launch.log'
    try:
        # The target may not exist yet; launch creates/writes it as processes
        # start, and the symlink becomes usable immediately afterwards.
        console_log_link.symlink_to(launch_log_file)
        print(
            f"[pct_scan_navigation] Full console log: "
            f"{console_log_link} -> {launch_log_file}"
        )
    except OSError as error:
        print(f"[pct_scan_navigation] Unable to link full console log: {error}")

    runs = sorted(
        [path for path in log_root.iterdir() if path.is_dir() and path.name.startswith('run_')],
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    for old_run in runs[20:]:
        shutil.rmtree(old_run, ignore_errors=True)

    latest = log_root / 'latest'
    try:
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        latest.symlink_to(run_dir.name)
    except OSError:
        pass

    print(f"[pct_scan_navigation] ROS logs: {run_dir}")
    launch.logging.launch_config.log_dir = str(run_dir)
    return [
        SetEnvironmentVariable('ROS_LOG_DIR', str(run_dir)),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '0'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_open3d_loc = LaunchConfiguration('start_open3d_loc')
    start_pct_planner = LaunchConfiguration('start_pct_planner')
    start_go2_bridge = LaunchConfiguration('start_go2_bridge')
    network_interface = LaunchConfiguration('network_interface')
    scan_params_file = LaunchConfiguration('scan_params_file')
    coordinator_params_file = LaunchConfiguration('coordinator_params_file')
    pct_params_file = LaunchConfiguration('pct_params_file')
    map_profiles_file = LaunchConfiguration('map_profiles_file')
    full_restart_command = LaunchConfiguration('full_restart_command')
    config_profile = LaunchConfiguration('config_profile')
    navigation_mode = LaunchConfiguration('navigation_mode')
    navigation_mode_value = ParameterValue(navigation_mode, value_type=int)

    # navigation_share = FindPackageShare('pct_scan_navigation')
    navigation_share = str(Path.home() / 'nav_map')

    def navigation_config(name):
        return PathJoinSubstitution([navigation_share, 'config', config_profile, name])

    fast_lio = Node(
        package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
        output='both', emulate_tty=True,
        parameters=[navigation_config('fast_lio.yaml'), {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_open3d_loc),
    )
    open3d_global = Node(
        package='open3d_loc', executable='global_localization_node',
        name='global_localization_node', output='both',
        parameters=[navigation_config('open3d_loc.yaml'), {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_open3d_loc),
    )
    open3d_service = Node(
        package='open3d_loc', executable='localization_service_node',
        name='localization_service_node', output='both',
        parameters=[navigation_config('open3d_loc.yaml'), {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_open3d_loc),
    )
    pct_planner = Node(
        package='pct_planner', executable='run_ros2_global_planner',
        name='pct_global_planner', output='both',
        parameters=[pct_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression([
            "'", navigation_mode, "' == '2' and '", start_pct_planner, "' == 'true'",
        ])),
    )
    scan_planner = Node(
        package='scan_planner', executable='scan_planner_node',
        name='scan_planner_node', output='both',
        parameters=[scan_params_file, {
            'fsm.navi_mode': navigation_mode_value,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('body_pose', '/Odometry_open3d'),
            ('sensor_pose', '/Odometry_open3d'),
            ('cloud', '/scan_map'),
            ('move_base_simple/goal', '/goal_pose'),
            ('waypoints', '/scan_planner/waypoints'),
            ('initial_path', '/initial_path'),
        ],
    )
    controller = Node(
        package='scan_planner', executable='closed_loop_controller',
        name='closed_loop_controller', output='both',
        parameters=[scan_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('body_pose', '/Odometry_open3d'),],

    )
    coordinator = Node(
        package='pct_scan_navigation', executable='pct_scan_coordinator',
        name='pct_scan_coordinator', output='both',
        parameters=[coordinator_params_file, {
            'mode': navigation_mode_value,
            'use_sim_time': use_sim_time,
        }],
        on_exit=Shutdown(reason='pct_scan_coordinator exited'),
    )
    nav_manager = Node(
        package='pct_scan_navigation', executable='nav_manager_node.py',
        name='nav_manager_node', output='both',
        parameters=[{
            'map_profiles_path': map_profiles_file,
            'initial_map_name': 'outdoor',
            'full_restart_command': full_restart_command,
            'use_sim_time': use_sim_time,
        }],
    )
    bridge = Node(
        package='pure_pursuit_planner', executable='go2_cmd_vel_bridge',
        name='go2_cmd_vel_bridge', output='both',
        parameters=[navigation_config('go2_bridge.yaml'), {
            'network_interface': network_interface,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(start_go2_bridge),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'navigation_mode', default_value='2',
            description='1: direct RViz goal to SCAN, 2: complete PCT reference path; 3 unsupported',
        ),
        DeclareLaunchArgument(
            'config_profile', default_value='local',
            description='Configuration directory under pct_scan_navigation/config',
        ),
        DeclareLaunchArgument('start_open3d_loc', default_value='true'),
        DeclareLaunchArgument('start_pct_planner', default_value='true'),
        DeclareLaunchArgument('start_go2_bridge', default_value='false'),
        DeclareLaunchArgument('network_interface', default_value='enp2s0'),
        DeclareLaunchArgument('scan_params_file', default_value=navigation_config('scan_planner.yaml')),
        DeclareLaunchArgument('coordinator_params_file', default_value=navigation_config('coordinator.yaml')),
        DeclareLaunchArgument('pct_params_file', default_value=navigation_config('pct_global_planner.yaml')),
        DeclareLaunchArgument('map_profiles_file', default_value=navigation_config('map_profiles.yaml')),
        DeclareLaunchArgument('full_restart_command', default_value=''),
        OpaqueFunction(function=_prepare_log_directory),
        SetEnvironmentVariable('LD_LIBRARY_PATH', [
            '/opt/unitree_robotics/lib:',
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ]),
        fast_lio,
        open3d_global,
        open3d_service,
        pct_planner,
        scan_planner,
        controller,
        coordinator,
        nav_manager,
        bridge,
    ])
