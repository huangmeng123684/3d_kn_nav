"""Launch PCT global path following without local ART/ROG planning."""

import datetime
import os
from pathlib import Path
import shutil
import socket

from launch import LaunchDescription
import launch.logging
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# CONFIG_NAME = 'unitree'
CONFIG_NAME = 'local'
MAX_LOG_SESSIONS = 50


def _source_package_dir():
    launch_file = Path(__file__).resolve()
    if launch_file.parents[1].name == 'pct_scan_navigation':
        return launch_file.parents[1]

    parts = launch_file.parts
    if 'install' in parts:
        install_index = parts.index('install')
        workspace = Path(*parts[:install_index])
        source_dir = workspace / 'src' / 'pct_scan_navigation'
        if source_dir.exists():
            return source_dir

    cwd_source_dir = Path.cwd() / 'src' / 'pct_scan_navigation'
    if cwd_source_dir.exists():
        return cwd_source_dir

    return launch_file.parents[1]


def _prepare_log_dir():
    log_root = _source_package_dir() / 'log'
    log_root.mkdir(parents=True, exist_ok=True)

    sessions = [path for path in log_root.iterdir() if path.is_dir()]
    sessions.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    for old_session in sessions[MAX_LOG_SESSIONS - 1:]:
        shutil.rmtree(old_session, ignore_errors=True)

    timestamp = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')
    session_dir = log_root / f'{timestamp}-{socket.gethostname()}-{os.getpid()}'
    session_dir.mkdir(parents=True, exist_ok=False)
    launch.logging.launch_config.log_dir = str(session_dir)
    os.environ['ROS_LOG_DIR'] = str(session_dir)
    return str(session_dir)


def log_process_start(node, label):
    return RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[LogInfo(msg=f'[global path follow test] started {label}')],
        )
    )


def generate_launch_description():
    log_dir = _prepare_log_dir()
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_open3d_loc = LaunchConfiguration('start_open3d_loc')
    start_pct_planner = LaunchConfiguration('start_pct_planner')
    start_go2_bridge = LaunchConfiguration('start_go2_bridge')
    pct_params_file = LaunchConfiguration('pct_params_file')
    network_interface = LaunchConfiguration('network_interface')
    package_share = FindPackageShare('pct_scan_navigation')

    def config_file(filename):
        return PathJoinSubstitution([package_share, 'config', CONFIG_NAME, filename])

    fast_lio_params = config_file('fast_lio.yaml')
    open3d_loc_params = config_file('open3d_loc.yaml')
    pct_default_params = config_file('pct_global_planner.yaml')
    follow_coordinator_params = config_file('global_path_follow_coordinator.yaml')
    pursuit_params = config_file('pure_pursuit.yaml')
    bridge_params = config_file('go2_bridge.yaml')

    unitree_runtime_environment = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        [
            '/opt/unitree_robotics/lib',
            ':',
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ],
    )
    ros_log_environment = SetEnvironmentVariable('ROS_LOG_DIR', log_dir)

    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='both',
        parameters=[fast_lio_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_open3d_loc),
        emulate_tty=True,
    )

    open3d_global_localization = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='both',
        parameters=[open3d_loc_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_open3d_loc),
    )

    open3d_localization_service = Node(
        package='open3d_loc',
        executable='localization_service_node',
        name='localization_service_node',
        output='both',
        parameters=[open3d_loc_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_open3d_loc),
    )

    pct_planner = Node(
        package='pct_planner',
        executable='run_ros2_global_planner',
        name='pct_global_planner',
        output='both',
        parameters=[pct_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_pct_planner),
    )

    global_path_follow_coordinator = Node(
        package='pct_scan_navigation',
        executable='pct_global_path_follow_coordinator',
        name='pct_global_path_follow_coordinator',
        output='both',
        parameters=[follow_coordinator_params, {'use_sim_time': use_sim_time}],
    )

    pure_pursuit = Node(
        package='pure_pursuit_planner',
        executable='pure_pursuit_planner',
        name='pure_pursuit_node',
        output='both',
        parameters=[pursuit_params, {'use_sim_time': use_sim_time}],
        remappings=[('/pct_path', '/global_path_follow/path')],
    )

    go2_bridge = Node(
        package='pure_pursuit_planner',
        executable='go2_cmd_vel_bridge',
        name='go2_cmd_vel_bridge',
        output='both',
        parameters=[
            bridge_params,
            {'network_interface': network_interface, 'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(start_go2_bridge),
    )

    startup_summary = LogInfo(
        msg=[
            '[global path follow test] launching PCT -> Global Path Coordinator '
            '-> Pure Pursuit -> Go2 bridge '
            'with config/',
            CONFIG_NAME,
            '. No pct_art_coordinator, ROG local planner, /local_goal, or /local_path. '
            'The global path coordinator clears the path after final pose completion. '
            'Logs=',
            log_dir,
            '. FAST-LIO + Open3D localization start=',
            start_open3d_loc,
            ', Go2 bridge start=',
            start_go2_bridge,
            '.',
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_open3d_loc', default_value='true'),
        DeclareLaunchArgument('start_pct_planner', default_value='true'),
        DeclareLaunchArgument('start_go2_bridge', default_value='false'),
        DeclareLaunchArgument(
            'network_interface',
            default_value='eth0',
            description='Network interface connected to the Go2',
        ),
        DeclareLaunchArgument(
            'pct_params_file',
            default_value=pct_default_params,
        ),
        ros_log_environment,
        unitree_runtime_environment,
        startup_summary,
        log_process_start(
            fast_lio,
            'fast_lio (/Odometry_loc, /cloud_registered_body_1)',
        ),
        log_process_start(
            open3d_global_localization,
            'open3d global localization (/Odometry_open3d)',
        ),
        log_process_start(open3d_localization_service, 'open3d localization service'),
        log_process_start(pct_planner, 'pct_global_planner (/pct_path)'),
        log_process_start(
            global_path_follow_coordinator,
            'pct_global_path_follow_coordinator (/global_path_follow/path)',
        ),
        log_process_start(
            pure_pursuit,
            'pure_pursuit_node (/cmd_vel from /global_path_follow/path)',
        ),
        log_process_start(
            go2_bridge,
            'go2_cmd_vel_bridge (/go2_cmd_vel_bridge/enable)',
        ),
        fast_lio,
        open3d_global_localization,
        open3d_localization_service,
        pct_planner,
        global_path_follow_coordinator,
        pure_pursuit,
        go2_bridge,
    ])
