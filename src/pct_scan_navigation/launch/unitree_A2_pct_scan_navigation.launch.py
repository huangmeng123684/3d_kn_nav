"""Unitree Go2-W bringup for PCT global planning with SCAN local navigation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_launch = PathJoinSubstitution([
        FindPackageShare('pct_scan_navigation'),
        'launch',
        'local_pct_scan_navigation.launch.py',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('navigation_mode', default_value='2'),
        DeclareLaunchArgument('start_open3d_loc', default_value='true'),
        DeclareLaunchArgument('start_pct_planner', default_value='true'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'config_profile': 'A2',
                'navigation_mode': LaunchConfiguration('navigation_mode'),
                'start_go2_bridge': 'true',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'start_open3d_loc': LaunchConfiguration('start_open3d_loc'),
                'start_pct_planner': LaunchConfiguration('start_pct_planner'),
                'network_interface': LaunchConfiguration('network_interface'),
            }.items(),
        ),
    ])
