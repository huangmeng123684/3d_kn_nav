# =============================================================================
# PCT + Pure Pursuit Planner — integrated launch
#
# Architecture:
#   PCT planner → /pct_path (nav_msgs/Path, frame_id=map)
#   open3d_loc → /Odometry_open3d (nav_msgs/Odometry, map→base_link)
#        ↓
#   pure_pursuit_planner → /cmd_vel
#        ↓
#   go2_cmd_vel_bridge → SportClient::Move
#
# Usage:
#   ros2 launch pure_pursuit_planner pct_pure_pursuit.launch.py \
#     network_interface:=eth0
# =============================================================================

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('pure_pursuit_planner')
    config_path = os.path.join(pkg_dir, 'config', 'pct_params.yaml')
    bridge_config_path = os.path.join(pkg_dir, 'config', 'go2_bridge_params.yaml')

    network_interface_argument = DeclareLaunchArgument(
        'network_interface',
        default_value='eth0',
        description='Network interface connected to the Go2, e.g. eth0',
    )

    pure_pursuit_node = Node(
        package='pure_pursuit_planner',
        executable='pure_pursuit_planner',
        name='pure_pursuit_node',
        output='screen',
        parameters=[config_path],
    )

    go2_bridge_node = Node(
        package='pure_pursuit_planner',
        executable='go2_cmd_vel_bridge',
        name='go2_cmd_vel_bridge',
        output='screen',
        parameters=[
            bridge_config_path,
            {'network_interface': LaunchConfiguration('network_interface')},
        ],
    )

    return LaunchDescription([
        network_interface_argument,
        pure_pursuit_node,
        go2_bridge_node,
    ])
