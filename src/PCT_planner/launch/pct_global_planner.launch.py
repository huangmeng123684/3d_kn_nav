# =============================================================================
# PCT Global Planner — launch file
#
# Usage:
#   ros2 launch pct_planner pct_global_planner.launch.py
#
# Edit params in: params/pct_global_planner.yaml
# =============================================================================

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('pct_planner')
    default_params = os.path.join(pkg_share, 'params', 'pct_global_planner.yaml')

    params_file = LaunchConfiguration('params_file', default=default_params)

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the YAML parameter file for pct_global_planner')

    node = Node(
        package='pct_planner',
        executable='run_ros2_global_planner',
        name='pct_global_planner',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        declare_params,
        node,
    ])
