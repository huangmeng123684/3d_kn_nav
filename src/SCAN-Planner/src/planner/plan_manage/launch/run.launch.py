"""Real-sensor SCAN-Planner bringup without simulation components."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    default_params = os.path.join(scan_share, "config", "scan_planner.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("body_pose_topic", default_value="/Odometry_open3d"),
        DeclareLaunchArgument("sensor_pose_topic", default_value="/Odometry_open3d"),
        DeclareLaunchArgument("cloud_topic", default_value="/scan_map"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        # Keep real-hardware motion opt-in.  The planner and RViz can be tested
        # without starting a velocity publisher.
        DeclareLaunchArgument("start_controller", default_value="true"),
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("body_pose", LaunchConfiguration("body_pose_topic")),
                ("sensor_pose", LaunchConfiguration("sensor_pose_topic")),
                ("cloud", LaunchConfiguration("cloud_topic")),
                ("move_base_simple/goal", "/goal_pose"),
                ("waypoints", "/scan_planner/waypoints"),
                ("initial_path", "/initial_path"),
            ],
        ),
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_controller")),
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("body_pose", LaunchConfiguration("body_pose_topic")),
                ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
            ],
        ),
    ])
