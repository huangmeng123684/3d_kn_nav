"""Verify that dynamic waypoint mode starts without static fsm.waypoints."""

import os
import time

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "scan_planner.yaml")
    )
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[config, {"fsm.navi_mode": 2}],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


class TestWaypointModeStartup:
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)

    def test_local_target_marker_topic(self):
        rclpy.init()
        node = rclpy.create_node("local_target_topic_test")
        try:
            deadline = time.monotonic() + 10.0
            topic_types = {}
            while time.monotonic() < deadline:
                topic_types = dict(node.get_topic_names_and_types())
                if "/scan_planner/local_target" in topic_types:
                    break
                rclpy.spin_once(node, timeout_sec=0.1)

            assert topic_types.get("/scan_planner/local_target") == [
                "visualization_msgs/msg/Marker"
            ]
        finally:
            node.destroy_node()
            rclpy.shutdown()
