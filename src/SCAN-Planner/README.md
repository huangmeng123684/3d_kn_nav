# SCAN-Planner ROS 2 — real-sensor edition

This repository contains the SCAN local planner for real sensor input.  It
keeps the occupancy map, dynamic A* search, B-spline optimization, RViz goal
interface, and optional closed-loop velocity controller.  All simulator,
synthetic sensing, Gazebo, and robot-model packages have been removed.

## Build

From the ROS 2 workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  plan_env path_searching bspline_opt traj_utils scan_planner_msgs scan_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## FAST-LIO + Open3D RViz test

Start FAST-LIO and Open3D localization first.  SCAN expects:

- `/Odometry_open3d` (`nav_msgs/msg/Odometry`) in the `map` frame;
- `/scan_map` (`sensor_msgs/msg/PointCloud2`) with points already in `map`.

Then run:

```bash
ros2 launch scan_planner open3d_rviz.launch.py
```

It starts only `scan_planner_node` and RViz.  The RViz `2D Goal Pose` tool
publishes `/move_base_simple/goal`; no velocity controller is started.

## Real-controller bringup

`run.launch.py` uses the same Open3D topics by default.  Its controller is
disabled by default and must be explicitly enabled when a safe `/cmd_vel`
consumer is available:

```bash
ros2 launch scan_planner run.launch.py start_controller:=true
```

Topics can be substituted without changing source code:

```bash
ros2 launch scan_planner run.launch.py \
  body_pose_topic:=/your/odom \
  sensor_pose_topic:=/your/sensor_odom \
  cloud_topic:=/your/map_frame_cloud \
  cmd_vel_topic:=/cmd_vel
```

## License and attribution

The planning algorithm originates from SCAN-Planner by Han Zheng, Zhe Chen,
Yiwen Fu, Ming Yang, and Tong Qin.  This ROS 2 adaptation remains under
Apache-2.0; retain the notices in [NOTICE](NOTICE).
