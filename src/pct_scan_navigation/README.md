# pct_scan_navigation

PCT 全局规划与 SCAN-Planner 局部导航的轻量协调和统一启动包。

## Build

```bash
cd work_space/kn_nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to pct_scan_navigation
source install/setup.bash
```

## Mode 1: direct SCAN goal

RViz `/goal_pose` 直接交给 SCAN-Planner，不启动 PCT 全局规划器：

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py \
  navigation_mode:=1
```

## Mode 2: complete PCT reference path

Mode 2 是默认模式。PCT 发布 `/pct_path`，coordinator 沿三维路径每隔 1 m
提取 waypoint，并把每条新路径的完整采样点列一次性发布到
`/scan_planner/waypoints`。SCAN 保存完整参考路径，通过单调路径投影和三维
弧长前视管理进度；中间点仅用于塑造参考轨迹，最后一点始终是唯一导航终点。
到达终点后 SCAN 原地调整到路径最终姿态；位置和朝向均达标后锁定停止，手动
移开不会重新返回。

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py
```

可通过 coordinator 配置调整：

- `waypoint_spacing`：三维弧长采样间距，默认 `1.0` m；
- `waypoint_z_offset`：发布 waypoint 的统一高度偏移，默认 `0.0` m；

SCAN 默认使用 `0.15` m 位置容差和 `0.10` rad 朝向容差。

Mode 3 尚未实现，选择 `navigation_mode:=3` 会使启动立即失败并关闭导航。

## Navigation services

使用 `map` 下的 XYZ 和四元数触发重定位。服务会等待新的
`/Odometry_open3d`，默认最多等待 10 秒：

```bash
ros2 service call /open3d_loc/relocalize open3d_loc/srv/Relocalize \
  "{x: 1.0, y: 2.0, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

获取 `base_link` 在 `map` 下的最新位姿：

```bash
ros2 service call /open3d_loc/get_pose open3d_loc/srv/GetPose "{}"
```

向 `/goal_pose` 发布一个导航点。返回成功仅表示消息已经发布，不表示导航完成：

```bash
ros2 service call /open3d_loc/publish_goal open3d_loc/srv/PublishGoal \
  "{x: 3.0, y: 1.0, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

## Robot profiles

配置位于 `config/local`、`config/unitree_go2` 和 `config/unitree_go2w`。
本地启动默认关闭底盘桥；Go2/Go2-W 实机启动文件默认启动桥，但仍需按照底盘
桥自身的安全接口完成硬件使能。

`global_path_follow_test.launch.py` 保留为 PCT + Pure Pursuit 直接跟踪测试；该链路
不使用 SCAN 局部规划和避障。
