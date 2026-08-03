# KN 导航工作区

基于 Ubuntu 22.04 / ROS 2 Humble，提供两条导航链路：

- PCT 全局规划 + SCAN-Planner 局部规划（默认，支持 Go2 / Go2-W 配置）。
- PCT 全局规划 + Pure Pursuit 直接跟踪（测试链路，不经过局部避障）。

## 功能包

工作区内需要编译的包：

- `fast_lio`、`open3d_loc`：里程计与点云定位。
- `pct_planner`：PCT 三维全局规划。
- `plan_env`、`path_searching`、`bspline_opt`、`traj_utils`、`scan_planner`：SCAN 局部规划与控制。
- `pure_pursuit_planner`：Pure Pursuit 和 Unitree Go2 速度桥。
- `pct_scan_navigation`：整套链路的启动、配置与任务协调。

## 安装

先安装 ROS 2 Humble，以及与雷达对应的 `livox_ros_driver2`。然后安装常规依赖：

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pip python3-rosdep \
  libeigen3-dev libpcl-dev libopencv-dev libboost-filesystem-dev \
  libboost-system-dev

cd ~/xq/code/work_space/kn_nav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "livox_ros_driver2 unitree_sdk2"
python3 -m pip install  open3d numpy scipy
```

另外需要：

- 安装 Unitree SDK2，并保证 CMake 能从 `/opt/unitree_robotics` 找到它。
- 安装 Open3D C++ SDK，并设置open3d_loc的CMakeList.txt `Open3D_DIR` 为其 `lib/cmake/Open3D`。
- 首次使用 PCT 时执行 `src/PCT_planner/planner/build_thirdparty.sh` 和 `build.sh`。

## 编译

```bash
cd ~/xq/code/work_space/kn_nav_ws
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/opt/unitree_robotics:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH
export Open3D_DIR=/path/to/open3d/lib/cmake/Open3D
colcon build --symlink-install
source install/setup.bash
```

## 使用

启动 PCT + SCAN（本机配置）：

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py
```

实机 Go2 / Go2-W：

```bash
ros2 launch pct_scan_navigation unitree_go2_pct_scan_navigation.launch.py network_interface:=eth0
# 或
ros2 launch pct_scan_navigation unitree_go2w_pct_scan_navigation.launch.py network_interface:=eth0
```

直接跟踪 PCT 全局路径（无局部避障，默认不启动底盘桥）：

```bash
ros2 launch pct_scan_navigation global_path_follow_test.launch.py \
  network_interface:=eth0 start_go2_bridge:=false
```

确认定位、路径和场地安全后，启动底盘桥时传入 `start_go2_bridge:=true`，再使能：

```bash
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool '{data: true}'
```

常用检查：

```bash
ros2 topic echo /pct_path
ros2 topic echo /pct_scan_navigation/status
ros2 service call /pct_scan_navigation/cancel std_srvs/srv/Trigger '{}'
```

地图路径、雷达话题、网卡和速度限制在 `src/pct_scan_navigation/config/` 中按机器人配置修改。
