# kn_nav 交接文档（操作版）

> 文档日期：2026-08-13  
> 宿主机项目目录：`/home/jetson/kn_nav`  
> 容器内项目目录：`/workspace/kn_nav_ws`  
> 当前导航容器：`kn_nav_container`  
> 当前镜像：`kn_nav:v1`

## 1. 环境介绍

### 1.1 运行环境

当前项目运行在 Jetson ARM64 主机的 Docker 容器中。

| 项目 | 当前实际配置 |
|---|---|
| 宿主机系统 | Ubuntu 20.04.6 LTS，`aarch64` |
| Docker | 28.1.1 |
| 导航容器系统 | Ubuntu 22.04.5 LTS，`aarch64` |
| ROS 2 | Humble |
| ROS Domain | `ROS_DOMAIN_ID=0` |
| 导航镜像 | `kn_nav:v1`，`linux/arm64` |
| 导航容器 | `kn_nav_container` |
| 容器网络 | host 网络 |
| ROS 1/2 桥 | `ros:foxy-ros1-bridge`，Noetic + Foxy |
| 机器人 | Unitree Go2 |
| 雷达输入 | `/livox/lidar/pointcloud` |
| IMU 输入 | `/livox/imu` |
| 当前地图 | `clean_data.pcd` + `clean_data.pickle` |

宿主机和容器之间的挂载：

```text
/home/jetson/kn_nav  -> /workspace/kn_nav_ws
/dev/shm             -> /dev/shm
```

项目目录是读写挂载。容器内修改 `/workspace/kn_nav_ws`，会直接修改宿主机 `/home/jetson/kn_nav`。

当前导航链路：

```text
Livox ROS 1 数据
    ↓ ros1_bridge
FAST-LIO 里程计和局部点云
    ↓
Open3D 离线地图定位
    ↓ map -> base_link
PCT 全局规划
    ↓ /pct_path
SCAN-Planner 局部规划和控制
    ↓ /cmd_vel
Go2 WebRTC bridge
    ↓
Unitree Go2
```

### 1.2 编译环境

容器内已经安装并验证的主要环境：

| 依赖 | 当前状态/路径 |
|---|---|
| ROS 2 Humble | `/opt/ros/humble` |
| Open3D C++ SDK | `/opt/open3d141/lib/cmake/Open3D` |
| PCT 第三方库 | `/opt/pct-install` |
| Unitree SDK/运行库 | 镜像内已安装 |
| Python | Python 3 |
| Python Open3D | 0.18.0 |
| NumPy | 2.2.6 |
| SciPy | 1.15.3 |
| 工作区 install | `/workspace/kn_nav_ws/install` |

每个新容器终端都要加载：

```bash
cd /workspace/kn_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export Open3D_DIR=/opt/open3d141/lib/cmake/Open3D
export THIRDPARTY_ROOT=/opt/pct-install
export ROS_DOMAIN_ID=0
```

当前工作区已编译的主要 ROS 包：

- `fast_lio`；
- `open3d_loc`；
- `pct_planner`；
- `pct_scan_navigation`；
- `scan_planner`；
- `plan_env`；
- `path_searching`；
- `bspline_opt`；
- `traj_utils`；
- `scan_planner_msgs`；
- `livox_ros_driver2`。

`src/pure_pursuit_planner` 下存在 `COLCON_IGNORE`，当前没有参与编译，也不是现用主导航链路。

重新编译工作区：

```bash
cd /workspace/kn_nav_ws
source /opt/ros/humble/setup.bash
export Open3D_DIR=/opt/open3d141/lib/cmake/Open3D
export THIRDPARTY_ROOT=/opt/pct-install

colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DOpen3D_DIR=/opt/open3d141/lib/cmake/Open3D

source install/setup.bash
```

如果修改了 PCT 的 C++/pybind 源码，还需先执行：

```bash
cd /workspace/kn_nav_ws/src/PCT_planner/planner
bash build.sh
```

只有首次安装或修改 GTSAM、OSQP 等第三方库时，才执行：

```bash
cd /workspace/kn_nav_ws/src/PCT_planner/planner
bash build_thirdparty.sh
bash build.sh
```

### 1.3 现在的进度

当前状态：**PCT + SCAN 主导航链路可以启动并进行仿真和真机调试，但镜像复现、地图归档和正式安全模式验收仍需补齐。**

已经完成：

1. FAST-LIO 接收桥接后的 Livox PointCloud2 和 IMU 数据；
2. Open3D 加载离线 PCD 并发布定位；
3. PCT 加载 tomogram，发布 `/pct_path`；
4. coordinator 将 PCT 路径传给 SCAN-Planner；
5. SCAN-Planner 输出 `/cmd_vel`；
6. Go2 WebRTC bridge 支持正式安全模式和 bypass 调试模式；
7. 当前容器已编译核心包；
8. 已有仿真、真机启动和运行日志。

当前主配置：

```text
/home/jetson/kn_nav/src/pct_scan_navigation/config/unitree_go2/
```

当前地图：

```text
PCD：/home/jetson/kn_nav/src/PCT_planner/rsc/pcd/clean_data.pcd
Tomogram：/home/jetson/kn_nav/src/PCT_planner/rsc/tomogram/clean_data.pickle
```

未完成或待补充：

1. 当前 `kn_nav:v1` 是 ARM64 镜像，但仓库没有对应的 ARM64 Dockerfile；
2. 当前地图 PCD 和 pickle 被 Git 忽略，重新 clone 后不会自动获得；
3. 镜像仓库地址、登录方式和 digest 尚未提供；
4. 正式模式 `bypass_safety:=false` 需要补完整的端到端验收记录；
5. 4G WebRTC 下 `sport_state_timeout=0.5` 秒可能导致自动 disarm；
6. `navigation_mode:=3` 未实现；
7. Web API 文件存在，但当前容器没有安装 FastAPI/Uvicorn；
8. 建图流程具备 FAST-LIO 和 `/map_save` 能力，但没有完整的场地级建图验收记录；
9. 雷达、IMU、机器人序列号及外参标定记录待补充。

## 2. 操作文档

### 2.1 构建镜像

#### 2.1.1 当前镜像说明

当前生产镜像：

```text
kn_nav:v1
架构：linux/arm64
```

仓库中的 `docker/amd/Dockerfile` 是 amd64 参考文件，不能直接用来重建当前 Jetson ARM64 镜像。

#### 2.1.2 从镜像仓库拉取

当前尚未提供实际镜像仓库地址。补齐后执行：

```bash
docker login <镜像仓库域名>
docker pull <镜像仓库域名>/<项目>/kn_nav:v1-arm64

docker image inspect <镜像仓库域名>/<项目>/kn_nav:v1-arm64 \
  --format '{{.Id}} {{.Os}}/{{.Architecture}}'

docker tag \
  <镜像仓库域名>/<项目>/kn_nav:v1-arm64 \
  kn_nav:v1
```

必须确认输出为 `linux/arm64`。不要直接复制尖括号占位符执行。

#### 2.1.3 从离线 tar 导入

```bash
cd <镜像文件所在目录>
sha256sum -c kn_nav_v1_arm64.tar.sha256
docker load -i kn_nav_v1_arm64.tar
docker image ls | grep kn_nav
```

如果导入后的标签不是 `kn_nav:v1`：

```bash
docker tag <导入后的镜像名:标签> kn_nav:v1
```

验证：

```bash
docker image inspect kn_nav:v1 \
  --format '{{.Id}} {{.Os}}/{{.Architecture}}'
```

#### 2.1.4 从当前容器生成应急镜像

当没有 ARM64 Dockerfile 时，可以用 `docker commit` 保存当前容器内的系统依赖：

```bash
docker commit kn_nav_container kn_nav:v1-arm64-backup-20260813

docker image inspect kn_nav:v1-arm64-backup-20260813 \
  --format '{{.Id}} {{.Os}}/{{.Architecture}}'
```

导出：

```bash
docker save -o /home/jetson/kn_nav_v1_arm64_backup_20260813.tar \
  kn_nav:v1-arm64-backup-20260813

sha256sum /home/jetson/kn_nav_v1_arm64_backup_20260813.tar \
  > /home/jetson/kn_nav_v1_arm64_backup_20260813.tar.sha256
```

注意：项目目录是 bind mount，`docker commit` 不会把 `/home/jetson/kn_nav` 的代码、地图、build 和 install 内容打进镜像，必须另外备份项目目录和地图。

#### 2.1.5 正式构建 ARM64 镜像

正式交接应先补：

```text
/home/jetson/kn_nav/docker/arm64/Dockerfile
```

补齐后使用：

```bash
cd /home/jetson/kn_nav

docker build \
  --network host \
  --platform linux/arm64 \
  -f docker/arm64/Dockerfile \
  -t kn_nav:v1-arm64 .
```

当前 `docker/arm64/Dockerfile` 尚不存在，以上是应补齐的标准构建入口，不是当前立即可执行的命令。

### 2.2 创建容器

创建前确认镜像、项目和旧容器：

```bash
docker image inspect kn_nav:v1
test -d /home/jetson/kn_nav
docker ps -a --filter name='^/kn_nav_container$'
```

如果已经存在 `kn_nav_container`，不要重复创建，直接执行 2.3 节。

首次创建前复制 Xauthority：

```bash
cp "${XAUTHORITY:-$HOME/.Xauthority}" /home/jetson/kn_nav/xauth_host
```

创建容器：

```bash
docker run -it --network host \
  -e ROS_DOMAIN_ID=0 \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/workspace/kn_nav_ws/xauth_host \
  -v /dev/shm:/dev/shm \
  -v /home/jetson/kn_nav:/workspace/kn_nav_ws \
  --name kn_nav_container \
  kn_nav:v1 \
  bash
```

创建后检查：

```bash
docker inspect kn_nav_container \
  --format 'image={{.Config.Image}} network={{.HostConfig.NetworkMode}} status={{.State.Status}}'

docker inspect kn_nav_container \
  --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}'
```

预期：

```text
image=kn_nav:v1
network=host
/home/jetson/kn_nav -> /workspace/kn_nav_ws
/dev/shm -> /dev/shm
```

### 2.3 进入容器

启动容器：

```bash
docker start kn_nav_container
```

进入普通终端：

```bash
docker exec -it kn_nav_container bash
```

进入可显示 RViz 的终端：

```bash
# 宿主机，每次新的 SSH X11 会话都要重新复制
cp "${XAUTHORITY:-$HOME/.Xauthority}" /home/jetson/kn_nav/xauth_host

docker exec -e DISPLAY=$DISPLAY -it kn_nav_container bash
```

进入后加载环境：

```bash
cd /workspace/kn_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export Open3D_DIR=/opt/open3d141/lib/cmake/Open3D
export THIRDPARTY_ROOT=/opt/pct-install
export ROS_DOMAIN_ID=0
```

### 2.4 启动前检查

#### 2.4.1 宿主机检查

```bash
docker info >/dev/null && echo "Docker OK"
docker image inspect kn_nav:v1 >/dev/null && echo "Image OK"
docker ps -a --filter name='^/kn_nav_container$'

test -s /home/jetson/kn_nav/src/PCT_planner/rsc/pcd/clean_data.pcd \
  && echo "PCD OK"
test -s /home/jetson/kn_nav/src/PCT_planner/rsc/tomogram/clean_data.pickle \
  && echo "Tomogram OK"

df -h /home/jetson/kn_nav
```

#### 2.4.2 Go2 网络检查

当前 Go2 IP：

```text
192.168.123.161
```

配置文件：

```text
/home/jetson/kn_nav/src/pct_scan_navigation/config/unitree_go2/go2_bridge.yaml
```

检查：

```bash
grep -n 'robot_ip' \
  /home/jetson/kn_nav/src/pct_scan_navigation/config/unitree_go2/go2_bridge.yaml
ping -c 3 192.168.123.161
```

当前 Python WebRTC bridge 读取的是 `robot_ip`。只修改 launch 的 `network_interface` 不会改变 Go2 连接地址。

#### 2.4.3 容器依赖检查

```bash
test -f /opt/open3d141/lib/cmake/Open3D/Open3DConfig.cmake \
  && echo "Open3D OK"
test -d /opt/pct-install && echo "PCT third-party OK"
test -f /workspace/kn_nav_ws/install/setup.bash \
  && echo "Workspace install OK"

ros2 pkg prefix fast_lio
ros2 pkg prefix open3d_loc
ros2 pkg prefix pct_planner
ros2 pkg prefix scan_planner
ros2 pkg prefix pct_scan_navigation
```

#### 2.4.4 启动 ROS 1/ROS 2 bridge

先确认 ROS 1 master 和雷达驱动已运行。

宿主机执行：

```bash
IP=$(hostname -I | awk '{print $1}')
echo "IP=$IP"
test -n "$IP" || { echo "ERROR: 未获取到宿主机 IP"; exit 1; }

docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://127.0.0.1:11311 \
  -e ROS_IP=$IP \
  -e ROS_HOSTNAME=$IP \
  -e ROS_DOMAIN_ID=0 \
  -v /dev/shm:/dev/shm \
  --name ros1-foxy-bridge \
  ros:foxy-ros1-bridge \
  bash -lc 'source /opt/ros/noetic/setup.bash && \
            source /opt/ros/foxy/setup.bash && \
            ros2 run ros1_bridge dynamic_bridge --bridge-all-topics'
```

该终端保持运行。命令中的 `ROS_MASTER_URI` 指向 `127.0.0.1:11311`，但命令本身不会启动 `roscore`。

#### 2.4.5 雷达和 IMU 检查

导航容器内执行：

```bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /livox/imu
```

雷达和 IMU 必须持续有数据。无数据时禁止进入真机导航。

### 2.5 导航

#### 2.5.1 第一次先启动无底盘模式

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py \
  config_profile:=unitree_go2 \
  start_open3d_loc:=true \
  start_pct_planner:=true \
  start_go2_bridge:=false \
  navigation_mode:=2
```

另开一个容器终端，加载环境后检查：

```bash
ros2 topic echo /localization_status --once
ros2 topic hz /Odometry_open3d
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /tomogram --once
```

成功标准：

1. `/localization_status` 最终为 `state: 3`；
2. `map -> base_link` 连续，位置和方向与现场一致；
3. RViz 中地图和机器人重合；
4. 发布目标后 `/pct_path` 和 `/scan_planner/waypoints` 非空；
5. 没有持续出现 `no_scan`、定位丢失或 sensor pose 缺失。

#### 2.5.2 启动 RViz

```bash
rviz2 -d \
  /workspace/kn_nav_ws/src/pct_scan_navigation/config/unitree_go2/kn_nav.rviz
```

Fixed Frame 使用 `map`。

#### 2.5.3 无定位静态 TF 测试

仅测试 PCT 全局规划时：

```bash
# 终端 1
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py \
  config_profile:=unitree_go2 \
  start_open3d_loc:=false \
  start_pct_planner:=true \
  start_go2_bridge:=false \
  navigation_mode:=2

# 终端 2
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map base_link
```

静态 TF 只能验证 PCT 和部分路径协调，不能验证定位、局部感知、闭环控制和真机安全。

#### 2.5.4 正式真机导航

机器狗先架空或放在空旷区域，现场人员站在实体急停旁。

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py \
  config_profile:=unitree_go2 \
  start_open3d_loc:=true \
  start_pct_planner:=true \
  start_go2_bridge:=true \
  bypass_safety:=false \
  navigation_mode:=2
```

另开终端检查：

```bash
ros2 topic echo /localization_status --once
ros2 run tf2_ros tf2_echo map base_link
ros2 node list | grep go2_cmd_vel_bridge
ros2 topic echo /go2_cmd_vel_bridge/armed --once
```

必须满足：

- 定位 `state: 3`；
- TF 正确；
- bridge 日志显示 WebRTC 连接成功；
- 当前 armed 为 `false`；
- 现场已经清场。

然后使能：

```bash
ros2 service call /go2_cmd_vel_bridge/enable \
  std_srvs/srv/SetBool "{data: true}"

ros2 topic echo /go2_cmd_vel_bridge/armed --once
```

armed 必须为 `true`。

#### 2.5.5 发布导航目标

可以在 RViz 使用 **2D Goal Pose**，也可以调用服务：

```bash
ros2 service call /open3d_loc/publish_goal open3d_loc/srv/PublishGoal \
  "{x: 3.0, y: 1.0, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

发布后检查：

```bash
ros2 topic echo /pct_path --once
ros2 topic echo /scan_planner/waypoints --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /go2_cmd_vel_bridge/safe_cmd_vel --once
```

服务返回成功只表示目标已经发布，不表示机器人已经到达。

#### 2.5.6 重定位

可以在 RViz 使用 **2D Pose Estimate**，也可以调用：

```bash
ros2 service call /open3d_loc/relocalize open3d_loc/srv/Relocalize \
  "{x: 1.0, y: 2.0, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

重定位后重新确认 `/localization_status` 和 `map -> base_link`。

#### 2.5.7 停止导航

正常停止顺序：

```bash
# 1. 禁止底盘 bridge
ros2 service call /go2_cmd_vel_bridge/enable \
  std_srvs/srv/SetBool "{data: false}"

# 2. 清空路径并软复位
ros2 service call /restart_navigation \
  pct_scan_navigation/srv/RestartNavigation "{mode: 0}"

# 3. 确认 disarm
ros2 topic echo /go2_cmd_vel_bridge/armed --once
```

然后回到 launch 终端按 `Ctrl+C`。

停止 ROS bridge：在 bridge 终端按 `Ctrl+C`。停止导航容器：

```bash
docker stop -t 15 kn_nav_container
```

紧急情况优先使用机器狗实体急停。如果运行的是 `bypass_safety:=true`，bridge 不创建 enable/disable 服务，应使用实体急停，并终止 launch 或停止容器。

#### 2.5.8 bypass 调试模式

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py \
  config_profile:=unitree_go2 \
  start_open3d_loc:=true \
  start_pct_planner:=true \
  start_go2_bridge:=true \
  bypass_safety:=true \
  navigation_mode:=2
```

该模式启动即 armed，`/cmd_vel` 可以直接驱动机器狗。只允许架空或受控调试，不允许正式运行。

### 2.6 建图

#### 2.6.1 建图前准备

1. 确认 ROS bridge、雷达和 IMU 数据正常；
2. 确认 FAST-LIO 外参、topic、雷达类型正确；
3. 新建独立地图文件名，不要覆盖当前正式 `clean_data.pcd`；
4. 建图时不要启动 Open3D 定位、PCT、SCAN 和 Go2 bridge；
5. 确认保存目录有足够磁盘空间。

当前 FAST-LIO 配置：

```text
/workspace/kn_nav_ws/src/pct_scan_navigation/config/unitree_go2/fast_lio.yaml
```

修改前先备份：

```bash
cp \
  /workspace/kn_nav_ws/src/pct_scan_navigation/config/unitree_go2/fast_lio.yaml \
  /workspace/kn_nav_ws/src/pct_scan_navigation/config/unitree_go2/fast_lio.mapping.backup.yaml
```

将配置中的以下参数修改为实际输出路径：

```yaml
map_file_path: "/workspace/kn_nav_ws/data/20260813_场景/map/场景_20260813_v01.pcd"

pcd_save:
  pcd_save_en: true
  interval: -1
```

先创建输出目录：

```bash
mkdir -p /workspace/kn_nav_ws/data/20260813_场景/map
```

#### 2.6.2 启动 FAST-LIO 建图

```bash
ros2 launch fast_lio mapping.launch.py \
  config_path:=/workspace/kn_nav_ws/src/pct_scan_navigation/config/unitree_go2 \
  config_file:=fast_lio.yaml \
  rviz:=false
```

另开终端检查：

```bash
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /livox/imu
ros2 topic hz /Odometry_loc
ros2 service list | grep map_save
```

缓慢移动机器人完成场地采集，避免剧烈转动、运动物体密集和长时间数据中断。

#### 2.6.3 保存 PCD

```bash
ros2 service call /map_save std_srvs/srv/Trigger "{}"
```

预期返回：

```text
success: true
message: Map saved.
```

检查文件：

```bash
ls -lh /workspace/kn_nav_ws/data/20260813_场景/map/*.pcd
sha256sum /workspace/kn_nav_ws/data/20260813_场景/map/*.pcd
```

保存完成后再在 launch 终端按 `Ctrl+C`。

#### 2.6.4 地图后处理

使用 CloudCompare 等工具：

1. 删除人员、移动物体和离群点；
2. 裁剪无关区域；
3. 检查地面朝向；
4. 进行体素降采样；
5. 另存为新版本，保留原始 PCD。

不要直接覆盖原始建图数据。

#### 2.6.5 生成 PCT tomogram

```bash
python3 /workspace/kn_nav_ws/src/PCT_planner/tomography/scripts/tomogram_cpu.py \
  --pcd /workspace/kn_nav_ws/data/20260813_场景/map/场景_20260813_v01.pcd \
  --out /workspace/kn_nav_ws/data/20260813_场景/map/场景_20260813_v01.pickle \
  --voxel 0.10 \
  --resolution 0.20
```

生成后记录校验：

```bash
sha256sum \
  /workspace/kn_nav_ws/data/20260813_场景/map/场景_20260813_v01.pcd \
  /workspace/kn_nav_ws/data/20260813_场景/map/场景_20260813_v01.pickle
```

#### 2.6.6 切换到新地图

以下三个配置必须同步：

1. `config/unitree_go2/map_profiles.yaml`：修改 `pcd_path` 和 `tomo_path`；
2. `config/unitree_go2/open3d_loc.yaml`：修改 `path_map`；
3. `config/unitree_go2/pct_global_planner.yaml`：修改 `tomo_path`。

修改后先用无底盘模式验证：

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py \
  config_profile:=unitree_go2 \
  start_open3d_loc:=true \
  start_pct_planner:=true \
  start_go2_bridge:=false \
  navigation_mode:=2
```

只有定位、TF、tomogram、起终点和路径全部正确后，才能进行真机验证。

## 3. 错误排查

### 3.1 容器启动失败

```bash
docker ps -a --filter name='^/kn_nav_container$'
docker logs --tail 100 kn_nav_container
docker image inspect kn_nav:v1
```

常见情况：

- `Conflict. The container name is already in use`：旧容器已存在，使用 `docker start`，不要重复 `docker run`；
- `No such image`：先 `docker pull` 或 `docker load`；
- 架构错误：检查镜像必须为 `linux/arm64`；
- 项目目录为空：检查 bind mount 的宿主机路径。

### 3.2 RViz 无法打开

宿主机执行：

```bash
echo "$DISPLAY"
cp "${XAUTHORITY:-$HOME/.Xauthority}" /home/jetson/kn_nav/xauth_host
docker exec -e DISPLAY=$DISPLAY -it kn_nav_container bash
```

确认当前 SSH 连接启用了 X11 转发。每次新 SSH 会话都要重新复制 Xauthority。

### 3.3 ROS bridge 没有数据

```bash
echo "$ROS_MASTER_URI"
docker ps --filter name=ros1-foxy-bridge
ros2 topic list | grep livox
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /livox/imu
```

检查：

1. `roscore` 是否运行；
2. 雷达 ROS 1 驱动是否发布数据；
3. `IP=$(hostname -I | awk '{print $1}')` 是否为空或选错网卡；
4. bridge 和导航端 `ROS_DOMAIN_ID` 是否都是 0；
5. topic 类型是否是当前使用的 PointCloud2。

### 3.4 FAST-LIO 报 `No point, skip this scan`

```bash
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /livox/imu
ros2 topic info /livox/lidar/pointcloud -v
```

持续报错通常表示桥接无数据、topic 名不对或消息类型不匹配。当前配置使用：

```yaml
lid_topic: "/livox/lidar/pointcloud"
lidar_type: 4
```

不要回退到旧的 CustomMsg 配置 `/livox/lidar`、`lidar_type: 1`。

### 3.5 定位失败或定位丢失

```bash
ros2 topic echo /localization_status --once
ros2 topic hz /Odometry_open3d
ros2 run tf2_ros tf2_echo map base_link

grep -RniE 'no_scan|lost|fitness|invalid|map_empty' \
  /workspace/kn_nav_ws/src/log/latest
```

常见原因：

- PCD 路径错误；
- 初始位姿偏差太大；
- PCD 与现场不一致；
- 雷达/IMU 外参错误；
- 点云数量不足；
- ICP fitness 阈值不适合当前场景。

定位不是 `state: 3` 时禁止 enable。

### 3.6 PCT 不出路径

```bash
ros2 node list | grep pct
ros2 topic echo /tomogram --once
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /goal_pose --once
ros2 topic echo /pct_path --once
```

检查：

- tomogram 文件是否存在；
- PCD 和 pickle 是否同一版坐标；
- 起点、终点是否位于地图范围；
- 目标 Z 是否选错楼层；
- `map -> base_link` 是否存在；
- 起终点 cost 是否小于可通行阈值 20。

### 3.7 SCAN 没有局部轨迹

```bash
ros2 topic hz /scan_map
ros2 topic hz /Odometry_open3d
ros2 topic echo /scan_planner/waypoints --once
ros2 topic info /scan_map -v
```

如果持续出现 `no sensor_pose received for lidar cloud update`，检查 `/Odometry_open3d` 是否持续发布，以及 launch 中的 remap 是否正常。

### 3.8 Go2 bridge 无法连接

```bash
grep -n 'robot_ip' \
  /workspace/kn_nav_ws/src/pct_scan_navigation/config/unitree_go2/go2_bridge.yaml
ping -c 3 192.168.123.161
ros2 node list | grep bridge
```

检查机器狗 IP、STA 网络、WebRTC 日志和防火墙。`network_interface` 参数不会覆盖当前 Python bridge 的 `robot_ip`。

### 3.9 机器人不动

```bash
ros2 topic echo /go2_cmd_vel_bridge/armed --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /go2_cmd_vel_bridge/safe_cmd_vel --once
ros2 topic echo /localization_status --once
```

正式模式下尝试使能：

```bash
ros2 service call /go2_cmd_vel_bridge/enable \
  std_srvs/srv/SetBool "{data: true}"
```

如果 enable 后很快 disarm，重点检查：

- `/Odometry_open3d` 心跳；
- WebRTC sport state 心跳；
- `/cmd_vel` 是否超过 0.3 秒未更新；
- 4G 网络是否使 `sport_state_timeout=0.5` 秒过紧；
- Move/StopMove 是否超时或返回失败。

优先解决网络稳定性。临时调大超时必须记录修改值和安全评估。

### 3.10 地图保存失败

```bash
ros2 service list | grep map_save
ros2 service type /map_save
grep -nE 'map_file_path|pcd_save_en' \
  /workspace/kn_nav_ws/src/pct_scan_navigation/config/unitree_go2/fast_lio.yaml
```

`/map_save` 类型应为：

```text
std_srvs/srv/Trigger
```

如果返回 `Map save disabled`，说明 `pcd_save.pcd_save_en` 没有设为 `true`。如果文件没有生成，检查 `map_file_path` 的父目录是否存在且可写。

### 3.11 查看最新日志

```bash
ls -l /workspace/kn_nav_ws/src/log/latest

grep -RniE 'error|fatal|failed|warn|lost' \
  /workspace/kn_nav_ws/src/log/latest
```

统一 launch 只保留最近约 20 次运行日志，重要日志需要及时复制归档。

## 4. 可能会用到的操作

### 4.1 查看当前版本

```bash
cd /home/jetson/kn_nav
git status
git branch --show-current
git remote -v
git log -1 --oneline
```

本文档基线：

```text
分支：master
提交：16588164d81c0a574c65215fa843100971c87ae0
```

### 4.2 查看容器和镜像

```bash
docker ps -a
docker image ls

docker inspect kn_nav_container \
  --format 'image={{.Config.Image}} network={{.HostConfig.NetworkMode}} status={{.State.Status}}'

docker image inspect kn_nav:v1 \
  --format '{{.Id}} {{.Os}}/{{.Architecture}}'
```

### 4.3 测试外网

```bash
curl -I --connect-timeout 5 https://www.github.com
```

### 4.4 查看 ROS 节点、topic、服务和 TF

```bash
ros2 node list | sort
ros2 topic list | sort
ros2 service list | sort
ros2 run tf2_ros tf2_echo map base_link
```

### 4.5 查看当前定位位姿

```bash
ros2 service call /open3d_loc/get_pose \
  open3d_loc/srv/GetPose "{}"
```

### 4.6 软复位导航

```bash
ros2 service call /restart_navigation \
  pct_scan_navigation/srv/RestartNavigation "{mode: 0}"
```

完整重启模式当前没有配置 `full_restart_command`，默认不可用。

### 4.7 PCD 转 tomogram

```bash
python3 /workspace/kn_nav_ws/src/PCT_planner/tomography/scripts/tomogram_cpu.py \
  --pcd /workspace/kn_nav_ws/src/PCT_planner/rsc/pcd/clean_data.pcd \
  --out /workspace/kn_nav_ws/src/PCT_planner/rsc/tomogram/clean_data.pickle \
  --voxel 0.10 \
  --resolution 0.20
```

不要直接覆盖正式 pickle。先输出为新版本，验证通过后再修改配置。

### 4.8 地图文件校验和备份

```bash
cd /home/jetson/kn_nav

sha256sum \
  src/PCT_planner/rsc/pcd/clean_data.pcd \
  src/PCT_planner/rsc/tomogram/clean_data.pickle \
  > clean_data.sha256

tar -czf clean_data_backup.tar.gz \
  src/PCT_planner/rsc/pcd/clean_data.pcd \
  src/PCT_planner/rsc/tomogram/clean_data.pickle \
  clean_data.sha256
```

PCD 和 pickle 被 Git 忽略，必须单独放入交接云盘或制品库。

### 4.9 查看磁盘占用

```bash
df -h /home/jetson/kn_nav
du -sh /home/jetson/kn_nav/src/PCT_planner/rsc/pcd
du -sh /home/jetson/kn_nav/src/PCT_planner/rsc/tomogram
du -sh /home/jetson/kn_nav/src/log
```

### 4.10 快速状态检查

```bash
ros2 topic echo /localization_status --once
ros2 topic echo /go2_cmd_vel_bridge/armed --once
ros2 topic echo /go2_cmd_vel_bridge/safe_cmd_vel --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /pct_path --once
ros2 run tf2_ros tf2_echo map base_link
```

### 4.11 操作红线

1. 第一次启动一律使用 `start_go2_bridge:=false`；
2. 正式运行一律使用 `bypass_safety:=false`；
3. 定位不是 TRACKING、TF 错误或路径穿障碍时禁止 enable；
4. `bypass_safety:=true` 启动即 armed，只允许架空或受控调试；
5. 正常停止顺序：disable bridge → 软复位 → `Ctrl+C` → 停容器；
6. 紧急情况优先使用实体急停；
7. 新地图必须保留原始 PCD、处理后 PCD、pickle、参数和 SHA-256；
8. PCD、pickle、Bag 和镜像 tar 不进 Git，必须另行备份。
