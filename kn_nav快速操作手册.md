# kn_nav 快速操作手册

> 适用环境：Jetson/ARM64 宿主机 + Docker + `kn_nav:v1` + ROS 2 Humble  
> 宿主机项目：`/home/jetson/kn_nav`  
> 容器内项目：`/workspace/kn_nav_ws`  
> 导航容器：`kn_nav_container`  
> 详细配置和排障见：[kn_nav交接文档（详细版）.md](./kn_nav交接文档（详细版）.md)

## 1. 最短启动流程

### 1.1 宿主机：启动 ROS 1/ROS 2 bridge

先确认 ROS 1 master 和雷达驱动已经运行，再执行：

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

该终端保持运行。

### 1.2 宿主机：启动并进入导航容器

```bash
docker start kn_nav_container
docker exec -it kn_nav_container bash
```

### 1.3 容器内：加载环境

每个新终端都要执行：

```bash
cd /workspace/kn_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export Open3D_DIR=/opt/open3d141/lib/cmake/Open3D
export THIRDPARTY_ROOT=/opt/pct-install
export ROS_DOMAIN_ID=0
```

### 1.4 容器内：先启动无底盘模式

首次启动或更换地图后，先关闭 Go2 bridge：

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py \
  config_profile:=unitree_go2 \
  start_open3d_loc:=true \
  start_pct_planner:=true \
  start_go2_bridge:=false \
  navigation_mode:=2
```

检查定位、TF、地图和路径都正常后，再进行真机启动。

### 1.5 容器内：正式真机启动

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py \
  config_profile:=unitree_go2 \
  start_open3d_loc:=true \
  start_pct_planner:=true \
  start_go2_bridge:=true \
  bypass_safety:=false \
  navigation_mode:=2
```

另开容器终端，重新加载 1.3 节环境，然后检查定位：

```bash
ros2 topic echo /localization_status --once
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /go2_cmd_vel_bridge/armed --once
```

只有满足以下条件才能 enable：

- `/localization_status` 为 `state: 3`（TRACKING）；
- `map -> base_link` 连续且位置、朝向正确；
- bridge 日志显示 WebRTC 已成功连接；
- 机器人周围清场，现场人员可随时操作实体急停。

使能：

```bash
ros2 service call /go2_cmd_vel_bridge/enable \
  std_srvs/srv/SetBool "{data: true}"

ros2 topic echo /go2_cmd_vel_bridge/armed --once
```

返回成功且 armed 为 `true` 后，才可在 RViz 发布目标。

## 2. 启动前检查

### 2.1 宿主机检查

```bash
# Docker 是否可用
docker --version
docker info >/dev/null && echo "Docker OK"

# 镜像、容器是否存在
docker image inspect kn_nav:v1 >/dev/null && echo "Image OK"
docker ps -a --filter name='^/kn_nav_container$'

# 项目和地图是否存在
test -d /home/jetson/kn_nav && echo "Workspace OK"
test -s /home/jetson/kn_nav/src/PCT_planner/rsc/pcd/clean_data.pcd \
  && echo "PCD OK"
test -s /home/jetson/kn_nav/src/PCT_planner/rsc/tomogram/clean_data.pickle \
  && echo "Tomogram OK"

# 磁盘空间
df -h /home/jetson/kn_nav
```

检查容器实际挂载：

```bash
docker inspect kn_nav_container \
  --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}'
```

必须包含：

```text
/home/jetson/kn_nav -> /workspace/kn_nav_ws
/dev/shm -> /dev/shm
```

### 2.2 网络和配置检查

当前 Go2 IP 写在：

```text
/home/jetson/kn_nav/src/pct_scan_navigation/config/unitree_go2/go2_bridge.yaml
```

当前值应为：

```yaml
robot_ip: "192.168.123.161"
```

检查：

```bash
grep -n 'robot_ip' \
  /home/jetson/kn_nav/src/pct_scan_navigation/config/unitree_go2/go2_bridge.yaml
ping -c 3 192.168.123.161
```

如果机器人 IP 变化，只改 launch 参数 `network_interface` 没有效果；当前 Python WebRTC bridge 实际读取 `go2_bridge.yaml` 中的 `robot_ip`。

### 2.3 容器内依赖检查

```bash
cd /workspace/kn_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

test -f /opt/open3d141/lib/cmake/Open3D/Open3DConfig.cmake \
  && echo "Open3D C++ OK"
test -d /opt/pct-install && echo "PCT third-party OK"

ros2 pkg prefix fast_lio
ros2 pkg prefix open3d_loc
ros2 pkg prefix pct_planner
ros2 pkg prefix scan_planner
ros2 pkg prefix pct_scan_navigation
```

### 2.4 ROS 数据检查

bridge 启动后，在导航容器中执行：

```bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /livox/imu
```

雷达和 IMU 必须持续有数据。持续无数据时不要启动真机 bridge。

### 2.5 RViz 检查

每次新 SSH X11 会话，在宿主机执行：

```bash
cp "${XAUTHORITY:-$HOME/.Xauthority}" /home/jetson/kn_nav/xauth_host
docker exec -e DISPLAY=$DISPLAY -it kn_nav_container bash
```

容器内：

```bash
cd /workspace/kn_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d \
  /workspace/kn_nav_ws/src/pct_scan_navigation/config/unitree_go2/kn_nav.rviz
```

成功标准：Fixed Frame 为 `map`，地图、机器人位姿、PCT 路径和局部规划显示正常。

## 3. 怎么停止

### 3.1 正常停止真机导航

在另一个已加载环境的容器终端执行：

```bash
# 先禁止底盘 bridge
ros2 service call /go2_cmd_vel_bridge/enable \
  std_srvs/srv/SetBool "{data: false}"

# 再清空任务、发布零速并软复位规划器
ros2 service call /restart_navigation \
  pct_scan_navigation/srv/RestartNavigation "{mode: 0}"

# 确认已经 disarm
ros2 topic echo /go2_cmd_vel_bridge/armed --once
```

然后回到运行 launch 的终端按 `Ctrl+C`，等待所有节点退出。

### 3.2 停止 bridge

在 `ros1-foxy-bridge` 前台终端按 `Ctrl+C`。因为启动时使用了 `--rm`，退出后容器会自动删除。

如果终端丢失：

```bash
docker stop -t 10 ros1-foxy-bridge
```

### 3.3 停止导航容器

确认 launch 和 RViz 已退出后，在宿主机执行：

```bash
docker stop -t 15 kn_nav_container
docker ps -a --filter name='^/kn_nav_container$'
```

`docker stop` 不会删除容器，下次可继续 `docker start`。

### 3.4 紧急停止

优先使用机器狗实体急停。若终端仍可操作，同时执行：

```bash
ros2 service call /go2_cmd_vel_bridge/enable \
  std_srvs/srv/SetBool "{data: false}"
```

必要时在宿主机强制停止导航容器：

```bash
docker stop -t 2 kn_nav_container
```

如果当前是 `bypass_safety:=true`，bridge 不创建 enable/disable 服务，应直接使用实体急停，并终止 launch 或停止容器。

容器停止不能代替实体急停；网络阻塞时 ROS/Docker 命令可能无法及时到达机器狗。

## 4. 怎么建容器

### 4.1 创建前检查

```bash
docker image inspect kn_nav:v1
docker ps -a --filter name='^/kn_nav_container$'
test -d /home/jetson/kn_nav
```

如果已经存在 `kn_nav_container`，直接使用：

```bash
docker start kn_nav_container
docker exec -it kn_nav_container bash
```

不要重复创建同名容器，也不要在未确认数据是否需要保留前删除旧容器。

### 4.2 首次创建

先准备 Xauthority 文件：

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

退出首次创建的交互 shell 后，如果容器停止，执行：

```bash
docker start kn_nav_container
docker exec -it kn_nav_container bash
```

### 4.3 创建后检查

```bash
docker inspect kn_nav_container \
  --format 'image={{.Config.Image}} network={{.HostConfig.NetworkMode}} status={{.State.Status}}'

docker exec kn_nav_container bash -lc \
  'test -f /workspace/kn_nav_ws/install/setup.bash && echo "Workspace OK"'
```

预期：镜像为 `kn_nav:v1`、网络为 `host`、workspace 检查通过。

## 5. 怎么建镜像

### 5.1 当前生产 ARM64 镜像的实际情况

当前正在使用的 `kn_nav:v1` 是 `linux/arm64` 镜像，但仓库中没有与它对应的 ARM64 Dockerfile。现有 `docker/amd/Dockerfile` 是 amd64 参考文件，不能直接当作 Jetson 生产镜像构建文件。

因此当前推荐顺序是：

1. 优先从项目制品仓库拉取已验证的 ARM64 镜像；
2. 没有仓库时，从交接 tar 包 `docker load`；
3. 在 ARM64 Dockerfile 补齐并通过干净机器验证前，不要声称可以从源码完全复现 `kn_nav:v1`。

### 5.2 从当前容器制作应急快照

仅用于保存容器内安装的依赖和修改：

```bash
docker commit \
  kn_nav_container \
  kn_nav:v1-arm64-backup-20260813

docker image inspect kn_nav:v1-arm64-backup-20260813 \
  --format '{{.Id}} {{.Os}}/{{.Architecture}}'
```

重要限制：`/workspace/kn_nav_ws` 是 bind mount，`docker commit` **不会把宿主机代码、PCD、pickle、build/install 目录打进镜像**。项目目录和地图必须单独备份。

建议立即导出并校验：

```bash
docker save -o /home/jetson/kn_nav_v1_arm64_backup_20260813.tar \
  kn_nav:v1-arm64-backup-20260813

sha256sum /home/jetson/kn_nav_v1_arm64_backup_20260813.tar \
  > /home/jetson/kn_nav_v1_arm64_backup_20260813.tar.sha256
```

### 5.3 使用 Dockerfile 正式构建

正式方案应先补充 ARM64 Dockerfile，例如：

```text
/home/jetson/kn_nav/docker/arm64/Dockerfile
```

然后在项目根目录执行：

```bash
cd /home/jetson/kn_nav

docker build \
  --network host \
  --platform linux/arm64 \
  -f docker/arm64/Dockerfile \
  -t kn_nav:v1-arm64 .
```

构建完成后必须检查架构并在干净容器中完成 ROS 包和 launch 自检：

```bash
docker image inspect kn_nav:v1-arm64 \
  --format '{{.Id}} {{.Os}}/{{.Architecture}}'

docker run --rm kn_nav:v1-arm64 bash -lc \
  'source /opt/ros/humble/setup.bash && python3 --version'
```

当前 `docker/arm64/Dockerfile` 尚不存在；上面的命令是补齐文件后的标准入口，不是当前立即可执行的构建命令。

### 5.4 仓库中的 amd64 参考构建

仅在 x86_64/amd64 机器使用。该 Dockerfile 还依赖构建上下文中的：

- `thirdparty/open3d141.zip`；
- `work_space/ws_livox/src/`；
- `work_space/kn_nav_ws/` 目录布局。

仓库原参考命令：

```bash
cd <包含 thirdparty 和 work_space 的构建上下文根目录>

docker buildx build \
  --network=host \
  --platform=linux/amd64 \
  --load \
  --build-arg GIT_REFRESH="$(date -u +%Y%m%d%H%M%S)" \
  -f work_space/kn_nav_ws/docker/amd/Dockerfile \
  -t cross-floor-nav:amd-v1 .
```

不要在当前 Jetson 上把这个 amd64 镜像标记成 `kn_nav:v1` 用于生产。

## 6. 怎么拉镜像

### 6.1 从镜像仓库拉取

当前交接资料尚未提供实际 registry 地址。补齐后使用：

```bash
docker login <镜像仓库域名>

docker pull <镜像仓库域名>/<项目>/kn_nav:v1-arm64

docker image inspect <镜像仓库域名>/<项目>/kn_nav:v1-arm64 \
  --format '{{.Id}} {{.Os}}/{{.Architecture}}'
```

确认输出为 `linux/arm64` 后，建立本地标准标签：

```bash
docker tag \
  <镜像仓库域名>/<项目>/kn_nav:v1-arm64 \
  kn_nav:v1
```

然后按第 4 节创建容器。

不要直接复制尖括号占位符执行。交接人必须补充：

- registry 域名和项目路径；
- 登录方式/权限申请人；
- 固定版本 tag；
- 镜像 digest；
- ARM64 构建日期和验收记录。

生产部署最好按 digest 拉取或至少记录 digest，避免同名 tag 被覆盖。

### 6.2 从离线 tar 导入

收到镜像 tar 和 SHA-256 文件后：

```bash
cd <镜像文件所在目录>
sha256sum -c kn_nav_v1_arm64_20260813.tar.sha256
docker load -i kn_nav_v1_arm64_20260813.tar
docker image ls | grep kn_nav
```

如果导入后的名字不是 `kn_nav:v1`，重新打标签：

```bash
docker tag <导入后的镜像名:标签> kn_nav:v1
```

验证：

```bash
docker image inspect kn_nav:v1 \
  --format '{{.Id}} {{.Os}}/{{.Architecture}}'
```

预期输出为 `linux/arm64`。

### 6.3 拉取 ROS bridge 镜像

```bash
docker pull ros:foxy-ros1-bridge
docker image inspect ros:foxy-ros1-bridge \
  --format '{{.Id}} {{.Os}}/{{.Architecture}}'
```

如果官方 tag 不提供当前 ARM64 架构，需要由项目方提供对应的多架构镜像或离线 tar，不能用错误架构镜像替代。

## 7. 常用状态检查

```bash
# 宿主机：容器状态
docker ps -a --filter name=kn_nav_container
docker ps -a --filter name=ros1-foxy-bridge

# 容器内：节点
ros2 node list | sort

# 定位
ros2 topic echo /localization_status --once
ros2 run tf2_ros tf2_echo map base_link

# 规划
ros2 topic echo /pct_path --once
ros2 topic echo /scan_planner/waypoints --once

# 底盘
ros2 topic echo /go2_cmd_vel_bridge/armed --once
ros2 topic echo /go2_cmd_vel_bridge/safe_cmd_vel --once
ros2 topic echo /cmd_vel --once

# 最近日志
ls -l /workspace/kn_nav_ws/src/log/latest
grep -RniE 'error|fatal|failed|warn|lost' \
  /workspace/kn_nav_ws/src/log/latest
```

## 8. 现场操作红线

1. 首次启动一律 `start_go2_bridge:=false`。
2. 正式运行一律 `bypass_safety:=false`。
3. 定位不是 TRACKING、TF 方向不正确、路径穿障碍时禁止 enable。
4. `bypass_safety:=true` 启动即 armed，只允许架空或受控调试。
5. 停止顺序是：disable bridge → 软复位 → `Ctrl+C` launch → 停容器。
6. 实体急停优先级高于 ROS 和 Docker 命令。
7. PCD、pickle、镜像 tar 不在 Git 中，迁移机器前必须单独备份和校验。
