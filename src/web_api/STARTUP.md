# KN 导航与 Web API 启动命令速查

```bash
python3 src/web_api/ros2_service_api.py \
  --host 0.0.0.0 \
  --port 8000
```

本文档用于个人记录常用启动、检查和停止命令。

接口字段和响应格式见同目录的 `API.md`。

## 1. 实机常用启动顺序

建议依次启动：

1. Livox 雷达驱动；
2. KN 导航系统；
3. Web API；
4. 检查定位和 ROS Service；
5. 确认安全后开启底盘速度桥；
6. 发布导航目标。

## 2. 进入机器人容器

在宿主机查看容器：

```bash
docker ps
```

进入容器：

```bash
docker exec -it <容器ID> /bin/bash
```

当前 tmux 工具脚本中记录的容器 ID 是：

```text
f3b82610c6d7
```

如果容器重新创建，需要使用 `docker ps` 查询新的 ID。

## 3. 每个容器终端的环境准备

```bash
cd /home/code/work_space/kn_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果需要 Unitree SDK 动态库：

```bash
export CMAKE_PREFIX_PATH=/opt/unitree_robotics:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH
```

## 4. 启动 Livox 雷达

终端 1：

```bash
cd /home/code/work_space/kn_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

检查雷达话题：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## 5. 启动导航系统

终端 2：

```bash
cd /home/code/work_space/kn_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Go2-W

```bash
ros2 launch pct_scan_navigation unitree_go2w_pct_scan_navigation.launch.py \
  network_interface:=eth0
```

### Go2

```bash
ros2 launch pct_scan_navigation unitree_go2_pct_scan_navigation.launch.py \
  network_interface:=eth0
```

### A2

```bash
ros2 launch pct_scan_navigation unitree_A2_pct_scan_navigation.launch.py \
  network_interface:=eth0
```

### 本机配置

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py
```

如果实际网卡不是 `eth0`，先检查：

```bash
ip link
```

然后替换 `network_interface`。

## 6. 启动 Web API

终端 3：

```bash
cd /home/code/work_space/kn_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/web_api/ros2_service_api.py \
  --host 0.0.0.0 \
  --port 8000
```

首次运行前如果缺少依赖：

```bash
python3 -m pip install fastapi uvicorn
```

API 启动时读取 `/global_localization_node` 的 `path_map` 参数，并以当前离线地图文件名创建或复用导航点文件。例如地图为 `global_map_downsize.pcd` 时：

```text
/home/code/work_space/kn_nav/src/web_api/global_map_downsize.json
```

服务端日志出现以下内容表示启动成功：

```text
Application startup complete.
Uvicorn running on http://0.0.0.0:8000
```

同机访问地址：

```text
http://127.0.0.1:8000
```

其他机器访问时使用机器人 IP：

```text
http://<机器人IP>:8000
```

查看机器人 IP：

```bash
hostname -I
```

接口文档：

```text
http://<机器人IP>:8000/docs
```

## 7. 带 API Key 启动

需要限制接口访问时：

```bash
cd /home/code/work_space/kn_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

export KN_NAV_API_KEY='替换为实际密钥'

python3 src/web_api/ros2_service_api.py \
  --host 0.0.0.0 \
  --port 8000
```

调用 `/api/*` 时需要增加：

```text
X-API-Key: 替换为实际密钥
```

## 8. 自定义导航点文件

默认根据当前离线地图自动选择 JSON 文件，一般不需要指定 `--points-file`。

指定其他文件：

```bash
python3 src/web_api/ros2_service_api.py \
  --host 0.0.0.0 \
  --port 8000 \
  --points-file /home/code/work_space/kn_nav/src/web_api/debug_points.json
```

也可以使用环境变量：

```bash
export KN_NAV_POINTS_FILE=/home/code/work_space/kn_nav/src/web_api/debug_points.json
```

## 9. 启动后快速检查

### 检查 API

```bash
curl http://127.0.0.1:8000/
```

### 检查 API 和 ROS 网关

```bash
curl http://127.0.0.1:8000/api/health
```

### 检查 ROS Service 是否就绪

```bash
curl http://127.0.0.1:8000/api/services
```

也可以直接检查 ROS：

```bash
ros2 service list -t
```

需要确认以下 Service：

```text
/open3d_loc/relocalize
/open3d_loc/get_pose
/open3d_loc/publish_goal
/go2_cmd_vel_bridge/enable
```

### 检查当前定位

```bash
ros2 topic echo /Odometry_open3d --once
```

### 检查全局路径

```bash
ros2 topic echo /pct_path --once
```

### 获取全部导航点

```bash
curl http://127.0.0.1:8000/api/navigation/points
```

## 10. 常用 API 命令

### 重定位

```bash
curl -X POST http://127.0.0.1:8000/api/open3d_loc/relocalize \
  -H 'Content-Type: application/json' \
  -d '{
    "x": 1.2327408046653152,
    "y": -0.09142521170438922,
    "z": -0.5001805743966447,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.741289779172096,
    "qw": 0.671185118499349
  }'
```

### 保存当前点

```bash
curl -X POST http://127.0.0.1:8000/api/navigation/points \
  -H 'Content-Type: application/json' \
  -d '{
    "name": "office",
    "overwrite": false
  }'
```

覆盖同名点：

```bash
curl -X POST http://127.0.0.1:8000/api/navigation/points \
  -H 'Content-Type: application/json' \
  -d '{
    "name": "office",
    "overwrite": true
  }'
```

### 发布保存的导航点

```bash
curl -X POST http://127.0.0.1:8000/api/navigation/goal \
  -H 'Content-Type: application/json' \
  -d '{"name":"office"}'
```

### 发布直接坐标

```bash
curl -X POST http://127.0.0.1:8000/api/navigation/goal \
  -H 'Content-Type: application/json' \
  -d '{
    "x": 3.0,
    "y": 1.0,
    "z": 0.4,
    "yaw": 1.57
  }'
```

### 开启底盘速度桥

执行前确认定位正确、机器人状态正常、场地安全：

```bash
curl -X POST http://127.0.0.1:8000/api/go2_cmd_vel_bridge/enable \
  -H 'Content-Type: application/json' \
  --data-raw '{"data":true}'
```

### 关闭底盘速度桥

```bash
curl -X POST http://127.0.0.1:8000/api/go2_cmd_vel_bridge/enable \
  -H 'Content-Type: application/json' \
  --data-raw '{"data":false}'
```

也可以直接调用 ROS：

```bash
ros2 service call /go2_cmd_vel_bridge/enable \
  std_srvs/srv/SetBool '{data: false}'
```

## 11. tmux 快速启动

宿主机已有工具脚本：

```bash
cd /home/code/work_space/kn_nav
bash src/tools/open_go2_nav_tmux.sh
```

重新创建 tmux 布局：

```bash
bash src/tools/open_go2_nav_tmux.sh --recreate
```

tmux 脚本只会预填命令，不会自动按回车执行。进入后检查每个窗格中的命令，再手动执行。

## 12. 停止顺序

### 先关闭底盘速度桥

```bash
curl -X POST http://127.0.0.1:8000/api/go2_cmd_vel_bridge/enable \
  -H 'Content-Type: application/json' \
  --data-raw '{"data":false}'
```

### 停止 API

在 API 终端按：

```text
Ctrl+C
```

### 停止导航和雷达

分别在对应终端按：

```text
Ctrl+C
```

不要在机器人仍运动时直接关闭 API 或导航进程，应先禁用底盘速度桥。

## 13. 常见问题命令

### 8000 端口被占用

```bash
ss -lntp | grep ':8000'
```

换端口启动：

```bash
python3 src/web_api/ros2_service_api.py \
  --host 0.0.0.0 \
  --port 8001
```

### 导航点 JSON 检查

```bash
cd /home/code/work_space/kn_nav/src/web_api
ls -l *.json
python3 -m json.tool global_map_downsize.json
```

### ROS Service 不可用

```bash
ros2 service list -t
ros2 node list
```

### API 日志显示 422

确认请求包含：

```text
Content-Type: application/json
```

确认 Body 是 JSON 对象，不是字符串。

### 查看帮助

```bash
python3 src/web_api/ros2_service_api.py --help
```
