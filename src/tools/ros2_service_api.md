# KN 导航 ROS 2 Service API 使用指南

本文档说明如何运行 `ros2_service_api.py`，以及如何通过 HTTP 接口调用 KN 导航系统的 ROS 2 Service。

脚本路径：

```text
src/tools/ros2_service_api.py
```

## 1. 功能说明

该脚本在同一个进程中运行：

- 一个 FastAPI HTTP 服务；
- 一个后台 ROS 2 节点；
- 6 个 ROS 2 Service Client。

HTTP API 与 ROS 2 Service 的对应关系如下：

| HTTP API | ROS 2 Service | 用途 |
|---|---|---|
| `POST /api/navigation/goal` | `/open3d_loc/publish_goal` | 使用 XYZ 和 yaw 发布导航目标 |
| `POST /api/open3d_loc/publish_goal` | `/open3d_loc/publish_goal` | 使用完整四元数发布导航目标 |
| `POST /api/open3d_loc/relocalize` | `/open3d_loc/relocalize` | 触发 Open3D 重定位 |
| `GET /api/open3d_loc/get_pose` | `/open3d_loc/get_pose` | 获取当前定位位姿 |
| `POST /api/open3d_loc/pose_deviation` | `/open3d_loc/pose_deviation` | 计算当前位姿与参考位姿的偏差 |
| `POST /api/map/save` | `/map_save` | 请求 FAST-LIO 保存地图 |
| `POST /api/go2_cmd_vel_bridge/enable` | `/go2_cmd_vel_bridge/enable` | 启用或禁用底盘速度桥 |

所有位姿均使用 `map` 坐标系。距离单位为米，角度除特别说明外均为弧度。

## 2. 运行要求

运行机器需要具备：

- ROS 2 Humble；
- 已编译的 KN 导航工作区；
- Python ROS 2 包 `rclpy`；
- 工作区生成的 `open3d_loc.srv` Python 接口；
- Python 包 `fastapi` 和 `uvicorn`。

安装 HTTP 服务依赖：

```bash
python3 -m pip install fastapi uvicorn
```

如果使用 Python 虚拟环境，需要让虚拟环境能够访问 ROS 2 的系统 Python 包，例如使用 `--system-site-packages` 创建虚拟环境。

## 3. 启动方法

### 3.1 启动导航系统

先根据实际机器人启动导航节点。例如：

```bash
cd /home/ehuy/KN/kn_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py
```

也可以使用对应的 Go2、Go2-W 或 A2 launch。

### 3.2 启动 API 服务

打开另一个终端：

```bash
cd /home/ehuy/KN/kn_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/tools/ros2_service_api.py
```

默认监听地址：

```text
http://127.0.0.1:8000
```

交互式接口文档：

```text
http://127.0.0.1:8000/docs
```

需要允许其他机器访问时，可以监听所有网卡：

```bash
python3 src/tools/ros2_service_api.py --host 0.0.0.0 --port 8000
```

### 3.3 停止服务

在 API 进程所在终端按 `Ctrl+C`。脚本会停止后台 ROS 2 executor 并销毁自身创建的 ROS 2 节点。

## 4. 没有运行导航节点时的行为

只要 ROS 2、`rclpy` 和工作区 Service 接口已经正确安装并完成 `source`，即使导航节点尚未启动，API 进程也可以启动。

此时：

- `GET /` 可以正常访问；
- `GET /api/health` 和 `GET /api/services` 可以正常访问；
- 未启动的 ROS 2 Service 会显示为 `false`；
- 调用不可用的业务 Service 会返回 HTTP `503`。

如果没有安装 ROS 2，或者没有 `source install/setup.bash`，API 会在启动阶段失败，因为无法导入 `rclpy` 或 `open3d_loc.srv`。

## 5. 启动参数

查看参数：

```bash
python3 src/tools/ros2_service_api.py --help
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--host` | `127.0.0.1` | HTTP 监听地址 |
| `--port` | `8000` | HTTP 监听端口 |
| `--service-timeout` | `30.0` | 等待 ROS Service 响应的最长秒数 |
| `--service-wait-timeout` | `1.0` | 等待 ROS Service 出现在 ROS graph 中的最长秒数 |
| `--log-level` | `info` | Uvicorn 日志等级 |

示例：

```bash
python3 src/tools/ros2_service_api.py \
  --host 0.0.0.0 \
  --port 8080 \
  --service-timeout 35 \
  --service-wait-timeout 2
```

## 6. 环境变量

| 环境变量 | 默认值 | 说明 |
|---|---:|---|
| `KN_NAV_API_HOST` | `127.0.0.1` | HTTP 监听地址 |
| `KN_NAV_API_PORT` | `8000` | HTTP 监听端口 |
| `KN_NAV_SERVICE_TIMEOUT` | `30.0` | ROS Service 响应超时 |
| `KN_NAV_SERVICE_WAIT_TIMEOUT` | `1.0` | ROS Service 可用性等待超时 |
| `KN_NAV_API_KEY` | 空 | 可选的 API 访问密钥 |

命令行参数会覆盖对应的地址、端口和超时环境变量。API Key 仅通过环境变量设置。

当 `KN_NAV_API_KEY` 非空时，所有 `/api/*` 请求必须携带 HTTP Header：

```text
X-API-Key: 设置的密钥
```

示例：

```bash
export KN_NAV_API_KEY='replace-with-a-secret'
python3 src/tools/ros2_service_api.py --host 0.0.0.0
```

后续请求需要增加：

```bash
-H 'X-API-Key: replace-with-a-secret'
```

`GET /`、`/docs` 和 OpenAPI 文档本身不要求 API Key，但实际 `/api/*` 接口会进行校验。

## 7. 通用数据格式

### 7.1 XYZ + yaw 位姿

用于 `POST /api/navigation/goal`：

```json
{
  "x": 3.0,
  "y": 1.0,
  "z": 0.4,
  "yaw": 1.57
}
```

字段说明：

| 字段 | 必填 | 默认值 | 说明 |
|---|---|---:|---|
| `x` | 是 | 无 | `map` 坐标系 X，单位 m |
| `y` | 是 | 无 | `map` 坐标系 Y，单位 m |
| `z` | 否 | `0.0` | `map` 坐标系 Z，单位 m |
| `yaw` | 否 | `0.0` | 目标航向角，单位 rad |

脚本会将 yaw 转换为四元数后调用 `/open3d_loc/publish_goal`。

### 7.2 四元数位姿

用于重定位、位姿偏差和原始目标发布接口：

```json
{
  "x": 3.0,
  "y": 1.0,
  "z": 0.4,
  "qx": 0.0,
  "qy": 0.0,
  "qz": 0.0,
  "qw": 1.0
}
```

字段说明：

| 字段 | 必填 | 默认值 | 说明 |
|---|---|---:|---|
| `x` | 是 | 无 | `map` 坐标系 X，单位 m |
| `y` | 是 | 无 | `map` 坐标系 Y，单位 m |
| `z` | 否 | `0.0` | `map` 坐标系 Z，单位 m |
| `qx` | 否 | `0.0` | 四元数 X |
| `qy` | 否 | `0.0` | 四元数 Y |
| `qz` | 否 | `0.0` | 四元数 Z |
| `qw` | 否 | `1.0` | 四元数 W |

输入不能包含 `NaN` 或无穷值。四元数会由 ROS Service 进行合法性检查和归一化；模长过小的四元数会被拒绝。

## 8. 系统接口

### 8.1 根接口

```http
GET /
```

调用：

```bash
curl http://127.0.0.1:8000/
```

响应示例：

```json
{
  "name": "KN Navigation ROS 2 Service API",
  "docs": "/docs",
  "health": "/api/health"
}
```

### 8.2 健康检查

```http
GET /api/health
```

调用：

```bash
curl http://127.0.0.1:8000/api/health
```

响应示例：

```json
{
  "success": true,
  "node": "kn_nav_fastapi_gateway_12345",
  "services": {
    "/open3d_loc/relocalize": true,
    "/open3d_loc/get_pose": true,
    "/open3d_loc/publish_goal": true,
    "/open3d_loc/pose_deviation": true,
    "/map_save": true,
    "/go2_cmd_vel_bridge/enable": false
  }
}
```

`success=true` 表示 API 内部的 ROS 2 节点和 executor 正常运行，不表示所有业务 Service 均已就绪。每个 Service 的状态以 `services` 中对应的布尔值为准。

### 8.3 Service 状态

```http
GET /api/services
```

调用：

```bash
curl http://127.0.0.1:8000/api/services
```

该接口返回 ROS 2 Service 与就绪状态，不返回 API 节点名称。

## 9. 导航目标接口

### 9.1 使用 XYZ 和 yaw 发布目标

```http
POST /api/navigation/goal
Content-Type: application/json
```

调用：

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

成功响应：

```json
{
  "success": true,
  "message": "goal published on /goal_pose"
}
```

该接口内部调用 `/open3d_loc/publish_goal`。`success=true` 仅表示目标已经发布到 `/goal_pose`，不表示规划成功或机器人已经到达。

### 9.2 使用完整四元数发布目标

```http
POST /api/open3d_loc/publish_goal
Content-Type: application/json
```

调用：

```bash
curl -X POST http://127.0.0.1:8000/api/open3d_loc/publish_goal \
  -H 'Content-Type: application/json' \
  -d '{
    "x": 3.0,
    "y": 1.0,
    "z": 0.4,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0
  }'
```

响应格式与 `/api/navigation/goal` 相同。

## 10. 定位接口

### 10.1 重定位

```http
POST /api/open3d_loc/relocalize
Content-Type: application/json
```

调用：

```bash
curl -X POST http://127.0.0.1:8000/api/open3d_loc/relocalize \
  -H 'Content-Type: application/json' \
  -d '{
    "x": 1.0,
    "y": 2.0,
    "z": 0.4,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0
  }'
```

成功响应：

```json
{
  "success": true,
  "message": "relocalization succeeded"
}
```

重定位会等待请求之后的新 `/Odometry_open3d`。同一时间只允许一个重定位请求。默认 API 响应超时是 30 秒，应当大于 Open3D 定位节点自身的重定位超时。

### 10.2 获取当前位姿

```http
GET /api/open3d_loc/get_pose
```

调用：

```bash
curl http://127.0.0.1:8000/api/open3d_loc/get_pose
```

成功响应：

```json
{
  "success": true,
  "message": "ok",
  "pose": {
    "x": 1.0,
    "y": 2.0,
    "z": 0.4,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0,
    "yaw": 0.0
  }
}
```

尚未收到有效 `/Odometry_open3d` 或正在进行 Service 重定位时，ROS Service 会返回 `success=false`，此时响应中不包含 `pose`。

### 10.3 计算位姿偏差

```http
POST /api/open3d_loc/pose_deviation
Content-Type: application/json
```

请求体表示参考位姿：

```bash
curl -X POST http://127.0.0.1:8000/api/open3d_loc/pose_deviation \
  -H 'Content-Type: application/json' \
  -d '{
    "x": 1.0,
    "y": 2.0,
    "z": 0.4,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0
  }'
```

成功响应：

```json
{
  "success": true,
  "message": "ok",
  "current_pose": {
    "position": {
      "x": 1.1,
      "y": 1.9,
      "z": 0.4
    },
    "orientation": {
      "qx": 0.0,
      "qy": 0.0,
      "qz": 0.05,
      "qw": 0.9987
    },
    "yaw": 0.1
  },
  "error_x": 0.1,
  "error_y": -0.1,
  "distance_xy": 0.1414,
  "yaw_error_rad": 0.1,
  "yaw_error_deg": 5.7296
}
```

`error_x` 和 `error_y` 是参考位姿自身航向坐标系下的平面误差。

## 11. 地图接口

### 11.1 保存 FAST-LIO 地图

```http
POST /api/map/save
```

调用：

```bash
curl -X POST http://127.0.0.1:8000/api/map/save
```

响应示例：

```json
{
  "success": true,
  "message": "Map saved."
}
```

如果 Fast-LIO 的 `pcd_save_en` 未启用，可能返回：

```json
{
  "success": false,
  "message": "Map save disabled."
}
```

## 12. 底盘桥接口

### 12.1 启用底盘桥

```http
POST /api/go2_cmd_vel_bridge/enable
Content-Type: application/json
```

调用：

```bash
curl -X POST http://127.0.0.1:8000/api/go2_cmd_vel_bridge/enable \
  -H 'Content-Type: application/json' \
  -d '{"data": true}'
```

启用前必须存在有效且未超时的：

- `/Odometry_open3d` 定位心跳；
- Unitree SportModeState 心跳。

即使启用成功，底盘桥仍会等待一条新的 `/cmd_vel`，不会复用启用前的旧速度命令。

### 12.2 禁用底盘桥

```bash
curl -X POST http://127.0.0.1:8000/api/go2_cmd_vel_bridge/enable \
  -H 'Content-Type: application/json' \
  -d '{"data": false}'
```

禁用操作会清空目标速度并请求机器人停止运动。

响应格式：

```json
{
  "success": true,
  "message": "bridge enabled; waiting for a new cmd_vel"
}
```

实际 `message` 由 ROS 2 底盘桥节点返回。

## 13. HTTP 状态码

| HTTP 状态码 | 含义 |
|---:|---|
| `200` | HTTP 调用完成；仍需检查 JSON 中的 `success` |
| `401` | 已设置 `KN_NAV_API_KEY`，但请求未携带正确密钥 |
| `422` | 请求字段缺失、类型错误，或包含非有限数值 |
| `502` | ROS Service 调用发生异常或未返回响应 |
| `503` | 对应 ROS Service 未启动或当前不可发现 |
| `504` | 等待 ROS Service 响应超时 |

ROS Service 自身拒绝请求时，HTTP 状态通常仍为 `200`，具体失败原因位于：

```json
{
  "success": false,
  "message": "具体原因"
}
```

## 14. 常见问题

### 14.1 FastAPI 依赖缺失

错误：

```text
FastAPI dependencies are missing
```

处理：

```bash
python3 -m pip install fastapi uvicorn
```

### 14.2 ROS 2 Python 接口不可用

错误包含：

```text
ROS 2 Python interfaces are unavailable
```

处理：

```bash
source /opt/ros/humble/setup.bash
source /home/ehuy/KN/kn_nav/install/setup.bash
```

如果仍然失败，需要先通过 `colcon build` 编译工作区，以生成 `open3d_loc.srv` 的 Python 接口。

### 14.3 API 返回 503

检查对应 ROS Service 是否存在：

```bash
ros2 service list -t
```

也可以检查 API 看到的状态：

```bash
curl http://127.0.0.1:8000/api/services
```

### 14.4 重定位返回 504

重定位可能需要等待新的定位结果。可以适当增加 API 超时：

```bash
python3 src/tools/ros2_service_api.py --service-timeout 40
```

同时确认 Open3D 定位节点持续收到点云和 `/Odometry_loc`。

### 14.5 获取位姿返回 `success=false`

确认：

- `global_localization_node` 正常运行；
- `localization_service_node` 正常运行；
- `/Odometry_open3d` 持续发布有效数据；
- 当前没有正在等待结果的重定位请求。

## 15. 安全说明

- 默认只监听 `127.0.0.1`。
- 监听 `0.0.0.0` 时，建议设置 `KN_NAV_API_KEY`，并通过防火墙限制访问来源。
- `/api/go2_cmd_vel_bridge/enable` 会改变机器人的可运动状态，启用前应确认定位、场地和急停措施正常。
- 发布导航目标前应确认目标位于正确地图、正确楼层和可通行区域。
- HTTP 接口的 `success=true` 不等价于机器人已经安全到达目标。
