# KN 导航 Web API 接口文档

本文档面向接口调用方，描述 KN 导航系统当前提供的 HTTP API。调用方不需要了解 ROS 2 内部实现，只需按照本文档发送 HTTP 请求并处理响应。

对应服务程序：`ros2_service_api.py`

## 1. 基本信息

| 项目 | 值 |
|---|---|
| 协议 | HTTP |
| 默认端口 | `8000` |
| 数据格式 | JSON |
| 字符编码 | UTF-8 |
| API 前缀 | `/api` |
| Swagger 文档 | `/docs` |
| OpenAPI 描述 | `/openapi.json` |
| 全局坐标系 | `map` |
| 距离单位 | 米（m） |
| 角度单位 | 弧度（rad） |

服务与调用方在同一台机器时，Base URL 为：

```text
http://127.0.0.1:8000
```

服务运行在机器人上、调用方在其他机器时，应使用机器人的实际 IP：

```text
http://<机器人IP>:8000
```

例如：

```text
http://192.168.1.100:8000
```

`0.0.0.0` 只用于服务端监听，不能作为客户端请求地址。

## 2. 通用调用约定

### 2.1 请求头

所有包含 Body 的接口必须发送：

```http
Content-Type: application/json
```

Body 必须是 JSON 对象。例如：

```json
{
  "data": false
}
```

不要把整个对象作为字符串发送。下面的格式是错误的：

```json
"{\"data\": false}"
```

如果请求被作为 `text/plain` 发送，接口可能返回 HTTP 422，并提示输入不是有效对象。

### 2.2 API Key

服务端未设置 `KN_NAV_API_KEY` 时，不需要认证。

服务端设置了 `KN_NAV_API_KEY` 时，所有 `/api/*` 请求必须增加：

```http
X-API-Key: <服务端配置的密钥>
```

密钥缺失或错误时返回 HTTP 401：

```json
{
  "detail": "Invalid or missing X-API-Key"
}
```

`GET /`、`/docs` 和 `/openapi.json` 不进行 API Key 校验。

### 2.3 HTTP 状态与业务状态

业务接口通常返回：

```json
{
  "success": true,
  "message": "业务结果说明"
}
```

调用方必须同时检查：

1. HTTP 状态码；
2. JSON 中的 `success`。

HTTP 200 只表示 HTTP 请求和 ROS Service 调用已经完成，不保证业务成功。底层 ROS 节点可能返回：

```json
{
  "success": false,
  "message": "具体失败原因"
}
```

## 3. 接口总览

| 序号 | 方法 | 路径 | 说明 |
|---:|---|---|---|
| 1 | `POST` | `/api/open3d_loc/relocalize` | 使用完整四元数执行重定位 |
| 2 | `POST` | `/api/navigation/points` | 获取当前位置并保存为命名导航点 |
| 3 | `POST` | `/api/navigation/goal` | 发布保存点或直接坐标作为导航目标 |
| 4 | `POST` | `/api/go2_cmd_vel_bridge/enable` | 开启或关闭底盘速度桥 |
| 5 | `GET` | `/api/navigation/points` | 获取全部导航点 |
| 6 | `PATCH` | `/api/navigation/points/rename` | 只修改导航点名称 |
| 7 | `POST` | `/api/navigation/queue` | 将一个导航目标加入顺序执行队列 |
| 8 | `GET` | `/api/navigation/queue` | 查询当前任务、等待队列和历史记录 |
| 9 | `GET` | `/api/robot/status` | 获取机器狗当前位置、四元数、速度和时间戳 |
| 10 | `GET` | `/api/current_map` | 获取最新的当前地图状态 |
| 11 | `GET` | `/api/localization_status` | 获取最新的Open3D定位状态 |
| 12 | `GET` | `/api/navigation_status` | 获取最新的导航执行状态 |
| 13 | `POST` | `/api/switch_map` | 同时切换定位PCD和规划tomogram |
| 14 | `POST` | `/api/restart_navigation` | 软复位导航或请求完整重启 |
| 诊断 | `GET` | `/api/health` | 获取 API 和 ROS 网关状态 |
| 诊断 | `GET` | `/api/services` | 获取底层 ROS Service 就绪状态 |

## 4. 重定位

使用给定的 `base_link` 在 `map` 坐标系下的三维位置和四元数执行 Open3D 重定位。

### 4.1 请求

```http
POST /api/open3d_loc/relocalize
Content-Type: application/json
```

请求 Body：

```json
{
  "x": 1.2327408046653152,
  "y": -0.09142521170438922,
  "z": -0.5001805743966447,
  "qx": 0.0,
  "qy": 0.0,
  "qz": 0.741289779172096,
  "qw": 0.671185118499349
}
```

### 4.2 请求字段

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `x` | number | 是 | `map` 坐标系 X，单位 m |
| `y` | number | 是 | `map` 坐标系 Y，单位 m |
| `z` | number | 是 | `map` 坐标系 Z，单位 m |
| `qx` | number | 是 | 四元数 X |
| `qy` | number | 是 | 四元数 Y |
| `qz` | number | 是 | 四元数 Z |
| `qw` | number | 是 | 四元数 W |

所有字段必须是有限数值，不允许 `NaN` 或无穷值。四元数模长必须大于等于 `1e-6`。

API 会自动归一化合法的非单位四元数，并将归一化后的四元数发送给 ROS。

### 4.3 成功响应

```json
{
  "success": true,
  "message": "relocalization succeeded",
  "pose": {
    "x": 1.2327408046653152,
    "y": -0.09142521170438922,
    "z": -0.5001805743966447,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.741289779172096,
    "qw": 0.671185118499349
  },
  "input_quaternion_norm": 1.0,
  "quaternion_normalized": false
}
```

| 响应字段 | 类型 | 说明 |
|---|---|---|
| `success` | boolean | 重定位是否成功 |
| `message` | string | ROS 重定位结果说明 |
| `pose` | object | 实际发送给 ROS 的归一化位姿 |
| `input_quaternion_norm` | number | 输入四元数归一化前的模长 |
| `quaternion_normalized` | boolean | API 是否对输入四元数进行了归一化 |

### 4.4 业务失败响应

规定时间内没有收到新的有效定位结果：

```json
{
  "success": false,
  "message": "timeout waiting for relocalization result on /Odometry_open3d",
  "pose": {
    "x": 1.0,
    "y": 2.0,
    "z": 0.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0
  },
  "input_quaternion_norm": 1.0,
  "quaternion_normalized": false
}
```

已有重定位请求正在执行时：

```json
{
  "success": false,
  "message": "another relocalize request is running",
  "pose": {
    "x": 1.0,
    "y": 2.0,
    "z": 0.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0
  }
}
```

重定位是长耗时接口。调用方应等待该请求完成，避免重复提交。

### 4.5 参数错误

四元数模长过小：

```http
HTTP/1.1 422 Unprocessable Entity
```

```json
{
  "detail": "Quaternion norm is too small"
}
```

字段缺失或类型错误时返回标准 FastAPI 422 响应。

## 5. 保存导航点（打点）

该接口获取机器人当前定位，并将其保存为具有唯一名称的导航点。

底层定位成功后才会写入当前离线地图对应的 JSON 文件。定位失败不会写入文件。每个导航点都会获得一个唯一的六位字符串编码。

### 5.1 请求

```http
POST /api/navigation/points
Content-Type: application/json
```

请求 Body：

```json
{
  "name": "office",
  "overwrite": false
}
```

### 5.2 请求字段

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| `name` | string | 是 | 无 | 导航点唯一名称，去除首尾空格后不能为空，最长 128 字符 |
| `overwrite` | boolean | 否 | `false` | 同名点存在时是否覆盖 |

调用方应发送标准 JSON 布尔值（小写）：

```json
false
```

不要依赖字符串到布尔值的自动转换：

```json
"false"
```

不能使用 Markdown 符号：

```text
**false**
```

### 5.3 成功响应

```json
{
  "success": true,
  "message": "Navigation point saved: office",
  "map_name": "global_map_downsize",
  "point": {
    "name": "office",
    "code": "483271",
    "frame_id": "map",
    "child_frame_id": "base_link",
    "x": 1.520143605688493,
    "y": 7.289045952913274,
    "z": -0.5119403031633517,
    "qx": 0.0,
    "qy": 0.0,
    "qz": -0.650156,
    "qw": 0.759801,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": -1.4155799076628224
  }
}
```

### 5.4 定位不可用

尚未获得有效定位时，HTTP 状态为 200，但 `success=false`：

```json
{
  "success": false,
  "message": "no current base_link pose from /Odometry_open3d"
}
```

### 5.5 同名点冲突

同名导航点存在且 `overwrite=false` 时：

```http
HTTP/1.1 409 Conflict
```

```json
{
  "detail": "Navigation point already exists: office"
}
```

需要覆盖时发送：

```json
{
  "name": "office",
  "overwrite": true
}
```

### 5.6 导航点存储

默认存储文件位于 API 脚本同目录，文件名取当前 `/global_localization_node` 的 `path_map` 参数中的地图文件名。例如：

```text
global_map_downsize.json
```

API 启动时：

- 文件不存在：自动创建并初始化为 `{}`；
- 文件为空：自动初始化为 `{}`；
- 文件包含合法 JSON：保留原内容；
- 旧导航点没有六位编码或编码重复：启动时自动补齐唯一编码；
- 文件非空但 JSON 损坏：不会覆盖，接口返回 HTTP 500。

## 6. 获取全部导航点

返回当前离线地图 JSON 中保存的全部导航点，按名称排序。

### 6.1 请求

```http
GET /api/navigation/points
```

该接口不接收 Body，也不支持 `?name=office` 查询单个点。

### 6.2 成功响应

```json
{
  "success": true,
  "map_name": "global_map_downsize",
  "map_path": "/home/nav_map/cross-floor-kn/global_map_downsize.pcd",
  "count": 2,
  "points": [
    {
      "name": "office",
      "code": "483271",
      "frame_id": "map",
      "child_frame_id": "base_link",
      "x": 1.520143605688493,
      "y": 7.289045952913274,
      "z": -0.5119403031633517,
      "qx": 0.0,
      "qy": 0.0,
      "qz": -0.650156,
      "qw": 0.759801,
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": -1.4155799076628224
    },
    {
      "name": "start",
      "code": "906154",
      "frame_id": "map",
      "child_frame_id": "base_link",
      "x": 1.2327408046653152,
      "y": -0.09142521170438922,
      "z": -0.5001805743966447,
      "qx": 0.0,
      "qy": 0.0,
      "qz": 0.741289779172096,
      "qw": 0.671185118499349,
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 1.6699799381943505
    }
  ]
}
```

没有导航点时：

```json
{
  "success": true,
  "map_name": "global_map_downsize",
  "map_path": "/home/nav_map/cross-floor-kn/global_map_downsize.pcd",
  "count": 0,
  "points": []
}
```

### 6.3 存储文件错误

```http
HTTP/1.1 500 Internal Server Error
```

```json
{
  "detail": "Unable to read navigation points file: ..."
}
```

## 7. 发布导航目标

该接口支持两种互斥的调用方式：

1. 使用已保存导航点的名称；
2. 直接提供 `x/y/z/yaw`。

`success=true` 只表示目标已经发布到 ROS `/goal_pose`，不表示路径规划成功、机器人开始运动或机器人已经到达。

### 7.1 使用保存点名称

```http
POST /api/navigation/goal
Content-Type: application/json
```

请求 Body：

```json
{
  "name": "office"
}
```

成功响应：

```json
{
  "success": true,
  "message": "goal published on /goal_pose",
  "goal": {
    "source": "saved_point",
    "x": 1.520143605688493,
    "y": 7.289045952913274,
    "z": -0.5119403031633517,
    "yaw": -1.4155799076628224,
    "name": "office"
  }
}
```

导航点不存在时：

```http
HTTP/1.1 404 Not Found
```

```json
{
  "detail": "Navigation point not found: office"
}
```

### 7.2 使用直接坐标

请求 Body：

```json
{
  "x": 3.0,
  "y": 1.0,
  "z": 0.4,
  "yaw": 1.57
}
```

字段说明：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| `x` | number | 是 | 无 | `map` 坐标系 X，单位 m |
| `y` | number | 是 | 无 | `map` 坐标系 Y，单位 m |
| `z` | number | 否 | `0.0` | `map` 坐标系 Z，单位 m |
| `yaw` | number | 否 | `0.0` | 目标航向角，单位 rad |

成功响应：

```json
{
  "success": true,
  "message": "goal published on /goal_pose",
  "goal": {
    "source": "coordinates",
    "x": 3.0,
    "y": 1.0,
    "z": 0.4,
    "yaw": 1.57
  }
}
```

### 7.3 错误组合

不能同时提供名称和坐标：

```json
{
  "name": "office",
  "x": 3.0,
  "y": 1.0
}
```

返回 HTTP 422：

```json
{
  "detail": "Provide either name or coordinates, not both"
}
```

未提供名称时，`x` 和 `y` 都是必填字段。

## 8. 开启或关闭底盘速度桥

底盘桥只有在开启状态下才会把 `/cmd_vel` 速度消息发送给机器人。

### 8.1 开启

```http
POST /api/go2_cmd_vel_bridge/enable
Content-Type: application/json
```

请求 Body：

```json
{
  "data": true
}
```

开启成功：

```json
{
  "success": true,
  "message": "bridge enabled; waiting for a new cmd_vel"
}
```

开启前，底盘桥要求：

- 已收到有效且未超时的定位心跳；
- 已收到有效且未超时的 Unitree SportModeState 心跳；
- 机器人停止指令调用成功。

条件不满足时，HTTP 状态通常为 200，但 `success=false`：

```json
{
  "success": false,
  "message": "cannot enable: odometry heartbeat is missing or stale"
}
```

或者：

```json
{
  "success": false,
  "message": "cannot enable: sport state heartbeat is missing or stale"
}
```

### 8.2 关闭

请求 Body：

```json
{
  "data": false
}
```

成功响应：

```json
{
  "success": true,
  "message": "bridge disabled and StopMove sent"
}
```

关闭操作会清空当前目标速度并要求机器人停止。

### 8.3 Body 类型错误

该接口要求 JSON 对象。如果请求被作为字符串或 `text/plain` 发送，会返回 HTTP 422：

```json
{
  "detail": [
    {
      "type": "model_attributes_type",
      "loc": ["body"],
      "msg": "Input should be a valid dictionary or object to extract fields from",
      "input": "{\n  \"data\": false\n}"
    }
  ]
}
```

遇到该错误时应检查请求头是否为：

```http
Content-Type: application/json
```

## 9. 修改导航点名称

该接口只修改当前地图导航点的名称，不修改六位编码、XYZ、四元数、RPY等数据。

### 9.1 请求

```http
PATCH /api/navigation/points/rename
Content-Type: application/json
```

请求 Body：

```json
{
  "old_name": "office",
  "new_name": "meeting_room"
}
```

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `old_name` | string | 是 | 当前已经存在的导航点名称 |
| `new_name` | string | 是 | 修改后的导航点名称 |

两个名称都会去除首尾空格，不能为空，最长为128个字符。

### 9.2 成功响应

```json
{
  "success": true,
  "message": "Navigation point renamed: office -> meeting_room",
  "old_name": "office",
  "point": {
    "name": "meeting_room",
    "code": "483271",
    "frame_id": "map",
    "child_frame_id": "base_link",
    "x": 1.520143605688493,
    "y": 7.289045952913274,
    "z": -0.5119403031633517,
    "qx": 0.0,
    "qy": 0.0,
    "qz": -0.650156,
    "qw": 0.759801,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": -1.4155799076628224
  }
}
```

旧名称不存在时返回 HTTP 404：

```json
{
  "detail": "Navigation point not found: office"
}
```

新名称已经被其他导航点占用时返回 HTTP 409：

```json
{
  "detail": "Navigation point already exists: meeting_room"
}
```

## 10. 顺序导航任务队列

导航队列用于持续接收新的目标，并严格按照加入顺序逐个发送。当前目标完成后才会发送下一个目标。

完成条件为：

```text
当前位置与目标的XY距离 <= 0.15 m
当前航向与目标的yaw误差 <= 0.10 rad
```

完成状态通过 `/open3d_loc/pose_deviation` 判断。ROS服务暂时不可用时，队列会等待并重试，不会跳过当前目标。

### 10.1 将保存点加入队列

```http
POST /api/navigation/queue
Content-Type: application/json
```

请求 Body：

```json
{
  "name": "office"
}
```

成功响应：

```json
{
  "success": true,
  "message": "navigation goal added to queue",
  "task": {
    "task_id": "a47fe3bf52b74ba688fe65d76de74a54",
    "sequence": 1,
    "status": "queued",
    "message": "waiting in navigation queue",
    "created_at": "2026-07-24T10:20:30.123456+00:00",
    "started_at": null,
    "finished_at": null,
    "goal": {
      "source": "saved_point",
      "name": "office",
      "x": 1.520143605688493,
      "y": 7.289045952913274,
      "z": -0.5119403031633517,
      "yaw": -1.4155799076628224
    },
    "queue_position": 1
  }
}
```

`task_id` 是本次队列任务的唯一ID，`sequence` 是API本次运行期间的任务添加顺序。

### 10.2 将直接坐标加入队列

请求路径与保存点方式相同，请求 Body 为：

```json
{
  "x": 3.0,
  "y": 1.0,
  "z": 0.4,
  "yaw": 1.57
}
```

字段要求与 `POST /api/navigation/goal` 完全相同：只能使用名称或直接坐标，不能同时使用。

### 10.3 查询队列状态

```http
GET /api/navigation/queue
```

该接口不接收 Body。响应示例：

```json
{
  "success": true,
  "active": {
    "task_id": "a47fe3bf52b74ba688fe65d76de74a54",
    "sequence": 1,
    "status": "navigating",
    "message": "navigation goal is active",
    "created_at": "2026-07-24T10:20:30.123456+00:00",
    "started_at": "2026-07-24T10:20:30.223456+00:00",
    "finished_at": null,
    "goal": {
      "source": "saved_point",
      "name": "office",
      "x": 1.520143605688493,
      "y": 7.289045952913274,
      "z": -0.5119403031633517,
      "yaw": -1.4155799076628224
    },
    "distance_xy": 1.26,
    "yaw_error_rad": 0.34
  },
  "queued_count": 1,
  "queued": [
    {
      "task_id": "b920fda67eca44de8754eca1dc08ea1c",
      "sequence": 2,
      "status": "queued",
      "message": "waiting in navigation queue",
      "created_at": "2026-07-24T10:20:35.123456+00:00",
      "started_at": null,
      "finished_at": null,
      "goal": {
        "source": "saved_point",
        "name": "meeting_room",
        "x": 4.0,
        "y": 2.0,
        "z": 0.0,
        "yaw": 0.0
      },
      "queue_position": 1
    }
  ],
  "history": [],
  "distance_tolerance": 0.15,
  "yaw_tolerance": 0.1
}
```

任务状态说明：

| 状态 | 说明 |
|---|---|
| `queued` | 等待前面的任务完成 |
| `publishing` | 正在等待或调用目标发布服务 |
| `navigating` | 目标已发布，正在等待机器人到达 |
| `completed` | 距离和航向误差均达到完成阈值 |
| `failed` | 目标发布被拒绝或任务发生不可恢复错误 |

调用注意事项：

- 需要顺序导航时，所有目标都必须通过本接口加入；
- `POST /api/navigation/goal` 会绕过队列，可能打断当前队列任务；
- 保存点在入队时会立即解析为坐标，入队后修改名称不会影响已经接受的任务；
- 队列和历史记录保存在API进程内存中，API重启后清空；
- 当前目标长期无法到达时，后续目标会继续等待。

### 10.4 返回机制与当前限制

`POST /api/navigation/queue` 每次只能加入一个导航目标。如果需要依次导航三个点，调用方需要连续发送三次POST请求，而不是在一个Body中发送三个点。

每次POST只返回该目标已经进入队列以及对应的`task_id`。HTTP请求返回后连接即结束，因此第一个目标完成时，服务端不会通过原POST请求再次返回消息，也不会主动向调用方推送完成通知。

调用方必须定时请求：

```http
GET /api/navigation/queue
```

例如三个任务中的第一个完成后，查询结果中的状态关系为：

```text
第一个目标：位于 history，status=completed，message=navigation goal reached
第二个目标：位于 active，通常为 status=navigating
第三个目标：位于 queued，status=queued
```

前端建议每500毫秒到1秒查询一次队列状态，并通过`task_id`判断指定任务是否完成。不要根据数组下标长期绑定任务，因为任务完成后会从`queued`移动到`active`，最后移动到`history`。

当前接口未提供WebSocket、SSE或HTTP回调，因此不支持服务端主动推送。如果后续需要实时完成通知，需要另外增加WebSocket或SSE接口。

## 11. 获取机器狗实时状态

该接口返回 `/Odometry_open3d` 最新一帧里程计中的位置、四元数、速度和时间戳。

### 11.1 请求

```http
GET /api/robot/status
```

该接口不接收 Body。

### 11.2 成功响应

```json
{
  "success": true,
  "state": "moving",
  "topic": "/Odometry_open3d",
  "frame_id": "map",
  "child_frame_id": "base_link",
  "position": {
    "x": 1.2,
    "y": -0.5,
    "z": 0.3
  },
  "orientation": {
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.5,
    "qw": 0.8660254
  },
  "velocity": {
    "linear": {
      "x": 0.4,
      "y": 0.3,
      "z": 0.0,
      "speed": 0.5
    },
    "angular": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.2,
      "speed": 0.2
    }
  },
  "timestamp": {
    "sec": 123,
    "nanosec": 456000000,
    "seconds": 123.456,
    "received_at": "2026-07-24T10:20:30.123456+00:00",
    "age_seconds": 0.03
  }
}
```

### 11.3 响应字段

| 字段 | 说明 |
|---|---|
| `state` | `moving`表示线速度或角速度大于 `0.01`；否则为`stopped` |
| `position` | `map`坐标系下的XYZ位置，单位m |
| `orientation` | 当前姿态四元数，顺序为qx、qy、qz、qw |
| `velocity.linear` | 三轴线速度和合速度，单位m/s |
| `velocity.angular` | 三轴角速度和合角速度，单位rad/s |
| `timestamp.sec/nanosec` | ROS里程计消息原始时间戳 |
| `timestamp.seconds` | ROS时间戳转换后的秒数 |
| `timestamp.received_at` | API接收到该消息时的UTC时间 |
| `timestamp.age_seconds` | 请求发生时该状态数据已经存在的时间 |

API尚未收到 `/Odometry_open3d` 时返回 HTTP 503：

```json
{
  "detail": "Robot status is unavailable: no odometry received from /Odometry_open3d"
}
```

## 12. 获取当前地图状态

API节点持续监听ROS Topic `/current_map`，该HTTP接口返回缓存的最新一帧。前端不需要直接连接ROS。

### 12.1 请求

```http
GET /api/current_map
```

该接口没有Body。

### 12.2 成功响应

```json
{
  "success": true,
  "topic": "/current_map",
  "frame_id": "map",
  "state": 2,
  "state_name": "LOADED",
  "map_name": "outdoor",
  "reason": "ok",
  "timestamp": {
    "sec": 1785123456,
    "nanosec": 123456789,
    "seconds": 1785123456.1234567,
    "received_at": "2026-07-27T06:10:00.000000+00:00",
    "age_seconds": 0.35
  }
}
```

### 12.3 地图状态

| `state` | `state_name` | 含义 | 前端处理 |
|---:|---|---|---|
| `0` | `UNLOADED` | 尚未加载地图 | 禁用导航目标发送 |
| `1` | `LOADING` | 正在切换地图 | 禁用目标发送和重复切图 |
| `2` | `LOADED` | 地图已加载 | 继续等待定位进入`TRACKING` |
| `3` | `FAILED` | 地图加载失败 | 显示`reason`，禁止导航 |

`/current_map`是事件触发状态，不是周期心跳。`timestamp.age_seconds`较大不一定表示异常，只要节点正常，该数据仍然是最新地图状态。

API启动后尚未收到地图状态时返回HTTP 503：

```json
{
  "detail": "ROS status is unavailable: no message received from /current_map"
}
```

## 13. 获取定位状态

API节点持续监听ROS Topic `/localization_status`。前端建议每`250～500 ms`轮询该HTTP接口。

### 13.1 请求

```http
GET /api/localization_status
```

### 13.2 成功响应

```json
{
  "success": true,
  "topic": "/localization_status",
  "frame_id": "map",
  "state": 3,
  "state_name": "TRACKING",
  "map_name": "outdoor",
  "reason": "ok",
  "fitness": 0.95,
  "timestamp": {
    "sec": 1785123456,
    "nanosec": 123456789,
    "seconds": 1785123456.1234567,
    "received_at": "2026-07-27T06:10:00.000000+00:00",
    "age_seconds": 0.08
  },
  "stale_after_seconds": 1.0,
  "stale": false
}
```

### 13.3 定位状态

| `state` | `state_name` | 含义 | 是否允许发导航目标 |
|---:|---|---|---|
| `0` | `UNINITIALIZED` | 未完成重定位 | 否 |
| `1` | `INITIALIZING` | 正在执行初始定位 | 否 |
| `2` | `INIT_SUCCESS` | 初始定位刚成功 | 建议继续等待`TRACKING` |
| `3` | `TRACKING` | 稳定跟踪 | 是，但仍需检查地图和导航状态 |
| `4` | `TRACKING_WARN` | 定位质量波动 | 建议暂停发新目标 |
| `5` | `TRACKING_LOST` | 定位丢失 | 否，需要重新定位 |
| `6` | `MAP_SWITCHING` | 定位地图切换中 | 否 |

`fitness`只用于辅助显示，不建议前端使用固定fitness阈值取代`state`判断。`stale=true`表示API超过1秒未收到新状态，此时应禁止发新目标并提示状态源异常。

尚未收到ROS消息时返回HTTP 503：

```json
{
  "detail": "ROS status is unavailable: no message received from /localization_status"
}
```

## 14. 获取导航状态

API节点持续监听ROS Topic `/navigation_status`。前端建议每`250～500 ms`轮询该HTTP接口。

### 14.1 请求

```http
GET /api/navigation_status
```

### 14.2 成功响应

```json
{
  "success": true,
  "topic": "/navigation_status",
  "frame_id": "map",
  "state": 5,
  "state_name": "NAVIGATING",
  "map_name": "outdoor",
  "reason": "ok",
  "goal_active": true,
  "distance_to_goal": 1.25,
  "remaining_waypoints": 3,
  "timestamp": {
    "sec": 1785123456,
    "nanosec": 123456789,
    "seconds": 1785123456.1234567,
    "received_at": "2026-07-27T06:10:00.000000+00:00",
    "age_seconds": 0.06
  },
  "stale_after_seconds": 1.0,
  "stale": false
}
```

### 14.3 响应字段

| 字段 | 含义 |
|---|---|
| `state/state_name` | 导航状态数值和可读名称 |
| `map_name` | 导航节点当前使用的地图名 |
| `goal_active` | 局部规划器是否持有活动目标 |
| `distance_to_goal` | 当前位置到最终目标的XY直线距离，单位m |
| `remaining_waypoints` | 当前active waypoints数量 |
| `reason` | 简短状态原因 |
| `stale` | 是否超过1秒未收到新消息 |

### 14.4 导航状态

| `state` | `state_name` | 前端显示/处理 |
|---:|---|---|
| `0` | `IDLE` | 空闲，结合地图和定位状态判断是否可发目标 |
| `1` | `WAITING_GOAL` | 等待目标 |
| `2` | `PLANNING_GLOBAL` | 全局规划中 |
| `3` | `GLOBAL_READY` | 全局路径就绪 |
| `4` | `PLANNING_LOCAL` | 局部轨迹生成中 |
| `5` | `NAVIGATING` | 导航中 |
| `6` | `AVOIDING` | 避障/局部重规划中 |
| `7` | `BLOCKED` | 受阻或急停，提示人工处理 |
| `8` | `GOAL_REACHED` | 已抵达目标 |
| `9` | `CANCELED` | 目标已取消 |
| `10` | `FAILED` | 导航失败，显示`reason` |
| `11` | `LOCALIZATION_LOST` | 定位丢失；同时以定位状态接口为准 |
| `12` | `MAP_SWITCHING` | 地图切换中，禁止发目标 |

`stale=true`或尚未收到ROS消息时，前端不应根据旧状态继续发导航目标。未收到消息时返回HTTP 503。

## 15. 切换地图

通过`nav_manager_node`同时切换Open3D定位PCD和PCT全局规划tomogram。这是业务上推荐的唯一切图入口。

### 15.1 请求

```http
POST /api/switch_map
Content-Type: application/json
```

```json
{
  "map_name": "outdoor"
}
```

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `map_name` | string | 是 | `map_profiles.yaml`中`maps`下的profile名；不是PCD或pickle文件名 |

### 15.2 成功响应

```json
{
  "success": true,
  "message": "switched to map: outdoor",
  "map_name": "outdoor"
}
```

`success=true`表示PCD和tomogram都已加载成功。这不表示已完成新地图上的重定位。

### 15.3 业务失败

```json
{
  "success": false,
  "message": "localization map load failed: ...",
  "map_name": "outdoor"
}
```

底层Service不存在时返回HTTP 503；切图超过API的Service超时上限时返回HTTP 504。大地图可由服务端增加`--service-timeout`。

### 15.4 前端调用规则

1. 调用前禁用“发送目标”和“重复切图”按钮。
2. 请求返回`success=true`后，继续等待`/api/current_map.state_name=LOADED`。
3. 重新调用重定位接口。
4. 等待`/api/localization_status.state_name=TRACKING`且`stale=false`。
5. 最后才允许发新地图上的导航目标。

切图前应确保HTTP顺序导航队列为空。当前API队列是独立的上层任务队列，ROS内部软复位不会自动删除尚未发布的HTTP排队任务。

## 16. 导航复位

### 16.1 请求

```http
POST /api/restart_navigation
Content-Type: application/json
```

常规软复位：

```json
{
  "mode": 0
}
```

| `mode` | 名称 | 功能 | 使用建议 |
|---:|---|---|---|
| `0` | `SOFT_RESET` | 发布零速度、清空waypoint并复位局部导航 | 常规停止或异常恢复优先使用 |
| `1` | `FULL_RESTART` | 请求执行预配置的外部重启命令 | 只在服务端已配置外部命令时使用 |

`mode`只能是`0`或`1`，其他值返回HTTP 422。

### 16.2 响应

```json
{
  "success": true,
  "accepted": true,
  "message": "navigation soft reset completed",
  "mode": 0,
  "mode_name": "SOFT_RESET"
}
```

`success`和`accepted`值相同。`accepted=false`表示请求未被底层接受，例如`mode=1`但未配置外部重启命令。

`SOFT_RESET`不会重启进程、切换地图或恢复定位。如果定位已丢失，还需要调用重定位接口。复位前同样应停止向HTTP导航队列添加新任务，并确保待执行队列为空。

## 17. 健康检查

### 17.1 请求

```http
GET /api/health
```

### 17.2 响应

```json
{
  "success": true,
  "node": "kn_nav_fastapi_gateway_41569",
  "services": {
    "/open3d_loc/relocalize": true,
    "/open3d_loc/get_pose": true,
    "/open3d_loc/publish_goal": true,
    "/open3d_loc/pose_deviation": true,
    "/go2_cmd_vel_bridge/enable": true,
    "/switch_map": true,
    "/restart_navigation": true,
    "/global_localization_node/get_parameters": true
  },
  "topics": {
    "/current_map": true,
    "/localization_status": true,
    "/navigation_status": true
  }
}
```

`success=true` 只表示 API 内部 ROS 节点和 executor 正常。`services`表示对应Service是否就绪；`topics`表示API启动后是否至少收到过一帧对应状态。具体业务是否可用，仍要检查状态接口中的`state_name`和`stale`。

## 18. ROS Service 状态

### 18.1 请求

```http
GET /api/services
```

### 18.2 响应

```json
{
  "success": true,
  "services": {
    "/open3d_loc/relocalize": true,
    "/open3d_loc/get_pose": true,
    "/open3d_loc/publish_goal": true,
    "/open3d_loc/pose_deviation": true,
    "/go2_cmd_vel_bridge/enable": false,
    "/switch_map": true,
    "/restart_navigation": true,
    "/global_localization_node/get_parameters": true
  }
}
```

Service 为 `false` 时，对应业务接口通常返回 HTTP 503。

## 19. HTTP 错误码

| 状态码 | 含义 | 调用方处理建议 |
|---:|---|---|
| `200` | HTTP 和 ROS 调用完成 | 继续检查 JSON 中的 `success` |
| `401` | API Key 缺失或错误 | 检查 `X-API-Key` |
| `404` | 指定导航点不存在 | 刷新导航点列表或检查名称 |
| `409` | 保存点名称冲突 | 更换名称或使用 `overwrite=true` |
| `422` | Body、字段或数值不合法 | 检查 JSON 对象、字段类型和必填字段 |
| `500` | 导航点文件损坏或服务内部错误 | 记录 `detail` 并通知服务维护人员 |
| `502` | ROS Service 调用异常 | 检查 ROS 节点日志 |
| `503` | ROS Service不可用，或API尚未收到状态Topic | 检查`/api/health`中的`services/topics` |
| `504` | ROS Service 响应超时 | 提示超时，避免无限自动重试 |

HTTP 层错误通常使用：

```json
{
  "detail": "错误说明"
}
```

FastAPI 字段校验错误使用：

```json
{
  "detail": [
    {
      "type": "错误类型",
      "loc": ["body", "字段名"],
      "msg": "错误说明",
      "input": "错误输入"
    }
  ]
}
```

## 20. 建议调用流程

建议按以下顺序组织业务调用：

1. 调用`GET /api/health`，确认API、必要Service和三个状态Topic正常；
2. 开始轮询`/api/current_map`、`/api/localization_status`和`/api/navigation_status`；
3. 如果需要切图，先确保HTTP导航队列为空，再调用`POST /api/switch_map`；
4. 等待地图状态进入`LOADED`，然后调用重定位接口；
5. 等待定位状态进入`TRACKING`，并确认`stale=false`；
6. 确认导航状态为`IDLE`、`WAITING_GOAL`、`GOAL_REACHED`或`CANCELED`；
7. 调用`GET /api/robot/status`，确认位姿、速度和数据时间正常；
8. 调用`GET /api/navigation/points`获取导航点，需要时进行保存或重命名；
9. 确认场地和机器人状态安全后，开启底盘速度桥；
10. 单个目标调用`POST /api/navigation/goal`；连续目标调用`POST /api/navigation/queue`；
11. 导航期间持续查看`/api/navigation_status`，队列任务另外查看`/api/navigation/queue`；
12. 出现`TRACKING_LOST`、`BLOCKED`或`FAILED`时停止添加目标，必要时调用`POST /api/restart_navigation`；
13. 任务结束、异常或人工停止时，关闭底盘速度桥。

关闭底盘桥请求：

```json
{
  "data": false
}
```

## 21. 重试建议

- HTTP 422：请求数据错误，不应原样重试；
- HTTP 401、404、409：修正请求后再调用；
- HTTP 503：可以间隔重试，并查询`/api/health`判断是Service未就绪还是Topic尚未收到；
- HTTP 504：应提示用户，避免高频自动重试；
- `success=false`：根据 `message` 判断原因，不要无条件循环调用；
- 重定位接口执行期间不要再次提交重定位请求；
- 导航队列接受请求后，不要再使用直接目标接口覆盖当前任务；
- 机器狗状态接口返回503时，应等待新的里程计数据，不要据此发送导航目标；
- 定位或导航状态返回`stale=true`时，不应继续发新目标；
- 切图失败时不要高频自动重试，应显示`message`并检查地图profile；
- 底盘桥启用失败时不要自动持续重试，应先检查定位和机器人状态心跳。

## 22. 部署与浏览器调用注意事项

当前 API 服务本身未配置 CORS。

如果调用程序运行在浏览器中，并且页面域名、协议或端口与 API 不一致，浏览器可能阻止跨域请求。部署时应由同源网关或反向代理转发 `/api`，或者由服务维护人员统一配置跨域策略。

调用方不应在代码中使用：

```text
http://0.0.0.0:8000
```

应使用：

```text
http://127.0.0.1:8000
```

或者机器人的实际网络地址。

## 23. 安全注意事项

- 发布目标和开启底盘桥是两个独立动作；发布目标不会自动保证底盘桥已开启。
- `success=true` 表示请求被相应节点接受，不表示机器人已经到达。
- 开启底盘桥前必须确认定位正确、目标安全、场地无人且急停措施可用。
- 调用方应始终提供明确的“停止/禁用底盘桥”操作。
- 网络异常、页面关闭或请求超时不能代替停止机器人，机器人仍由底盘桥自身的安全超时机制保护。
- 不要把具有底盘控制权限的 API Key 写入公开仓库或前端公开配置。
