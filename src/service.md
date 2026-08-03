# 导航 Service 与状态 Topic 接口文档

本文档描述 `pct_scan_navigation` 导航链路提供的业务 Service，以及本导航管理功能新增的状态
Topic。ROS 2 每个节点自动生成的 `describe_parameters`、`get_parameters`、`set_parameters`
等参数 Service 不在本文档范围内。

## 使用准备

```bash
cd work_space/kn_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

查看当前实际存在的 Service：

```bash
ros2 service list -t
```

查看当前关键 Topic：

```bash
ros2 topic list -t
```

检查 `/navigation_status` 是否只有一个 publisher：

```bash
ros2 topic info /navigation_status
```

所有位姿接口均使用 `map` 坐标系。四元数顺序为 `qx/qy/qz/qw`；输入四元数会先校验，
再归一化。包含非有限数值或模长过小的四元数会被拒绝。

## 上位机对接建议

上位机建议把本导航系统当成三个状态源和两个控制入口来使用：

| 用途 | 接口 | 上位机建议 |
|---|---|---|
| 当前地图 | `/current_map` | 必须监听。决定当前 UI 显示的地图名，以及是否允许发目标点 |
| 定位状态 | `/localization_status` | 必须监听。只有 `TRACKING` 或刚 `INIT_SUCCESS` 后进入稳定跟踪时，才建议允许发导航目标 |
| 导航状态 | `/navigation_status` | 必须监听。用于显示等待目标、导航中、避障、抵达、失败等状态 |
| 地图切换 | `/switch_map` | 推荐唯一切图入口。不要直接分别调用 PCD 和 pickle reload，除非调试 |
| 导航复位 | `/restart_navigation` | 推荐软停止/清空导航入口。异常恢复优先用 `mode=0` |

上位机最小状态机建议：

1. 启动后等待 `/current_map.state=LOADED`。
2. 等待 `/localization_status.state=TRACKING`。
3. 当 `/navigation_status.state=WAITING_GOAL` 或 `IDLE` 时允许发目标点。
4. 发目标点后监听 `/navigation_status`：
   - `NAVIGATING`：正常执行；
   - `AVOIDING`：局部避障/重规划；
   - `GOAL_REACHED`：到达目标；
   - `BLOCKED` / `FAILED`：需要人工或上层恢复；
   - `MAP_SWITCHING`：切图中，禁止发新目标。
5. 如果 `/localization_status.state=TRACKING_LOST`，认为当前导航已不可信；系统会自动 soft reset，
   上位机应提示重新定位，不要继续沿用旧目标。

几个重要注意事项：

- `/current_map` 是 transient local，后启动的上位机也能收到最近一次地图状态。
- `/navigation_status` 的唯一可信发布源是 `scan_planner_node`。如果发现多个 publisher，说明启动或代码配置有问题。
- `/switch_map` 会阻塞等待 Open3D 地图和 PCT tomogram 都加载完成后再返回。调用期间不要再发目标点。
- `/switch_map` 成功后不会自动重定位，也不会恢复旧目标点；必须重新 init/relocalize，再重新发目标。
- `/restart_navigation mode=0` 只是 ROS 内部软复位，不重启进程、不切地图、不重新定位。
- `reason` 字段是给人和上位机做简短判断用的短字符串，不承诺包含详细诊断；详细 ICP 数值和轨迹细节仍看日志。

## Service 总览

| Service | 类型 | 提供节点 | 启动条件 |
|---|---|---|---|
| `/open3d_loc/relocalize` | `open3d_loc/srv/Relocalize` | `localization_service_node` | `start_open3d_loc:=true` |
| `/open3d_loc/get_pose` | `open3d_loc/srv/GetPose` | `localization_service_node` | `start_open3d_loc:=true` |
| `/open3d_loc/publish_goal` | `open3d_loc/srv/PublishGoal` | `localization_service_node` | `start_open3d_loc:=true` |
| `/open3d_loc/pose_deviation` | `open3d_loc/srv/PoseDeviation` | `localization_service_node` | `start_open3d_loc:=true` |
| `/switch_map` | `pct_scan_navigation/srv/SwitchMap` | `nav_manager_node` | 启动导航管理节点 |
| `/restart_navigation` | `pct_scan_navigation/srv/RestartNavigation` | `nav_manager_node` | 启动导航管理节点 |
| `/global_localization_node/load_map` | `pct_scan_navigation/srv/LoadLocalizationMap` | `global_localization_node` | 启动 Open3D 定位节点 |
| `/pct_global_planner/load_tomogram` | `pct_scan_navigation/srv/LoadTomogram` | `pct_global_planner` | 启动 PCT 全局规划节点 |
| `/scan_planner_node/reset_navigation` | `std_srvs/srv/Trigger` | `scan_planner_node` | 启动 SCAN 局部规划节点 |
| `/map_save` | `std_srvs/srv/Trigger` | `fastlio_mapping` | `start_open3d_loc:=true` |
| `/go2_cmd_vel_bridge/enable` | `std_srvs/srv/SetBool` | `go2_cmd_vel_bridge` | `start_go2_bridge:=true`；Go2/Go2-W 实机启动默认开启该节点 |

`/switch_map` 和 `/restart_navigation` 是推荐给上层系统调用的管理接口。
`/global_localization_node/load_map`、`/pct_global_planner/load_tomogram`、
`/scan_planner_node/reset_navigation` 是导航管理节点内部编排使用的接口，一般不建议绕过
`nav_manager_node` 直接调用，除非在单节点调试。

## 状态 Topic 总览

| Topic | 类型 | 发布节点 | 订阅/使用节点 | 说明 |
|---|---|---|---|---|
| `/localization_status` | `pct_scan_navigation/msg/LocalizationStatus` | `global_localization_node` | `nav_manager_node`、上层监控 | Open3D 定位状态，周期发布 |
| `/navigation_status` | `pct_scan_navigation/msg/NavigationStatus` | `scan_planner_node` | 上层监控 | 导航执行状态；唯一可信发布源是 `scan_planner_node` |
| `/current_map` | `pct_scan_navigation/msg/MapStatus` | `nav_manager_node` | `scan_planner_node`、上层监控 | 当前地图状态；QoS 为 transient local |

状态 topic 当前发布特性：

| Topic | 当前实现频率/QoS | 超时建议 |
|---|---|---|
| `/localization_status` | 约 4Hz，普通 QoS depth=10 | 超过 1s 未更新可认为定位状态源异常 |
| `/navigation_status` | 约 4Hz，普通 QoS depth=10 | 超过 1s 未更新可认为局部规划状态源异常 |
| `/current_map` | 事件触发发布，reliable + transient local，depth=1 | 不按频率判断；后启动也应能收到最近一次状态 |

上位机启动时建议检查：

```bash
ros2 topic echo /current_map --once
ros2 topic echo /localization_status --once
ros2 topic echo /navigation_status --once
```

如果 `/current_map` 收不到，通常是 `nav_manager_node` 没启动或 QoS 不匹配。如果
`/localization_status` 或 `/navigation_status` 收不到，通常是对应定位/局部规划节点没启动。

### 新增的关键话题监听关系

- `nav_manager_node` 监听 `/localization_status`：当定位进入 `TRACKING_LOST` 时触发 soft reset，
  停止当前导航输出。
- `scan_planner_node` 监听 `/current_map`：地图加载中发布 `MAP_SWITCHING` 导航状态，地图加载完成后
  更新 `/navigation_status.map_name`。
- `nav_manager_node` 在 soft reset 和切图前会发布：
  - `/cmd_vel`：一帧零速度；
  - `/scan_planner/waypoints`：空 `nav_msgs/Path`，用于清空动态 waypoint。

## `/switch_map`

按地图名切换 Open3D 定位 PCD 和 PCT tomogram pickle。该接口由 `nav_manager_node` 提供，
是上层系统切图时推荐调用的入口。

### 类型

```text
pct_scan_navigation/srv/SwitchMap
```

### 请求

```text
string map_name
```

`map_name` 必须存在于 `map_profiles.yaml` 的 `maps` 字段下，并且对应 profile 至少包含：

```yaml
pcd_path: "/path/to/map.pcd"
tomo_path: "/path/to/tomogram.pickle"
```

字段说明：

| 字段 | 含义 | 注意事项 |
|---|---|---|
| `map_name` | 要切换到的地图 profile 名称 | 必须和 `map_profiles.yaml` 中 `maps` 下的 key 完全一致；不是 PCD 文件名，也不是 pickle 文件名 |

`map_profiles.yaml` 通常放在：

```text
src/pct_scan_navigation/config/<robot>/map_profiles.yaml
```

示例结构：

```yaml
maps:
  outdoor:
    pcd_path: "/home/kangneng/xq/map/pcd/outdoor.pcd"
    tomo_path: "/home/kangneng/xq/map/tomogram/outdoor_global_voxel002_ground.pickle"
    localization:
      fitness_eval_threshold: 0.2
      threshold_fitness: 0.9
      threshold_fitness_init: 0.9
```

其中 `pcd_path` 和 `tomo_path` 必填；`localization` 可选。`localization` 缺省时，Open3D 定位继续使用节点当前参数。

### 响应

```text
bool success
string message
```

| 字段 | 含义 |
|---|---|
| `success=true` | PCD 和 tomogram 都已加载成功，`/current_map` 已发布 `LOADED` |
| `success=false` | 切图失败，失败原因写在 `message`，`/current_map` 会发布 `FAILED` |
| `message` | 简短人可读结果，例如 `switched to map: outdoor`、`localization map load failed: ...` |

### 行为

- 发布 `/current_map` 为 `LOADING`，`map_name` 为请求的地图名，`reason=switching`。
- 先执行 soft reset：发布零 `/cmd_vel`、发布空 `/scan_planner/waypoints`，并尝试调用
  `/scan_planner_node/reset_navigation`。
- 调用 `/global_localization_node/load_map` 加载新 PCD。
- 调用 `/pct_global_planner/load_tomogram` 加载新 pickle。
- 两者都成功后发布 `/current_map` 为 `LOADED`，`reason=ok`。
- 任意一步失败时返回 `success=false`，并发布 `/current_map` 为 `FAILED`。
- 切图成功后定位会回到 `UNINITIALIZED`，不会沿用旧定位结果，也不会自动恢复旧目标点。

### 上位机注意事项

- 切图期间应禁用“发送目标点”按钮，直到：
  - `/switch_map` 返回 `success=true`；
  - `/current_map.state=LOADED`；
  - `/localization_status.state=TRACKING`。
- 如果 `/switch_map` 返回失败，建议保留在错误页面或提示人工处理，不要立即自动重试很多次。
- 如果 PCD 加载成功但 tomogram 加载失败，service 会返回失败；此时 `/current_map=FAILED`，不要认为定位和规划地图一致。
- service 本身保持同步语义：返回时代表本次切换流程已经成功或失败，不是“开始切图”的异步通知。

### 调用示例

```bash
ros2 service call /switch_map pct_scan_navigation/srv/SwitchMap "{map_name: outdoor}"
ros2 service call /switch_map pct_scan_navigation/srv/SwitchMap "{map_name: cross-floor-kn}"
```

## `/restart_navigation`

导航运行管理重启接口。当前 v1 支持 ROS 内部软复位，并预留外部完整重启入口。

### 类型

```text
pct_scan_navigation/srv/RestartNavigation
```

### 请求

```text
uint8 SOFT_RESET=0
uint8 FULL_RESTART=1
uint8 mode
```

字段说明：

| `mode` 值 | 名称 | 功能 | 上位机使用建议 |
|---:|---|---|---|
| `0` | `SOFT_RESET` | ROS 内部软复位，停止速度输出并清空局部导航状态 | 常规停止、异常恢复优先用这个 |
| `1` | `FULL_RESTART` | 调用外部 supervisor/systemd/launch 管理命令 | 只有配置了外部重启命令时使用 |

### 响应

```text
bool accepted
string message
```

| 字段 | 含义 |
|---|---|
| `accepted=true` | 请求已被接受并完成当前节点能执行的动作 |
| `accepted=false` | 请求未被接受，例如 mode 非法或 `full_restart_command` 未配置 |
| `message` | 简短结果说明 |

### 行为

- `SOFT_RESET=0`：
  - 发布零 `/cmd_vel`；
  - 发布空 `/scan_planner/waypoints`；
  - 尝试调用 `/scan_planner_node/reset_navigation`；
  - reset service 不可用或失败时只打印 warn，不阻塞主流程。
- `FULL_RESTART=1`：
  - 执行 `nav_manager_node` 参数 `full_restart_command` 指定的外部命令；
  - 如果该参数为空，返回 `accepted=false`。

### 上位机注意事项

- `SOFT_RESET` 不会清空 Open3D 定位状态，也不会重新加载地图。
- `SOFT_RESET` 后，局部规划状态通常回到 `WAITING_GOAL` 或 `IDLE`。
- 如果定位已经丢失，soft reset 只能停止导航，不能恢复定位；仍需要重新 init/relocalize。
- `FULL_RESTART` 当前只是预留外部管理入口，真正杀进程/重启 launch 的动作依赖外部命令配置。

### 调用示例

```bash
ros2 service call /restart_navigation pct_scan_navigation/srv/RestartNavigation "{mode: 0}"
```

## `/global_localization_node/load_map`

Open3D 定位节点的内部地图加载接口。通常由 `/switch_map` 间接调用。

### 类型

```text
pct_scan_navigation/srv/LoadLocalizationMap
```

### 请求

```text
string map_name
string pcd_path
bool use_localization_thresholds
float32 fitness_eval_threshold
float32 threshold_fitness
float32 threshold_fitness_init
```

字段说明：

| 字段 | 含义 | 取值/注意事项 |
|---|---|---|
| `map_name` | 地图名，写入 `/localization_status.map_name` | 建议和 `/current_map.map_name` 一致 |
| `pcd_path` | Open3D 定位使用的 PCD 文件路径 | 必须是本机可读的 `.pcd` 文件 |
| `use_localization_thresholds` | 是否随地图一起覆盖定位阈值 | `false` 时后面三个阈值会被忽略 |
| `fitness_eval_threshold` | Open3D `EvaluateRegistration` 的距离阈值 | 大于 0 才会覆盖当前参数；单位 m |
| `threshold_fitness` | tracking ICP 接受阈值 | 大于 0 才会覆盖当前参数 |
| `threshold_fitness_init` | init ICP 接受阈值 | 大于 0 才会覆盖当前参数 |

### 响应

```text
bool success
string message
```

### 行为

- 发布 `/localization_status` 为 `MAP_SWITCHING`。
- 从 `pcd_path` 读取 PCD，并按节点参数重新构建定位用下采样地图。
- 清空 scan queue，重置 `map→odom` 为单位矩阵。
- 设置 `loc_initialized=false`，定位状态回到 `UNINITIALIZED`。
- `use_localization_thresholds=true` 时，只应用请求中大于 0 的定位阈值。
- 加载失败时恢复旧阈值，并返回 `success=false`。

### 上位机/调试注意事项

- 正常业务切图不要直接调用该 service，应调用 `/switch_map`。
- 直接调用该 service 只会替换 Open3D 定位地图，不会替换 PCT tomogram，容易造成“定位地图”和“规划地图”不一致。
- 成功加载后定位状态是 `UNINITIALIZED`，需要重新给 `/initialpose` 或调用重定位服务。
- 如果 `pcd_path` 文件不存在、为空、读取失败，返回 `success=false`，旧地图不会被可靠认为已经切换成功。

### 调试调用示例

```bash
ros2 service call /global_localization_node/load_map pct_scan_navigation/srv/LoadLocalizationMap \
  "{map_name: outdoor, pcd_path: '/path/to/outdoor.pcd', use_localization_thresholds: false, fitness_eval_threshold: 0.0, threshold_fitness: 0.0, threshold_fitness_init: 0.0}"
```

## `/pct_global_planner/load_tomogram`

PCT 全局规划器的内部 tomogram reload 接口。通常由 `/switch_map` 间接调用。

### 类型

```text
pct_scan_navigation/srv/LoadTomogram
```

### 请求

```text
string map_name
string tomo_path
```

字段说明：

| 字段 | 含义 | 取值/注意事项 |
|---|---|---|
| `map_name` | 地图名，仅用于日志和 response | 建议和 `/current_map.map_name` 一致 |
| `tomo_path` | PCT 全局规划使用的 tomogram pickle 路径 | 必须是本机存在的 `.pickle` 文件 |

### 响应

```text
bool success
string message
```

### 行为

- 只先检查 `tomo_path` 非空且文件存在。
- 用临时 `TomogramPlanner` 加载新 pickle。
- 临时 planner 加载成功后才替换当前 planner；加载失败不会污染旧 planner。
- 成功后立即发布一次新的 `/tomogram`，并发布空 `/pct_path` 清除旧全局路径。
- 原有 `/tomogram` 1Hz 周期发布逻辑保持不变。

### 上位机/调试注意事项

- 正常业务切图不要直接调用该 service，应调用 `/switch_map`。
- 直接调用该 service 只会替换 PCT tomogram，不会替换 Open3D PCD 地图。
- reload 是近似原子的：新 planner 加载成功前不会替换旧 planner；失败后旧 planner 仍可继续规划。
- service 内只做 `tomo_path` 非空和文件存在检查，不提前构建 `/tomogram` 点云，也不做额外数据结构校验。
- 如果 pickle 文件存在但内容损坏，临时 planner 加载会失败，返回 `success=false`。

### 调试调用示例

```bash
ros2 service call /pct_global_planner/load_tomogram pct_scan_navigation/srv/LoadTomogram \
  "{map_name: outdoor, tomo_path: '/path/to/outdoor_global_voxel002_ground.pickle'}"
```

## `/scan_planner_node/reset_navigation`

SCAN 局部规划器内部 soft reset 接口。通常由 `/restart_navigation` 或 `/switch_map` 间接调用。

### 类型

```text
std_srvs/srv/Trigger
```

### 请求/响应

```text
---
bool success
string message
```

### 行为

- 清空 active waypoints 和 pending path。
- 清除当前目标、new target 标志、replan fail count、end yaw 等运行状态。
- 如果已有 odom，则调用 emergency stop。
- FSM 回到 `WAIT_TARGET`。
- 由 `scan_planner_node` 发布新的 `/navigation_status`。

### 上位机/调试注意事项

- 正常业务停止建议调用 `/restart_navigation mode=0`，让 `nav_manager_node` 同时处理零速度和空 waypoint。
- 直接调用该 service 不会重新加载地图，也不会恢复定位。
- 如果当前没有 odom，局部规划内部无法基于当前位置调用 emergency stop，但仍会清空导航状态。
- reset 后如果仍有外部节点继续发布目标或路径，导航可能马上再次被触发；上位机应同步停止发目标。

### 调试调用示例

```bash
ros2 service call /scan_planner_node/reset_navigation std_srvs/srv/Trigger "{}"
```

## `/localization_status`

Open3D 定位状态 topic，由 `global_localization_node` 周期发布。

### 类型

```text
pct_scan_navigation/msg/LocalizationStatus
```

### 字段

```text
std_msgs/Header header

uint8 UNINITIALIZED=0
uint8 INITIALIZING=1
uint8 INIT_SUCCESS=2
uint8 TRACKING=3
uint8 TRACKING_WARN=4
uint8 TRACKING_LOST=5
uint8 MAP_SWITCHING=6
uint8 state

string map_name
float32 fitness
string reason
```

字段说明：

| 字段 | 含义 | 上位机使用建议 |
|---|---|---|
| `header.stamp` | 状态发布时间 | 可用于判断状态是否超时 |
| `header.frame_id` | 固定为 `map` | 通常不用显示 |
| `state` | 定位状态枚举 | 上位机主判断字段 |
| `map_name` | 当前 Open3D 定位地图名 | 应和 `/current_map.map_name` 对齐 |
| `fitness` | 最近一次 ICP fitness | 只做辅助显示；不同地图/采样大小下阈值可能不同 |
| `reason` | 简短原因 | 可直接显示在诊断区域 |

状态枚举值：

| 数值 | 名称 | 含义 | 上位机建议 |
|---:|---|---|---|
| `0` | `UNINITIALIZED` | 未完成重定位，或切图后等待重新 init | 禁止发导航目标，提示用户初始化定位 |
| `1` | `INITIALIZING` | 正在执行 init ICP | 显示“正在重定位”，等待结果 |
| `2` | `INIT_SUCCESS` | init ICP 成功后的短暂状态 | 可以显示成功，但建议等后续 `TRACKING` 稳定后再允许导航 |
| `3` | `TRACKING` | tracking ICP 正常接受 | 定位可用，可以导航 |
| `4` | `TRACKING_WARN` | 最近一次 tracking 被拒绝，但未达到 lost 条件 | 提示定位波动，谨慎继续 |
| `5` | `TRACKING_LOST` | 连续 tracking 失败达到阈值 | 禁止导航；系统会 soft reset，需重新定位 |
| `6` | `MAP_SWITCHING` | Open3D 正在加载新地图 | 禁止发目标，等待切图结束 |

常见 `reason`：

| `reason` | 含义 | 建议处理 |
|---|---|---|
| `ok` | 正常 | 无需特殊处理 |
| `map_switching` | 地图切换中 | 等待 `/current_map=LOADED` |
| `map_loaded` | PCD 地图已加载，定位回到未初始化 | 触发/提示重新定位 |
| `map_load_failed` | PCD 地图加载失败 | 提示检查 PCD 路径和文件 |
| `fitness_low` | ICP fitness 低于接受阈值 | 提示定位质量差 |
| `delta_too_large` | 本次定位平移跳变过大 | 提示可能漂移或初值错误 |
| `yaw_delta_too_large` | 本次定位 yaw 跳变过大 | 提示朝向可能异常 |
| `invalid_cloud` | 点云无效或点数不足 | 检查雷达/点云输入 |
| `no_scan` | 暂无可用 scan | 检查点云 topic 和时间戳 |

`nav_manager_node` 会监听该 topic；第一次发现 `TRACKING_LOST` 时触发 soft reset。

上位机判断定位可用的推荐逻辑：

```text
localization_ok = (state == TRACKING)
```

如果要更保守，可以要求连续收到数帧 `TRACKING` 后再允许发目标。

### 监听示例

```bash
ros2 topic echo /localization_status
```

## `/navigation_status`

导航执行状态 topic。v1 中唯一可信发布源是 `scan_planner_node`，`nav_manager_node` 不再发布或转发
该 topic。

### 类型

```text
pct_scan_navigation/msg/NavigationStatus
```

### 字段

```text
std_msgs/Header header

uint8 IDLE=0
uint8 WAITING_GOAL=1
uint8 PLANNING_GLOBAL=2
uint8 GLOBAL_READY=3
uint8 PLANNING_LOCAL=4
uint8 NAVIGATING=5
uint8 AVOIDING=6
uint8 BLOCKED=7
uint8 GOAL_REACHED=8
uint8 CANCELED=9
uint8 FAILED=10
uint8 LOCALIZATION_LOST=11
uint8 MAP_SWITCHING=12
uint8 state

string map_name
bool goal_active
float32 distance_to_goal
uint32 remaining_waypoints
string reason
```

### 状态来源

- `WAIT_TARGET`：无目标时发布 `WAITING_GOAL`，有目标待处理时发布 `PLANNING_LOCAL`。
- `GEN_NEW_TRAJ`：发布 `PLANNING_LOCAL`。
- `REPLAN_TRAJ`：发布 `AVOIDING`。
- `EXEC_TRAJ` / `FINAL_YAW_ALIGN`：发布 `NAVIGATING`。
- `EMERGENCY_STOP`：发布 `BLOCKED`。
- 目标抵达、取消、地图切换、地图失败会分别发布 `GOAL_REACHED`、`CANCELED`、
  `MAP_SWITCHING`、`FAILED`。

字段说明：

| 字段 | 含义 | 上位机使用建议 |
|---|---|---|
| `header.stamp` | 状态发布时间 | 可用于判断导航状态是否超时 |
| `header.frame_id` | 当前实现通常为地图/规划坐标系 | 一般只显示或忽略 |
| `state` | 导航状态枚举 | 上位机主判断字段 |
| `map_name` | 当前地图名，来自 `/current_map` | 用于确认 UI 地图和导航地图一致 |
| `goal_active` | 局部规划是否持有目标 | 判断是否正在执行目标 |
| `distance_to_goal` | 当前 odom 到最终目标的 XY 距离，单位 m | 用于 UI 显示剩余距离；不是全局路径长度 |
| `remaining_waypoints` | 当前 active waypoints 数量 | 用于显示还有多少 waypoint 待处理 |
| `reason` | 简短原因 | 可直接显示 |

状态枚举值：

| 数值 | 名称 | 含义 | 上位机建议 |
|---:|---|---|---|
| `0` | `IDLE` | 空闲或初始化阶段 | 可等待目标；实际是否允许发目标还要看定位状态 |
| `1` | `WAITING_GOAL` | 等待目标点 | 定位 OK 时允许发目标 |
| `2` | `PLANNING_GLOBAL` | 全局规划中 | 预留状态；当前主要由 PCT/上层链路表达 |
| `3` | `GLOBAL_READY` | 全局路径已就绪 | 预留状态 |
| `4` | `PLANNING_LOCAL` | 局部轨迹生成中 | 显示“规划中” |
| `5` | `NAVIGATING` | 正在执行轨迹 | 显示“导航中” |
| `6` | `AVOIDING` | 局部重规划/避障中 | 显示“避障中”或“绕障中” |
| `7` | `BLOCKED` | emergency stop 或连续重规划失败 | 提示受阻，需要恢复或重新发目标 |
| `8` | `GOAL_REACHED` | 已到达目标 | 显示到达，可清除当前任务 |
| `9` | `CANCELED` | 目标被取消或 waypoint 被清空 | 显示已取消 |
| `10` | `FAILED` | 失败，例如地图加载失败导致导航不可用 | 提示错误详情 |
| `11` | `LOCALIZATION_LOST` | 定位丢失 | v1 不由 nav_manager 伪造该状态；主要看 `/localization_status` |
| `12` | `MAP_SWITCHING` | 地图切换中 | 禁止发目标 |

常见 `reason`：

| `reason` | 含义 |
|---|---|
| `startup` | 节点刚启动 |
| `ok` | 正常 |
| `map_switching` | 当前地图正在切换 |
| `map_loaded` | 地图加载完成 |
| `map_failed` | 地图加载失败 |
| `goal_reached` | 到达目标 |
| `canceled` | 导航被取消 |
| `soft_reset` | 局部规划被软复位 |
| `replan_failed` | 重规划连续失败，进入阻塞/急停 |

上位机判断是否可以发新目标的推荐逻辑：

```text
can_send_goal =
  current_map.state == LOADED &&
  localization_status.state == TRACKING &&
  navigation_status.state in [IDLE, WAITING_GOAL, GOAL_REACHED, CANCELED]
```

如果处于 `MAP_SWITCHING`、`BLOCKED`、`FAILED`、`LOCALIZATION_LOST`，不要直接发目标，先处理对应错误。

### 监听示例

```bash
ros2 topic echo /navigation_status
ros2 topic info /navigation_status
```

正常情况下 `ros2 topic info /navigation_status` 应只看到一个 publisher，即 `scan_planner_node`。

## `/current_map`

当前地图状态 topic，由 `nav_manager_node` 发布。QoS 使用 reliable + transient local，因此 RViz 或后启动
节点也能拿到最近一次地图状态。

### 类型

```text
pct_scan_navigation/msg/MapStatus
```

### 字段

```text
std_msgs/Header header

uint8 UNLOADED=0
uint8 LOADING=1
uint8 LOADED=2
uint8 FAILED=3
uint8 state

string map_name
string reason
```

### 监听关系

- `scan_planner_node` 监听 `/current_map`，用于更新 `/navigation_status.map_name`。
- 当 `/current_map.state=LOADING` 时，`scan_planner_node` 发布 `NavigationStatus.MAP_SWITCHING`。
- 当 `/current_map.state=FAILED` 时，`scan_planner_node` 发布 `NavigationStatus.FAILED`。

字段说明：

| 字段 | 含义 | 上位机使用建议 |
|---|---|---|
| `header.stamp` | 状态发布时间 | 因为是 transient local，后启动收到的可能是旧时间戳，但仍是最近一次地图状态 |
| `header.frame_id` | 固定为 `map` | 通常不用处理 |
| `state` | 地图状态枚举 | 上位机主判断字段 |
| `map_name` | 当前或正在切换的地图名 | UI 显示当前地图 |
| `reason` | 简短原因 | 可直接显示 |

状态枚举值：

| 数值 | 名称 | 含义 | 上位机建议 |
|---:|---|---|---|
| `0` | `UNLOADED` | 尚未加载地图或启动时未配置初始地图 | 禁止导航，提示选择/加载地图 |
| `1` | `LOADING` | 正在切换地图 | 禁止发目标，等待 service 返回 |
| `2` | `LOADED` | 当前地图已加载完成 | 仍需等待定位 `TRACKING` 后才能导航 |
| `3` | `FAILED` | 地图切换或加载失败 | 提示错误，不要导航 |

常见 `reason`：

| `reason` | 含义 |
|---|---|
| `startup` | 节点启动时发布的初始地图状态 |
| `switching` | `/switch_map` 正在执行 |
| `ok` | 地图切换成功 |
| `localization map load failed: ...` | PCD 地图加载失败 |
| `tomogram load failed: ...` | PCT pickle 加载失败 |

### 监听示例

```bash
ros2 topic echo /current_map
```

## `/open3d_loc/relocalize`

使用给定的 `base_link` 在 `map` 下的初始位姿触发 Open3D 重定位。

### 类型

```text
open3d_loc/srv/Relocalize
```

### 请求

```text
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
```

字段说明：

| 字段 | 含义 | 单位/要求 |
|---|---|---|
| `x/y/z` | 初始估计的 `base_link` 在 `map` 下位置 | m，必须是有限数值 |
| `qx/qy/qz/qw` | 初始估计朝向四元数 | 会自动归一化；模长过小会拒绝 |

### 响应

```text
bool success
string message
```

| 字段 | 含义 |
|---|---|
| `success=true` | 已发布初始位姿，并等待到了新的 `/Odometry_open3d` |
| `success=false` | 输入无效、已有重定位请求正在运行，或等待定位结果超时 |
| `message` | 简短原因 |

### 行为

- 校验位置和四元数，并归一化四元数。
- 向 `/initialpose` 发布 `geometry_msgs/PoseWithCovarianceStamped`。
- 清除服务节点缓存的旧定位结果，等待请求之后的新 `/Odometry_open3d`。
- 新 `/Odometry_open3d` 到达时返回 `success=true`；默认 10 秒内没有结果则返回
  `success=false`。等待上限由节点参数 `relocalize_timeout_sec` 配置，不属于请求字段。
- 同一时间只允许一个重定位请求；并发请求返回失败。

### 上位机注意事项

- 该 service 是“发布初始位姿并等待新定位结果”的封装，不是直接设置最终定位。
- `success=true` 表示收到了一帧新的 Open3D 定位结果，不代表后续 tracking 永远稳定。
- 调用成功后仍建议监听 `/localization_status`，等 `state=TRACKING` 后再允许发目标。
- 初始位姿误差过大时，Open3D 可能 init 失败或后续进入 `TRACKING_WARN/TRACKING_LOST`。

### 调用示例

```bash
ros2 service call /open3d_loc/relocalize open3d_loc/srv/Relocalize \
  "{x: 1.0, y: 2.0, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

## `/open3d_loc/get_pose`

获取最新的 `base_link` 在 `map` 下的位姿。数据来自 `/Odometry_open3d`。

### 类型

```text
open3d_loc/srv/GetPose
```

### 请求

请求为空。

### 响应

```text
bool success
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
string message
```

尚未收到有效 `/Odometry_open3d` 或正在通过 Service 重定位时，返回
`success=false`，不会返回重定位前缓存的旧位姿。

字段说明：

| 字段 | 含义 |
|---|---|
| `success=true` | 返回的是最新有效 `/Odometry_open3d` 位姿 |
| `x/y/z` | 当前 `base_link` 在 `map` 下位置，单位 m |
| `qx/qy/qz/qw` | 当前 `base_link` 在 `map` 下朝向 |
| `message` | 成功或失败原因 |

上位机注意事项：

- 如果要连续显示机器人位置，更推荐直接监听 `/Odometry_open3d`；该 service 更适合按需查询。
- `get_pose` 不会触发 ICP 或重新定位，只返回服务节点缓存的最新结果。

### 调用示例

```bash
ros2 service call /open3d_loc/get_pose open3d_loc/srv/GetPose "{}"
```

## `/open3d_loc/publish_goal`

把给定导航点封装为 `geometry_msgs/PoseStamped` 并发布到 `/goal_pose`。

### 类型

```text
open3d_loc/srv/PublishGoal
```

### 请求

```text
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
```

字段说明：

| 字段 | 含义 | 单位/要求 |
|---|---|---|
| `x/y/z` | 导航目标点位置 | m，`map` 坐标系 |
| `qx/qy/qz/qw` | 目标点朝向 | 四元数，自动校验和归一化 |

### 响应

```text
bool success
string message
```

发布消息的 `header.frame_id` 固定为 `map`，时间戳使用节点当前 ROS 时间。
`success=true` 只表示导航点已经交给 ROS publisher，不表示规划成功、机器人到达或导航完成。

Mode 1 中 `/goal_pose` 直接由 SCAN-Planner 接收；Mode 2 中由 PCT 全局规划器接收并生成
`/pct_path`。

### 上位机注意事项

- 发目标前建议同时满足：
  - `/current_map.state=LOADED`；
  - `/localization_status.state=TRACKING`；
  - `/navigation_status.state` 为 `IDLE`、`WAITING_GOAL`、`GOAL_REACHED` 或 `CANCELED`。
- `success=true` 只代表目标点发布成功，不代表 PCT 全局规划成功，也不代表 SCAN 局部规划成功。
- 目标点 frame 固定为 `map`；如果上位机 UI 使用其他坐标系，需要先自行转换。
- Mode 2 下建议继续监听 `/navigation_status` 和 `/pct_path`，判断全局路径是否生成。

### 调用示例

```bash
ros2 service call /open3d_loc/publish_goal open3d_loc/srv/PublishGoal \
  "{x: 3.0, y: 1.0, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

## `/open3d_loc/pose_deviation`

计算当前定位相对给定参考位姿的平面位置和航向误差，不会触发重定位。

### 类型

```text
open3d_loc/srv/PoseDeviation
```

### 请求

```text
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
```

请求表示参考 `base_link` 位姿，坐标系为 `map`。

### 响应

```text
bool success
geometry_msgs/Pose current_pose
float64 error_x
float64 error_y
float64 distance_xy
float64 yaw_error_rad
float64 yaw_error_deg
string message
```

- `current_pose`：当前 `/Odometry_open3d` 位姿。
- `error_x/error_y`：在参考位姿自身航向坐标系下的平面误差。
- `distance_xy`：平面欧氏距离。
- `yaw_error_rad/yaw_error_deg`：当前航向减参考航向，并归一化到 `[-pi, pi]`。
- 没有有效当前定位时返回 `success=false`。

字段说明：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功拿到当前定位并完成误差计算 |
| `current_pose` | 当前定位位姿 |
| `error_x` | 当前位姿在参考位姿前向方向上的误差，单位 m |
| `error_y` | 当前位姿在参考位姿左/右方向上的横向误差，单位 m |
| `distance_xy` | 平面距离误差，单位 m |
| `yaw_error_rad` | 航向误差，单位 rad |
| `yaw_error_deg` | 航向误差，单位 deg |
| `message` | 成功或失败原因 |

上位机注意事项：

- 该 service 适合做“是否偏离预期停靠点/检查点”的一次性判断。
- 它不会触发导航、不会触发重定位，也不会改变当前定位状态。

### 调用示例

```bash
ros2 service call /open3d_loc/pose_deviation open3d_loc/srv/PoseDeviation \
  "{x: 1.0, y: 2.0, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```


## `/map_save`

Fast-LIO 建图保存接口。该 service 来自 `fastlio_mapping`，用于把当前建图点云保存到配置指定路径。

### 类型

```text
std_srvs/srv/Trigger
```

### 请求/响应

```text
---
bool success
string message
```

### 行为

- 请求为空。
- `success=true` 表示保存命令执行成功。
- `success=false` 表示当前配置未启用保存，或保存过程失败。

### 上位机注意事项

- 该接口只和建图保存有关，不会切换当前导航地图。
- 保存后的 PCD 是否被 Open3D 定位使用，仍取决于配置里的 `path_map` 或 `map_profiles.yaml`。
- 常见失败 `Map save disabled.` 表示 Fast-LIO 配置未开启保存。

### 调用示例

```bash
ros2 service call /map_save std_srvs/srv/Trigger "{}"
```

## `/go2_cmd_vel_bridge/enable`

启用或禁用 Unitree Go2/Go2-W 速度命令桥。桥节点启动后默认处于禁用状态。

### 类型与字段

```text
std_srvs/srv/SetBool
bool data
---
bool success
string message
```

字段说明：

| 字段 | 含义 |
|---|---|
| `data=true` | 请求启用速度桥 |
| `data=false` | 请求禁用速度桥 |
| `success` | 请求是否执行成功 |
| `message` | 成功或失败原因 |

### 启用

```bash
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool "{data: true}"
```

启用前必须同时满足：

- 已收到且未超时的 `/Odometry_open3d`；
- 已收到且未超时的 Unitree SportModeState；
- `SportClient::StopMove` 调用成功。

启用成功后桥仍会等待一条新的 `/cmd_vel`，不会立即沿用启用前的旧速度命令。

上位机注意事项：

- 启用速度桥前，建议确认定位和导航状态均正常，避免机器人接收旧任务。
- 使能成功不代表机器人正在移动，只代表 `/cmd_vel` 可以被桥转发到底盘。
- 如果收到定位心跳或 Unitree 状态心跳超时，启用会失败。

### 禁用

```bash
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool "{data: false}"
```

禁用会清空目标速度并请求 `StopMove`。当前使能状态也会发布在
`/go2_cmd_vel_bridge/armed`。

上位机可以监听 `/go2_cmd_vel_bridge/armed` 显示底盘速度桥是否处于使能状态。

## 常见失败说明

| `message` 示例 | 含义 |
|---|---|
| `quaternion norm is too small` | 输入四元数无效 |
| `position contains non-finite value` | XYZ 包含 NaN 或无穷值 |
| `another relocalize request is running` | 已有重定位调用正在等待结果 |
| `timeout waiting for relocalization result on /Odometry_open3d` | 等待上限内没有新的有效定位结果 |
| `no current base_link pose from /Odometry_open3d` | 尚无可返回的定位结果 |
| `cannot enable: odometry heartbeat is missing or stale` | 底盘桥缺少新鲜定位心跳 |
| `cannot enable: sport state heartbeat is missing or stale` | 底盘桥缺少新鲜 Unitree 状态 |
| `Map save disabled.` | Fast-LIO 配置未启用 PCD 保存 |
| `unknown map_name: ...` | `/switch_map` 请求的地图名不在 `map_profiles.yaml` 中 |
| `map profile ... requires pcd_path and tomo_path` | 地图 profile 缺少必填 PCD 或 pickle 路径 |
| `localization map load failed: ...` | Open3D PCD 地图加载失败 |
| `tomogram load failed: ...` | PCT tomogram pickle 加载失败 |
| `tomo_path does not exist: ...` | `/pct_global_planner/load_tomogram` 请求的 pickle 文件不存在 |
| `full_restart_command is empty` | 请求 FULL_RESTART，但未配置外部重启命令 |
