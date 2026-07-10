# ROS1 通信整理

本文档按当前代码整理项目内 ROS1 节点、话题、消息类型、数据格式和主要数据内容。默认配置下，`Messager` 处理后的对外发送仍走串口；ROS 主要用于视觉进程、通信状态、无人机定位和雷达 UDP 桥之间的数据交换。

## 主流程

```text
detect.Detector
  -> /vision/detect
26_main.py
  -> /vision/result
  -> /messager/state
communication.Receiver
  -> /receiver/state
Counter/init_angle_sender.py
  -> /drone_field_xyz
  -> /init_yawpitch
Radio/field_info_publisher.py
  -> /radar/enemy/*
communication.Messager
  <- /messager/state
  <- /receiver/state
  <- /drone_field_xyz
  <- /radar/enemy/health_array
  <- /radar/enemy/jam_key
  -> serial /dev/ttyUSB0
```

## 节点

| 启动文件 | ROS 节点名 | 作用 |
| --- | --- | --- |
| `26_main.py` | `radar_vision_main` | 视觉主处理：订阅检测结果，做坐标解算、车辆状态维护、发布给 `Messager` 的状态和给 `Detector` 的绘图结果。 |
| `detect/Detector.py` | `vision_detector` | YOLO 检测/分类，发布轻量检测结果，订阅绘图指令。 |
| `communication/Receiver.py` | `radar_receiver` | 读取下位机/裁判串口帧，解析状态后发布 ROS 状态。 |
| `communication/Messager.py` | `radar_messager` | 订阅视觉、Receiver、无人机、雷达 UDP 桥状态，执行决策与频率控制，默认通过串口发送给下位机。 |
| `Counter/init_angle_sender.py` | `lidar_tracker` | 订阅激光雷达点云，追踪无人机，发布云台 yaw/pitch 和无人机场地坐标。 |
| `Radio/field_info_publisher.py` | `radar_ros_publisher` | UDP 裁判/雷达数据桥，发布敌方血量、密钥、位置、弹药、Buff 等 ROS 话题。 |
| `livox_ros_driver` | 由 launch 决定 | 发布 Livox 原始点云，供无人机追踪节点订阅。 |
| `detect/Capture.py` | `image_processor_node` | 独立测试/示例节点，订阅 `/camera/image`，不属于当前主启动链路。 |

节点均使用 `anonymous=True` 的地方，运行时实际节点名会追加随机后缀；表中是代码里的基础节点名。

## 话题总览

| Topic | 类型 | 发布者 | 订阅者 | 说明 |
| --- | --- | --- | --- | --- |
| `/radar/main_ready` | `std_msgs/String` | `26_main.py` | `detect/Detector.py` | 主程序完成相机/场地初始化后的 ready 信号，latched。 |
| `/vision/detect` | `std_msgs/String` JSON | `detect/Detector.py` | `26_main.py` | 每帧检测框、track id、分类 label。 |
| `/vision/result` | `std_msgs/String` JSON | `26_main.py` | `detect/Detector.py` | 绘图指令，Detector 按 seq 对齐后显示。 |
| `/messager/state` | `std_msgs/String` JSON | `26_main.py` | `communication/Messager.py` | 视觉解算后的敌我车辆状态和哨兵预警信息。 |
| `/receiver/state` | `std_msgs/String` JSON | `communication/Receiver.py` | `communication/Messager.py` | 串口接收端解析出的比赛/血量/标记/双倍/飞镖/干扰状态。 |
| `/drone_field_xyz` | `sensor_msgs/PointCloud2` | `Counter/init_angle_sender.py` | `communication/Messager.py` | 单点点云，表示无人机在赛场坐标系的位置。 |
| `/init_yawpitch` | `std_msgs/Float32MultiArray` | `Counter/init_angle_sender.py` | 下位机/外部节点 | 云台初始 yaw、pitch。 |
| `/radar/enemy/health_array` | `std_msgs/Float32MultiArray` | `Radio/field_info_publisher.py` | `communication/Messager.py` | 敌方血量数组。 |
| `/radar/enemy/jam_key` | `std_msgs/String` | `Radio/field_info_publisher.py` | `communication/Messager.py` | 干扰波密钥字符串。 |
| `/radar/enemy/positions_array` | `std_msgs/Float32MultiArray` | `Radio/field_info_publisher.py` | 可视化/调试 | 敌方位置数组。 |
| `/radar/enemy/positions_markers` | `visualization_msgs/MarkerArray` | `Radio/field_info_publisher.py` | RViz | 敌方位置可视化。 |
| `/radar/enemy/health_json` | `std_msgs/String` JSON | `Radio/field_info_publisher.py` | 调试/记录 | 敌方血量 JSON。 |
| `/radar/enemy/ammo_array` | `std_msgs/Float32MultiArray` | `Radio/field_info_publisher.py` | 调试/记录 | 敌方弹药数组。 |
| `/radar/enemy/team_status` | `std_msgs/String` JSON | `Radio/field_info_publisher.py` | 调试/记录 | 敌方队伍状态。 |
| `/radar/enemy/module_status` | `std_msgs/String` JSON | `Radio/field_info_publisher.py` | 调试/记录 | 模块状态。 |
| `/radar/enemy/buffs` | `std_msgs/String` JSON | `Radio/field_info_publisher.py` | 调试/记录 | Buff 信息。 |
| `/radar/enemy/all_info` | `std_msgs/String` JSON | `Radio/field_info_publisher.py` | 调试/记录 | 预留综合信息发布者，当前文件中创建了 publisher。 |
| `/radar/raw/frame_hex` | `std_msgs/String` | `Radio/field_info_publisher.py` | 调试/记录 | 预留原始帧十六进制发布者，当前文件中创建了 publisher。 |
| `/messge/position` | `std_msgs/String` JSON | `communication/Messager.py` | 外部 ROS 串口桥 | 可选输出。默认 `send_transport: serial` 时不发布，`Messager` 直接串口发送。topic 拼写保留为 `/messge/position`。 |
| `/camera/image` | `sensor_msgs/Image` | 外部相机节点 | `detect/Capture.py` 测试类 | 只在 `ImageProcessor_test` 中使用，不属于主链路。 |

## 主要 Topic 数据格式

### `/radar/main_ready`

类型：`std_msgs/String`

```text
data: "ready"
```

作用：`detect.Detector` 启动时等待该消息，避免检测端早于主程序初始化。

### `/vision/detect`

类型：`std_msgs/String`，`data` 是 JSON。

发布者：`detect/Detector.py`

订阅者：`26_main.py`

```json
{
  "seq": 1,
  "stamp": 1720000000.123,
  "frame_id": "vision_camera",
  "source_width": 4024,
  "source_height": 3036,
  "detections": [
    {
      "xyxy": [100.0, 200.0, 300.0, 420.0],
      "xywh": [200.0, 310.0, 200.0, 220.0],
      "track_id": 12,
      "label": "R1",
      "stamp": 1720000000.123
    }
  ]
}
```

字段说明：

| 字段 | 内容 |
| --- | --- |
| `seq` | Detector 发布序号。 |
| `stamp` | 检测帧时间戳。 |
| `frame_id` | 配置里的相机帧名，默认 `vision_camera`。 |
| `source_width/source_height` | 原始图像尺寸。 |
| `detections[].xyxy` | 检测框左上/右下角。 |
| `detections[].xywh` | 检测框中心点和宽高。 |
| `detections[].track_id` | 跟踪 ID。 |
| `detections[].label` | 分类标签，如 `R1`、`B7`、`NULL`。 |
| `detections[].stamp` | 单个检测结果时间戳。 |

### `/vision/result`

类型：`std_msgs/String`，`data` 是 JSON。

发布者：`26_main.py`

订阅者：`detect/Detector.py`

```json
{
  "seq": 1,
  "detect_stamp": 1720000000.123,
  "stamp": 1720000000.130,
  "texts": [
    {
      "text": "fps: 50.00",
      "point": [10, 500],
      "scale": 0.75,
      "color": [0, 255, 122],
      "thickness": 2
    }
  ],
  "circles": [
    {
      "center": [1000, 800],
      "radius": 5,
      "color": [0, 0, 255],
      "thickness": -1
    }
  ],
  "lines": [
    {
      "p1": [1000, 800],
      "p2": [1200, 820],
      "color": [0, 255, 122],
      "thickness": 2
    }
  ]
}
```

作用：主程序把重投影点、哨兵预警文字、调试线段等返回给 Detector，由 Detector 在对应缓存帧上绘制。

### `/messager/state`

类型：`std_msgs/String`，`data` 是 JSON。

发布者：`26_main.py`

订阅者：`communication/Messager.py`

```json
{
  "seq": 1,
  "stamp": 1720000000.200,
  "vision_seq": 18,
  "vision_stamp": 1720000000.123,
  "enemy_car_infos": [],
  "our_car_infos": [],
  "sentinel_alert_info": [101, 3.25, 2]
}
```

`enemy_car_infos` 和 `our_car_infos` 使用项目内 `CarList` 的列表结构，当前 `Messager` 按以下索引读取：

| 索引 | 内容 |
| --- | --- |
| `0` | `track_id` |
| `1` | `car_id` |
| `2` | 图像中心点或相关图像坐标 |
| `3` | 相机坐标 |
| `4` | 赛场坐标 `field_xyz` |
| `5` | 颜色 |
| `6` | `is_valid` |

`sentinel_alert_info` 格式：

```text
[car_id, distance, quadrant]
```

### `/receiver/state`

类型：`std_msgs/String`，`data` 是 JSON。

发布者：`communication/Receiver.py`

订阅者：`communication/Messager.py`

```json
{
  "seq": 1,
  "stamp": 1720000000.300,
  "type": "robot_status",
  "state": {
    "is_activating_double_effect": false,
    "my_health": [100, 100, 100, 100, 100, 0, 1500, 5000],
    "mark_progress": [0, 0, 0, 0, 0, 0],
    "have_double_effect_times": 0,
    "time_left": -1,
    "dart_target": 0,
    "interference_level": 1,
    "is_key_update": 0
  },
  "payload": {
    "my_health": [100, 100, 100, 100, 100, 0, 1500, 5000]
  }
}
```

`type` 取值由解析到的串口命令决定：

| `type` | 触发内容 |
| --- | --- |
| `initial` | Receiver 启动后发布的初始状态，publisher 使用 latch。 |
| `game_status` | 比赛阶段和剩余时间。 |
| `robot_status` | 己方 8 个血量值。 |
| `mark_process` | 对方 6 个单位标记状态。 |
| `double_effect` | 双倍易伤机会和激活状态。 |
| `interference_status` | 干扰等级和密钥更新标志。 |
| `dart_target` | 飞镖当前选定目标。 |

`state` 字段说明：

| 字段 | 内容 |
| --- | --- |
| `is_activating_double_effect` | 是否正在激活双倍易伤。 |
| `my_health` | `[hero, engineer, infantry_3, infantry_4, reserved, aerial_or_reserved, sentry_or_outpost, base]`，代码注释为己方 1-4、7 号前哨站和基地等 8 个值。 |
| `mark_progress` | 对方 `[hero, engineer, infantry_3, infantry_4, drone, sentry]` 标记状态，0/1。 |
| `have_double_effect_times` | 当前拥有的双倍易伤机会次数。 |
| `time_left` | 比赛剩余时间，单位秒；初始值 `-1`。 |
| `dart_target` | 飞镖目标，0/1/2 等。 |
| `interference_level` | 干扰等级。 |
| `is_key_update` | 是否更新密钥标志。 |

### `/drone_field_xyz`

类型：`sensor_msgs/PointCloud2`

发布者：`Counter/init_angle_sender.py`

订阅者：`communication/Messager.py`

内容是宽度为 1 的单点点云：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `x` | `FLOAT32` | 无人机赛场坐标 x。 |
| `y` | `FLOAT32` | 无人机赛场坐标 y。 |
| `z` | `FLOAT32` | 无人机赛场坐标 z。 |
| `timestamp` | `FLOAT32` | 发布时间戳。 |

`header.frame_id` 为 `world`。`Messager` 取第一个点作为无人机坐标；当我方颜色为 `Blue` 时，代码会做 `(28 - x, 15 - y)` 对称变换。

### `/init_yawpitch`

类型：`std_msgs/Float32MultiArray`

发布者：`Counter/init_angle_sender.py`

```text
data: [yaw_deg, pitch_deg]
```

追踪到目标时发布计算得到的云台角度；未锁定目标时发布 `[0.0, 0.0]`。打表模式下按配置扫描 yaw/pitch 并发布同样格式。

## `Radio/field_info_publisher.py` 发布的话题

### `/radar/enemy/positions_array`

类型：`std_msgs/Float32MultiArray`

```text
data: [
  hero_x, hero_y,
  engineer_x, engineer_y,
  infantry_3_x, infantry_3_y,
  infantry_4_x, infantry_4_y,
  aerial_x, aerial_y,
  sentry_x, sentry_y
]
layout.dim[0].label = "robot_positions"
layout.dim[0].size = 6
layout.dim[0].stride = 2
```

坐标来自 UDP 数据，单位按上游字典值保留；Marker 可视化时会除以 1000 转成米。

### `/radar/enemy/positions_markers`

类型：`visualization_msgs/MarkerArray`

每个敌方单位发布一个圆柱 marker 和一个文字 marker，用于 RViz 可视化。`frame_id` 为 `map`。

### `/radar/enemy/health_array`

类型：`std_msgs/Float32MultiArray`

订阅者：`communication/Messager.py`

```text
data: [
  hero,
  engineer,
  infantry_3,
  infantry_4,
  reserved,
  sentry
]
layout.dim[0].label = "robot_healths"
layout.dim[0].size = 6
layout.dim[0].stride = 1
```

`Messager` 内部转换为 5 个敌方血量：

```text
[hero, engineer, infantry_3, infantry_4, sentry]
```

### `/radar/enemy/health_json`

类型：`std_msgs/String`，`data` 是 JSON。

```json
{
  "seq": 1,
  "timestamp": 1720000000.0,
  "health": {
    "hero": 100,
    "engineer": 100,
    "infantry_3": 100,
    "infantry_4": 100,
    "sentry": 600
  }
}
```

### `/radar/enemy/ammo_array`

类型：`std_msgs/Float32MultiArray`

```text
data: [hero, infantry_3, infantry_4, aerial, sentry]
layout.dim[0].label = "robot_ammos"
layout.dim[0].size = 5
layout.dim[0].stride = 1
```

### `/radar/enemy/team_status`

类型：`std_msgs/String`，`data` 是 JSON。

主要字段：

```json
{
  "seq": 1,
  "timestamp": 1720000000.0,
  "remaining_coins": 0,
  "destroy_count": 0,
  "module_status": {
    "base_shield": 0,
    "outpost_shield": 0,
    "hero_shield": 0,
    "engineer_shield": 0,
    "infantry_3_shield": 0,
    "infantry_4_shield": 0,
    "sentry_shield": 0,
    "base_occupied": 0,
    "outpost_occupied": 0,
    "power_rune": 0,
    "flyover_buff": 0,
    "flyover_cooldown": 0,
    "center_buff": 0,
    "resource_island_buff": 0,
    "power_rune_point": 0,
    "trapezoid_highland": 0,
    "ring_highland": 0
  }
}
```

### `/radar/enemy/module_status`

类型：`std_msgs/String`，`data` 是 JSON。

内容等于 `/radar/enemy/team_status` 中的 `module_status` 子对象。

### `/radar/enemy/buffs`

类型：`std_msgs/String`，`data` 是 JSON。

```json
{
  "seq": 1,
  "timestamp": 1720000000.0,
  "buffs": {
    "hero": {
      "hp_percent": 0,
      "cooling_percent": 0,
      "defense_percent": 0,
      "attack_percent": 0,
      "hp_value": 0
    },
    "engineer": {},
    "infantry_3": {},
    "infantry_4": {},
    "sentry": {},
    "aerial": {}
  }
}
```

每个单位的对象字段相同：`hp_percent`、`cooling_percent`、`defense_percent`、`attack_percent`、`hp_value`。

### `/radar/enemy/jam_key`

类型：`std_msgs/String`

订阅者：`communication/Messager.py`

```text
data: "123456"
```

`Messager` 收到后会反转字符串再用于发送，例如收到 `123456`，内部使用 `654321`。

### `/radar/enemy/all_info` 和 `/radar/raw/frame_hex`

类型：`std_msgs/String`

当前 `Radio/field_info_publisher.py` 中创建了 publisher。需要确认 `radar_udp_receiver.py` 的具体回调路径后，才能认为运行时一定有数据发布；主链路当前不依赖这两个 topic。

## `Messager` 可选 ROS 输出 `/messge/position`

当前 `configs/main_config.yaml` 默认：

```yaml
communication:
  send_transport: 'serial'
```

因此 `communication/Messager.py` 默认不通过 `/messge/position` 输出，而是保持串口发送：

```text
Messager -> Sender.send_info(tx_buff) -> /dev/ttyUSB0
```

如果配置为：

```yaml
communication:
  send_transport: 'ros1'
```

则 `Messager` 会把发送内容发布为 `std_msgs/String` JSON：

```json
{
  "seq": 1,
  "stamp": 1720000000.0,
  "type": "map",
  "rate_hz": 4.8,
  "payload": {},
  "frame_hex": "a5..."
}
```

常见 `type`：

| `type` | 内容 |
| --- | --- |
| `map` | 小地图位置，`payload.positions` 为 12 个 `[x, y]`。 |
| `sentry_perception` | 哨兵全局感知的敌方 6 车位置。 |
| `enemy_hp` | 敌方 5 个血量。 |
| `double_effect_decision` | 双倍易伤决策次数和密钥/分析结果。 |
| `sentinel_alert` | 哨兵预警车辆 ID、距离、象限。 |
| `hero_alert` | 英雄预警开关。 |
| `hero_assist` | 英雄辅助 pitch/yaw。 |
| `secure_our_hero` | 我方英雄保护预警。 |
| `double_effect_times_to_car` | 给己方车辆同步双倍易伤次数。 |

## 外部输入 Topic

### Livox 点云

`Counter/init_angle_sender.py` 默认订阅命令行参数里的 `lidar_topic`，主启动脚本通过 `livox_ros_driver` 提供点云。README 中记录常用为：

```text
/livox/lidar
```

类型：

```text
sensor_msgs/PointCloud2
```

该点云用于无人机追踪；追踪结果再发布到 `/drone_field_xyz` 和 `/init_yawpitch`。

### `/camera/image`

`detect/Capture.py` 里的 `ImageProcessor_test` 订阅：

```text
/camera/image
```

类型：

```text
sensor_msgs/Image
```

这是独立测试入口，不是 `26_main.py + detect.Detector.py` 当前主视觉链路。

## 配置入口

| 配置文件 | 字段 | 默认值 | 影响 |
| --- | --- | --- | --- |
| `configs/detector_config.yaml` | `ros.detect_topic` | `/vision/detect` | Detector 发布检测结果。 |
| `configs/detector_config.yaml` | `ros.result_topic` | `/vision/result` | Detector 订阅绘图结果。 |
| `configs/detector_config.yaml` | `ros.process_hz` | `100` | Detector/主循环处理频率相关。 |
| `configs/detector_config.yaml` | `ros.detect_publish_hz` | `100` | Detector 发布检测结果频率。 |
| `configs/main_config.yaml` | `messager.state_topic` | `/messager/state` | `26_main.py` 到 `Messager` 的视觉状态 topic。 |
| `configs/main_config.yaml` | `communication.receiver_state_topic` | `/receiver/state` | `Receiver` 到 `Messager` 的接收状态 topic。 |
| `configs/main_config.yaml` | `communication.position_topic` | `/messge/position` | `Messager` 可选 ROS 输出 topic。 |
| `configs/main_config.yaml` | `communication.send_transport` | `serial` | `serial` 为默认串口发送；`ros1` 才发布 `/messge/position`。 |
| `configs/main_config.yaml` | `messager.*_hz` | 见配置 | `Messager` 内部各类发送频率控制。 |

## 启动脚本中的主节点顺序

`launch_radar_system.sh` 当前顺序：

```text
roscore
roslaunch livox_ros_driver livox_lidar.launch
python3 Counter/init_angle_sender.py
python3 -m communication.Receiver
python3 -m communication.Messager
python3 -m detect.Detector
python3 26_main.py
```

其中 `communication.Receiver` 和 `communication.Messager` 之间的数据已经通过 `/receiver/state` 解耦；`Messager` 的最终对下位机发送仍按配置默认走串口。
