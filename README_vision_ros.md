# Vision ROS Split Notes

本文档记录本次对视觉检测链路的修改。

## 目标

原来的 `Detector` 在 `26_main.py` 内部用后台线程运行，主循环通过共享变量读取最新检测结果。这样主循环不会等待 YOLO 推理，所以主循环可以跑到较高频率。

本次修改后的目标是：

- 不再在业务逻辑里使用 Python 多线程跑检测。
- 不通过 ROS 传输图像，避免大图序列化和拷贝。
- 检测和主逻辑用两个进程并行运行。
- ROS 只传轻量 JSON 数据：
  - `/vision/detect`: 检测结果。
  - `/vision/result`: 主程序计算后的绘图指令。
- `/vision/result` 使用 `/vision/detect` 原始的 `seq` 和 `stamp` 回传，`Detector` 只绘制匹配同一帧的结果。
- 图像读取、YOLO 推理、绘图、显示全部放在 `detect/Detector.py`。

## 当前数据流

```text
detect/Detector.py 进程
  Capture/Video -> YOLO detect/classify -> /vision/detect
       ^                                      |
       |                                      v
  本地图像绘制 <- /vision/result <- 26_main.py 主进程

26_main.py 主进程
  /vision/detect -> 坐标解算 -> CarList -> /messager/state
                 -> 生成绘图指令 -> /vision/result

communication/Messager.py 进程
  /messager/state -> 决策/打包 -> /messge/position
```

图像不经过 ROS。`Detector` 保留自己读到的当前帧，并在本地根据 `/vision/result` 进行 `cv2.putText/circle/line` 绘制和 `imshow` 显示。

## 修改的文件

### `detect/Detector.py`

主要变化：

- 移除了检测线程、结果锁、保存视频线程队列等手写线程逻辑。
- 增加独立 ROS 节点入口：

```bash
python3 -m detect.Detector
```

- 发布 `/vision/detect`，消息类型为 `std_msgs/String`，内容是 JSON。
- 订阅 `/vision/result`，消息类型为 `std_msgs/String`，内容是 JSON 绘图指令。
- 在本文件内完成图像绘制和显示。

### `26_main.py`

主要变化：

- 不再 import 或实例化 `Detector`。
- 标定完成后释放相机，然后启动 detector 子进程：

```text
python -m detect.Detector ...
```

- 订阅 `/vision/detect` 获取检测框。
- 完成坐标解算、CarList 更新。
- 发布 `/messager/state`，把敌方车辆和我方车辆结果交给 Messager 进程。
- 将最终绘图信息发布到 `/vision/result`。

### `configs/detector_config.yaml`

新增/调整 ROS 配置：

```yaml
ros:
  detect_topic: "/vision/detect"
  result_topic: "/vision/result"
  frame_id: "vision_camera"
  process_hz: 100
  detect_publish_hz: 100
  alignment_cache_size: 8
  timestamp_tolerance_sec: 0.002
  display_enabled: True
  display_width: 1920
  display_height: 1080
```

## Topic 格式

### `/vision/detect`

方向：`Detector -> 26_main.py`

类型：`std_msgs/String`

JSON 示例：

```json
{
  "seq": 12,
  "stamp": 1720000000.123,
  "frame_id": "vision_camera",
  "source_width": 4024,
  "source_height": 3036,
  "detections": [
    {
      "xyxy": [100, 120, 260, 340],
      "xywh": [180, 230, 160, 220],
      "track_id": 3,
      "label": "B3",
      "stamp": 1720000000.120
    }
  ]
}
```

### `/vision/result`

方向：`26_main.py -> Detector`

类型：`std_msgs/String`

JSON 示例：

```json
{
  "seq": 12,
  "detect_stamp": 1720000000.123,
  "stamp": 1720000000.130,
  "texts": [
    {
      "text": "distance: 4.20",
      "point": [100, 120],
      "scale": 1.5,
      "color": [0, 255, 122],
      "thickness": 2
    }
  ],
  "circles": [
    {
      "center": [500, 600],
      "radius": 5,
      "color": [0, 0, 255],
      "thickness": -1
    }
  ],
  "lines": [
    {
      "p1": [500, 600],
      "p2": [700, 800],
      "color": [0, 255, 122],
      "thickness": 2
    }
  ]
}
```

`seq` 和 `detect_stamp` 必须来自对应的 `/vision/detect`。`Detector` 端收到 `/vision/result` 后，会在本地缓存中查找同一个 `seq` 的图像，并检查时间戳误差不超过 `timestamp_tolerance_sec`。找不到匹配帧或时间戳不匹配时，会丢弃这条绘图结果，不会画到当前帧上。

## 运行方式

现在使用 Docker 内的系统 Python 环境，通过脚本同时启动所有进程：

```bash
./launch_radar_system.sh
```

脚本会依次准备 ROS 环境，然后后台启动：

- `roscore`
- `roslaunch livox_ros_driver livox_lidar.launch`
- `python3 Counter/init_angle_sender.py`
- `python3 -m communication.Messager`
- `python3 -m detect.Detector`
- `python3 26_main.py`

`26_main.py` 不再用 `subprocess.Popen` 启动 Detector 或 Messager。启动脚本不接收 ROS 外部参数；视觉模式和频率仍由源文件和 YAML 配置决定。

相机模式下，`Detector` 会等待 `26_main.py` 完成相机标定并发布 `/radar/main_ready` 后再打开相机，避免两个进程同时抢占相机。

## 关于帧率

`process_hz: 100` 是检测循环目标频率，不保证 YOLO 实际能达到 100 FPS。实际检测帧率取决于：

- 相机取图耗时。
- YOLO stage1 track 耗时。
- stage2 ROI 分类耗时。
- 绘图和 `imshow` 耗时。

本次拆分解决的是 `26_main.py` 等待检测的问题。也就是说：

- `main` 主循环不再被 YOLO 同步阻塞。
- `/vision/detect` 的实际发布频率仍然由检测端真实推理速度决定。
- 如果检测端仍只有 30 FPS，那是 YOLO/取图/显示链路的实际性能，而不是 ROS 图像传输导致。
- 当前不会自动降频；`process_hz` 和 `detect_publish_hz` 配置为 100 时，就按 100Hz 目标频率调度。

## 注意事项

- 当前不使用 ROS 传图像。
- 当前没有使用 ROS nodelet。Python `rospy` 无法实现真正 nodelet 零拷贝。
- 如果相机只能被一个进程独占打开，`26_main.py` 会先用相机完成标定并释放，再启动 `Detector` 子进程打开相机。
- 如果要让标定也不占用相机，可以后续把标定结果持久化，主程序启动时直接加载外参。

## Messager ROS 输出

`communication/Messager.py` 的发送端已改为 ROS1 topic 输出，默认 topic：

```text
/messge/position
```

类型：`std_msgs/String`，内容是 JSON。所有原来由 `Sender` 串口发送的数据都会通过这一个 topic 发出，`type` 字段区分数据类型，`payload` 是结构化内容，`frame_hex` 是按原 `Sender` 协议生成的串口帧十六进制字符串。

示例：

```json
{
  "seq": 1,
  "stamp": 1720000000.123,
  "type": "map",
  "rate_hz": 4.8,
  "payload": {
    "positions": [[1.0, 2.0]],
    "enemy_ids": [1, 2, 3, 4, 6, 7],
    "our_ids": [101, 102, 103, 104, 106, 107]
  },
  "frame_hex": "..."
}
```

`26_main.py` 不再 import、实例化或调用 `Messager`，只发布 `/messager/state`。`communication/Messager.py` 作为独立 ROS1 节点运行，订阅 `/messager/state` 后自行更新车辆信息。

发送频率由 `configs/main_config.yaml` 控制：

- `map`: 4.9Hz
- `sentry_perception`: 5Hz
- `enemy_hp`: 4.9Hz
- `double_effect_decision`: 25Hz skip 控制
- `main_loop_hz`: 100Hz，用于保证 25Hz 发送不会被 Messager 主循环限制

配置在 `configs/main_config.yaml`：

```yaml
communication:
  send_transport: 'ros1'
  position_topic: '/messge/position'

messager:
  enabled: true
  state_topic: '/messager/state'
  main_loop_hz: 100.0
  map_hz: 4.9
  sentry_hz: 5.0
  enemy_hp_hz: 4.9
  double_effect_hz: 25.0
```

注意：这里按用户指定保留 topic 拼写 `/messge/position`。
