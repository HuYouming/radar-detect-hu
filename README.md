# Hust Radar 2026

HUST（华中科技大学）RoboMaster 2026 赛季雷达站软件。雷达站固定在赛场高处，用工业相机检测所有机器人，将其定位到赛场三维坐标后，通过串口广播给友方机器人。

---

## 快速启动

```bash
# 激活环境
conda activate Radar
source /opt/ros/noetic/setup.bash

# 全系统启动（6个终端窗口并行）
cd /home/py/Hust_Radar_2026
bash 26main.sh

# 单进程调试启动
export PYTHONPATH=/home/py/Hust_Radar_2025:$PYTHONPATH
python3 26_main.py
```

> **两仓库依赖**：本仓库 import `Log.Log` 来自兄弟仓库 `Hust_Radar_2025`，需将其加入 `PYTHONPATH`。

---

## 进程架构

`26main.sh` 按序启动 6 个独立进程，它们通过 **ROS 话题** 交换数据：

```
┌─────────────────────────────────────────────────────────────────┐
│                        26main.sh                                 │
│                                                                   │
│  [1] roscore          ← ROS 消息总线，其他进程依赖它               │
│  [2] livox_ros_driver ← Livox Mid-70 LiDAR 驱动                  │
│        │ 发布 /livox/lidar (PointCloud2)                          │
│        ▼                                                          │
│  [3] Counter/init_angle_sender.py (LidarTracker)                 │
│        │ 订阅 /livox/lidar                                         │
│        │ 发布 /drone_field_xyz (PointCloud2) ←──────────────┐     │
│        │ 发布 /init_yawpitch (Float32MultiArray)             │     │
│                                                              │     │
│  [4] Radio/field_info_publisher.py (RadarRos1Publisher)      │     │
│        │ 接收裁判系统 UDP (127.0.0.1:40001/40002)            │     │
│        │ 发布 /radar/enemy/health_array ←──────────────┐    │     │
│        │ 发布 /radar/enemy/jam_key ←───────────────┐   │    │     │
│        │ 发布 /radar/positions_array 等             │   │    │     │
│                                                    │   │    │     │
│  [5] All_In UDP Receiver (外部程序)                │   │    │     │
│        │ 接收裁判系统原始 UDP，转发到 127.0.0.1     │   │    │     │
│                                                    │   │    │     │
│  [6] 26_main.py  ←─────────────────────────────────┘   │    │     │
│        │  communication/Messager.py 订阅:               │    │     │
│        │    /radar/enemy/jam_key ──────────────────────┘    │     │
│        │    /radar/enemy/health_array ──────────────────────┘     │
│        │    /drone_field_xyz ─────────────────────────────────────┘
│        │                                                           │
│        │  串口 /dev/ttyUSB0  ←────────→  友方机器人               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 26_main.py 内部线程模型

主程序内部有 3 个并发线程，通过 Python 对象（共享内存引用）交换数据：

```
┌──────────────────────────────────────────────────────────────┐
│                       26_main.py 进程                         │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐     │
│  │  Detector 守护线程（detect/Detector.py）              │     │
│  │  capture.get_frame() → YOLO stage1(ByteTrack)        │     │
│  │                      → YOLO stage2(分类装甲板标签)    │     │
│  │  写入：detector._results                              │     │
│  └──────────────────────┬──────────────────────────────┘     │
│                         │ detector.get_results()              │
│                         ▼                                     │
│  ┌─────────────────────────────────────────────────────┐     │
│  │  主循环（~20fps）                                    │     │
│  │  get_new_box() → Converter.detection_main()          │     │
│  │  → vision_locator.post_process()                    │     │
│  │  → CarList.update_car_info()                        │     │
│  │  → messager.update_enemy_car_infos()                │     │
│  └──────────────────────┬──────────────────────────────┘     │
│                         │ 线程锁保护的共享列表                  │
│                         ▼                                     │
│  ┌─────────────────────────────────────────────────────┐     │
│  │  Messager 线程（communication/Messager.py, ~5fps）   │     │
│  │  读 enemy_car_infos / our_car_infos                 │     │
│  │  → Sender 串口发送位置/预警/双倍易伤                  │     │
│  │  → Receiver 解析来自下位机的状态（进程间共享内存）     │     │
│  └─────────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────────┘
```

---

## 文件详解

### 入口与启动脚本

#### `26_main.py`
**系统主循环。** 负责串联所有模块：

1. 初始化 `Capture`（相机）或 `Video`（视频文件），`Detector`（YOLO），`Converter`（坐标解算），`CarList`（目标状态），`Messager`（通信）。
2. 调用 `converter.camera_to_field_init(capture)` 弹出交互式选点窗口，建立相机→赛场的透视变换矩阵。
3. 进入主循环：从 `detector.get_results()` 取推理结果 → `get_new_box()` 修正检测框到底盘位置 → `converter.detection_main()` 解算赛场 XYZ → `carList.update_car_info()` 更新目标状态 → 通知 `messager` 发送。

顶部变量 `mode = "camera" | "video"` 控制使用实体相机还是本地视频文件。

#### `26main.sh`
Shell 脚本，按序启动 6 个 gnome-terminal 窗口（roscore → Livox SDK → 无人机追踪 → UDP接收 → Radio发布 → 主程序），窗口间用 `sleep` 等待依赖就绪。

#### `start_ros.sh`
仅启动 ROS 基础栈（roscore + livox_ros_driver + pclmatcher），用于只需要 LiDAR 而不运行视觉检测时。

---

### `detect/` — 图像采集与检测

#### `detect/Capture.py`
**Hikrobot 工业相机驱动。**

- 类 `Capture`：通过 Hikrobot MvImport SDK（`stereo_camera/MvImport/`）打开 USB3 相机，配置 4024×3036 分辨率、曝光、增益，对外提供 `get_frame() → np.ndarray`。
- 读取 `configs/bin_cam_config.yaml` 获取相机序列号和硬件参数。
- 依赖：`stereo_camera/MvImport/MvCameraControl_class.py`（SDK 封装）。

#### `detect/Video.py`
**视频文件读取，`Capture` 的替代品。**

- 类 `Video`：用 `cv2.VideoCapture` 打开 `.mp4`/`.avi`，同样暴露 `get_frame()` 接口，使主循环代码无需感知来源差异。

#### `detect/Detector.py`
**两阶段 YOLO 目标检测，运行在独立守护线程。**

- `create(capture)`：绑定图像源。
- `start()`：启动检测线程（`detect_thread`），持续 `capture.get_frame()` → `track_infer()` → `classify_infer()` → 结果写入 `self._results`。
- `get_results() → (result_img, results)`：主循环调用，非阻塞取最新结果。每条结果格式：`[xyxy_box, xywh_box, track_id, label, timestamp]`，其中 `label` 是投票稳定后的车辆编号（如 `"R3"`、`"B7"`）。

**两阶段推理：**
- Stage 1（`track_infer`）：全图 ByteTrack，输出带 `track_id` 的检测框。
- Stage 2（`classify_infer`）：对每个 track 的 ROI 裁剪后跑分类模型，判断装甲板标签。每个 `track_id` 独立维护投票计数器（`Track_value`），连续多帧才输出稳定标签。

配置：`configs/detector_config.yaml`（模型路径、置信度阈值、投票衰减周期）。
模型：`weights/new_stage1.pt`（stage1 检测）、`weights/stage3.pt`（stage2 分类）。

---

### `Lidar/` — 坐标解算

#### `Lidar/Converter.py`
**核心坐标变换类，连接视觉检测与赛场坐标系。**

- 构造时读取 `configs/converter_config.yaml`，加载相机内参（fx/fy/cx/cy）、外参（R/T）、畸变系数，初始化 `Vision_Locator`。
- `camera_to_field_init(capture)`：弹出交互式选点 UI（调用 `camera_locator/point_picker.py`），让操作员在当前帧上点选已知赛场地标，计算透视变换矩阵存入 `Vision_Locator`。
- `detection_main(box, t) → [x, y, z]`：走 `camera_results()` → `Vision_Locator.parser()` 纯视觉定位路径。
- `camera_to_image(pc)`：将相机坐标系 XYZ 反投影到图像像素，用于 debug 可视化。

#### `Lidar/vision_locator.py`
**单目视觉透视定位，`Converter` 的定位核心。**

- 类 `Vision_Locator`：启动时按高度平面（地面 0m、高地 0.6m 等）预计算单应矩阵（`_calculate_perspective_matrix`）。
- `parser(xy) → [field_x, field_y]`：将图像像素坐标经透视变换映射到赛场 2D 坐标（米）。
- `get_height(pixel) → float`：根据像素位置估算目标所在高度平面。
- `post_process(xyz, color) → xyz`：对解算结果做边界裁剪和红蓝方坐标系翻转（蓝方原点在 (28, 15)，红方在 (0, 0)）。
- `visualize(points)`：在 `Lidar/RM2026.png` 上绘制俯视小地图，debug 使用。
- 使用 `Lidar/rm25_points.yaml` 中存储的赛场地标世界坐标。

#### `Lidar/fast_search.py`
**GPU 加速点云近邻搜索（遗留 LiDAR 融合辅助模块）。**

- 类 `FastSearch`（依赖 PyTorch/CuPy）：`select_points()` 在点云中截取 2D 检测框附近的点，`find_nearest_point()` 找最近质心。
- 在旧版双模融合（LiDAR + 视觉）时使用；当前 `Converter` 不再实例化或调用此模块。

---

### `Car/` — 目标状态管理

#### `Car/Car.py`
**维护赛场上 12 辆机器人的状态，是各模块的数据中枢。**

- 类 `Car`：单车状态机，字段包括 `track_id`、`car_id`（红1-7/蓝101-107）、`camera_xyz`、`field_xyz`、`is_valid`（信任标志）、`life_span`（生命值倒计时）。每帧被检测到则 `life_up()`，否则 `life_down()`；归零后 `is_valid=False`。
- 类 `CarList`：包含 12 个 `Car` 对象的线程安全集合，读取 `configs/main_config.yaml`。
  - `update_car_info(results)`：将检测结果（`[track_id, car_id, xywh, conf, center, field_xyz]`）写入对应 `Car`。
  - `get_all_info() → list`：返回所有车辆状态，格式为 `[track_id, car_id, center_xy, camera_xyz, field_xyz, color, is_valid]`。
  - `get_car_id(label) → int`：将 YOLO 标签字符串（`"R3"`）转为数值 ID。

**数据流向**：`Detector` 写 → `CarList` 聚合 → `Messager` 读。

---

### `communication/` — 通信与决策

#### `communication/Messager.py`
**通信总控，独立线程运行（~5fps），集成决策逻辑。**

- 管理串口通信（`Sender` + `Receiver`）、ROS 订阅、决策计算。
- **ROS 订阅**（非阻塞回调，线程锁保护）：
  - `/drone_field_xyz`：接收无人机赛场坐标（来自 `Counter/init_angle_sender.py`）。
  - `/radar/enemy/jam_key`：接收裁判系统下发的干扰波密钥（来自 `Radio/field_info_publisher.py`）。
  - `/radar/enemy/health_array`：接收敌方血量数组（来自 `Radio/field_info_publisher.py`）。
- **由主循环调用的接口**：
  - `update_enemy_car_infos(list)`：更新敌方车辆位置。
  - `update_our_car_infos(list)`：更新己方车辆位置。
- **main_loop 内每帧执行**：
  - 从 `Receiver`（进程间 `multiprocessing.Value`）同步己方血量、标记进度、剩余时间、飞镖目标等裁判系统数据。
  - 调用 `send_double_effect_decision()`：根据战场态势（飞镖目标/基地血量/前哨血量/时间）决定是否请求双倍易伤，发送密钥。
  - 调用 `send_map()`（~4.8fps）：打包 12 辆车坐标发给下位机。
  - 分别按 `sentry_hz` 和 `enemy_hp_hz` 调用 `send_sentry_perception()`、`send_sentinel_enemy_HP()`。
- **位置超时降级**：当车辆位置 life 小于 0 时，对应小地图坐标发送 `[0.0, 0.0]`，不再使用预测点或固定备份点。

#### `communication/Sender.py`
**串口帧编码与发送。**

- 管理串口对象（`/dev/ttyUSB0`，115200 baud）。
- 每个 `generate_xxx_info()` 方法将数据打包为裁判系统自定义协议（SOF 帧头 + CRC8 + cmd_id + 数据体 + CRC16 帧尾）。
- 主要发送指令：
  - `send_all_location(infos)`：12 辆车 XY 坐标（cm 单位，uint16）→ 给哨兵小地图。
  - `send_sentinel_field_info(car_infos)`：6 辆敌方车坐标 → 哨兵全局感知。
  - `send_double_effect_analysis_result_info(times, jam_key)`：双倍易伤决策 + 干扰密钥。
  - `send_enemy_HP_info(hp_list)`：敌方血量 → 给哨兵。

#### `communication/Receiver.py`
**串口帧解码，运行在独立进程（由 Messager 启动）。**

- 持续读串口，解析帧头 SOF（0xA5）、CRC 校验、cmd_id，将结果写入 `multiprocessing.Value`/`Array` 共享内存。
- 解析的数据类型（`switch_method` 分发）：
  - `0x0001`（game_status）→ 剩余时间
  - `0x0003`（robot_status）→ 己方血量
  - `0x0105`（mark_progress）→ 标记进度
  - `0x010D`（double_effect）→ 双倍易伤状态 + 拥有次数
  - `0x020C`（dart_target）→ 飞镖目标
  - `0x0401`（interferance）→ 干扰等级
- **通信方式**：通过 `multiprocessing.Value`（bool/int）和 `multiprocessing.Array`（int[]）将数据传给同进程的 `Messager` 主线程，无需 pickle，实时共享。

---

### `Radio/` — 裁判系统数据桥接

#### `Radio/field_info_publisher.py`
**裁判系统 UDP → ROS 话题桥，独立进程。**

- 类 `RadarRos1Publisher`：监听 UDP `127.0.0.1:40001`（位置/血量/状态）和 `40002`（干扰密钥），解析后发布多个 ROS 话题：
  - `/radar/enemy/health_array`（`Float32MultiArray`）→ Messager 订阅
  - `/radar/enemy/jam_key`（`String`）→ Messager 订阅
  - `/radar/positions_array`、`/radar/positions_markers` 等（供 RViz 可视化）
- 依赖 `Radio/radar_udp_receiver.py` 中的 UDP 解析类。

#### `Radio/radar_udp_receiver.py`
**裁判系统 UDP 协议解析。**

- 被 `field_info_publisher.py` import，提供带 CRC8 校验的 UDP 帧解析和回调注册（`_on_position`、`_on_health`、`_on_ammo` 等）。

#### `Radio/interferance_level_sender.py`
**向无线电板发送干扰等级（UDP）。**

- 类 `InterferenceSender`：向 `192.168.3.99:40003` 发送 int 类型的干扰等级指令（1/2/3 级）。
- 被 `Sender.py` 和 `Messager.py` 各自持有一个实例。

---

### `Counter/` — 无人机追踪与哨兵角度计算

#### `Counter/init_angle_sender.py`
**LiDAR 无人机检测与哨兵炮台角度计算，独立进程。**

- 类 `LidarTracker`：
  - 订阅 `/livox/lidar`（`PointCloud2`），将实时点云与预加载的静态赛场地图 PCD（`RM2026_map.pcd`）做差，DBSCAN 聚类找到动态目标（无人机）。
  - 发布 `/drone_field_xyz`（`PointCloud2`）→ `Messager` 通过 ROS 订阅获取无人机坐标。
  - 发布 `/init_yawpitch`（`Float32MultiArray`）→ 哨兵炮台控制。
- 从 `configs/world_points.yaml` 读取地图特征点。
- 将录制数据存入 `Counter/recordings/`（可用于事后回放分析）。

---

### `camera_locator/` — 相机标定辅助

#### `camera_locator/anchor.py`
**特征点容器，在交互式选点流程中使用。**

- 类 `Anchor`：固定容量 5 个点的栈式容器，支持 `append`/`pop`/`clear`。
- `set_by_hand(image, anchor)` 函数：调用 `PointsPicker` 让操作员在图上手动点选，直到选满 5 个点。
- 被 `Converter.camera_to_field_init()` 调用，在系统启动时收集图像-世界对应点对。

#### `camera_locator/point_picker.py`
**高分辨率图像交互选点工具。**

- 类 `PointsPicker`：OpenCV 窗口 + 鼠标回调，支持滚轮缩放（以鼠标为中心）和右键拖拽平移，记录用户左键点击的像素坐标，支持在 4024×3036 原图上精确取点。

---

### `stereo_camera/MvImport/` — Hikrobot 相机 SDK

Hikrobot MvImport Python SDK 封装，被 `detect/Capture.py` import。

| 文件 | 内容 |
|---|---|
| `MvCameraControl_class.py` | `MvCamera` 类，封装相机打开/关闭/参数设置/取帧 |
| `MvCameraControl_header.py` | ctypes 结构体定义，对应 C SDK 头文件 |
| `CameraParams_const.py` | 相机参数枚举常量 |
| `CameraParams_header.py` | 参数结构体 |
| `MvErrorDefine_const.py` | 错误码定义 |
| `PixelType_const.py` / `PixelType_header.py` | 像素格式定义 |

---

### `Tools/` — 工具函数

#### `Tools/Tools.py`
- `frame_control_sleep(fps, last_time)`：通过 sleep 控制主循环帧率。
- `frame_control_skip(fps, last_time) → (is_skip, new_time)`：判断是否跳过本次执行（非阻塞帧率限制），`Messager` 用于控制各类数据的发送频率。

---

### `configs/` — 配置文件

| 文件 | 用途 |
|---|---|
| `main_config.yaml` | 全局：`my_color`（红/蓝）、`is_debug`；通信：串口号/波特率；`car.life_span`；英雄预警多边形区域坐标 |
| `detector_config.yaml` | YOLO 模型路径、置信度阈值、投票 `life_time`、是否录制 |
| `converter_config.yaml` | 相机内参（fx/fy/cx/cy）、外参 R/T、畸变系数 |
| `bin_cam_config.yaml` | 相机硬件参数：分辨率 4024×3036、曝光时间、增益、序列号 |
| `bytetrack.yaml` | ByteTrack 追踪器超参数（`track_thresh`、`match_thresh`、`track_buffer` 等） |
| `world_points.yaml` | 赛场已知地标的世界坐标，用于 LiDAR 外参标定和无人机定位 |

---

### 其他资源文件

| 文件 | 用途 |
|---|---|
| `weights/new_stage1.pt` | Stage1 检测模型（ByteTrack 追踪用） |
| `weights/stage3.pt` | Stage2 装甲板分类模型 |
| `RM2026_map.pcd` | 2026 赛场静态点云地图，由 `init_angle_sender.py` 加载用于无人机背景差分 |
| `Lidar/RM2026.png` | 赛场俯视图，`vision_locator.visualize()` 绘制小地图时使用 |
| `Lidar/rm25_points.yaml` | 赛场地标的相机像素坐标与世界坐标对应表，`Vision_Locator` 初始化时读取 |
| `Lidar/parameters.yaml` | `Converter` 默认参数文件（显式传路径时忽略） |
| `detect/points.yaml` | 用于 `Converter` 内部的选点结果缓存 |
| `data/test_video_trimmed.mp4` | 测试用录像，`mode="video"` 时替代相机 |
| `data/video.avi` | 测试用录像备份 |

---

## 模块间通信方式汇总

| 通信方式 | 发送方 | 接收方 | 数据 |
|---|---|---|---|
| **Python 对象引用**（同进程内存） | `Detector` 线程 | 主循环 | `detector._results`（list）|
| **Python 对象引用**（同进程内存） | 主循环 | `Messager` 线程 | `enemy_car_infos`、`our_car_infos`（threading.Lock 保护）|
| **multiprocessing 共享内存** | `Receiver` 进程 | `Messager` 主线程 | `Value('b')`/`Array('i')` 存储血量/状态/时间等 |
| **ROS 话题** `/livox/lidar` | `livox_ros_driver` | `init_angle_sender.py` | PointCloud2 原始点云 |
| **ROS 话题** `/drone_field_xyz` | `init_angle_sender.py` | `Messager.py` | PointCloud2（单点：无人机 XYZ）|
| **ROS 话题** `/radar/enemy/health_array` | `field_info_publisher.py` | `Messager.py` | Float32MultiArray（6个血量值）|
| **ROS 话题** `/radar/enemy/jam_key` | `field_info_publisher.py` | `Messager.py` | String（6位密钥）|
| **串口** `/dev/ttyUSB0` | `Sender.py` | 友方机器人 | 自定义二进制帧（CRC8+CRC16）|
| **串口** `/dev/ttyUSB0` | 友方机器人 | `Receiver.py` | 自定义二进制帧（裁判系统透传）|
| **UDP** `127.0.0.1:40001/40002` | All_In UDP Receiver | `field_info_publisher.py` | 裁判系统数据（位置/血量/密钥）|
| **UDP** `192.168.3.99:40003` | `interferance_level_sender.py` | 无线电板 | int（干扰等级 1/2/3）|
