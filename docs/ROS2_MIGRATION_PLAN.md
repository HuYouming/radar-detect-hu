# radar-detect-final ROS1 → ROS2 迁移方案

> 面向 RoboMaster 2026 雷达站软件（HUST）的 ROS 通信层迁移。
> 本文档基于仓库当前状态（`ros` 分支，本地工作区 `radar-detect-final`）逐文件分析后给出。

---

## 1. 现状分析

### 1.1 一个关键事实：本仓库不是 catkin 包

```text
$ find . -name package.xml -o -name CMakeLists.txt
（无结果）
```

仓库里**没有任何 `package.xml` / `CMakeLists.txt` / `.launch` 文件**。ROS1 的用法是：

* 用系统里外部的 catkin 工作区（`ws_livox`、`PCLMATCHER`）提供 `livox_ros_driver`、`pclmatcher`；
* 本仓库的 Python 脚本直接 `import rospy`，靠 `roscore` 提供的全局 ROS master 组网；
* 启动靠 `gnome-terminal` 拉多个终端（`26main.sh`）或后台 `&`（`launch_radar_system.sh`）。

**这决定了迁移的性质**：不是"改 `package.xml` 的 format 1→3"，而是
（a）运行期 API `rospy → rclpy` 的代码重构 +
（b）把裸脚本仓库包装成 ament_python 包并改写启动编排 +
（c）迁移两个外部工作区。

### 1.2 ROS1 资产清单

#### 节点

| 文件 | ROS1 节点名 | 说明 | 迁移优先级 |
| --- | --- | --- | --- |
| `26_main.py` | `radar_vision_main` | 视觉主循环：订阅检测、坐标解算、发布状态与绘图指令 | P0 |
| `detect/Detector.py` | `vision_detector` | YOLO 两阶段检测，发布 `/vision/detect`，订阅 `/vision/result` | P0 |
| `communication/Messager.py` | `radar_messager`（另有分支名 `messager_ros_subscriber`） | 决策 + 串口/UDP 下发，订阅 4 个 ROS 话题 | P0 |
| `communication/Receiver.py` | `radar_receiver` | 串口帧解析，发布 `/receiver/state`（latched） | P1 |
| `communication/guess.py` | `radar_guess` | 猜点节点，发布 `/guess/point` | P1 |
| `Counter/init_angle_sender.py` | `lidar_tracker` | 无人机追踪，订阅点云，发布云台角度与无人机坐标 | P0（含传感器） |
| `Radio/field_info_publisher.py` | `radar_ros_publisher` | 裁判系统 UDP → ROS 桥，发布 10 个话题 | P0 |
| `Radio/test/interference_pub.py` | `test_interference_pub` | 测试用 | P3 |
| `Radio/radar_udp_receiver.py` | — | 第 10 行 `import rospy` 但 **未被使用**，直接删除 | P3 |
| 外部 `livox_ros_driver` | launch 决定 | 发布 `/livox/lidar` | 外部 |
| 外部 `pclmatcher` | launch 决定 | 发布 `/centroid_points`（`Lidar_25` 链路，当前 26 赛季未启用） | 外部 |

#### 话题 / 消息类型（ROS1）

| Topic | 类型 | 发布 → 订阅 | ROS1 语义要点 |
| --- | --- | --- | --- |
| `/radar/main_ready` | `std_msgs/String` | main → Detector | **latched**，`queue_size=1` |
| `/vision/detect` | `std_msgs/String`(JSON) | Detector → main | `queue_size=1`，高频（目标 100 Hz） |
| `/vision/result` | `std_msgs/String`(JSON) | main → Detector | `queue_size=1`，按 `seq` 对齐 |
| `/messager/state` | `std_msgs/String`(JSON) | main → Messager | `queue_size=1` |
| `/receiver/state` | `std_msgs/String`(JSON) | Receiver → Messager | **latched**，`queue_size=20` |
| `/guess/point` | `std_msgs/String`(JSON) | guess → Messager | `queue_size=1` |
| `/drone_field_xyz` | `sensor_msgs/PointCloud2` | drone → Messager | 单点云 `width=height=1`，4 个 FLOAT32 字段 |
| `/init_yawpitch` | `std_msgs/Float32MultiArray` | drone → 下位机 | `data=[yaw_deg,pitch_deg]` |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 外部驱动 → drone | 传感器数据，高频大消息 |
| `/gimbal/angle_feedback` | `std_msgs/String` | 外部 → drone | — |
| `/radar/enemy/positions_array` | `std_msgs/Float32MultiArray` | Radio → 调试 | `layout.dim` 有语义 |
| `/radar/enemy/positions_markers` | `visualization_msgs/MarkerArray` | Radio → RViz | `Marker.CYLINDER/TEXT_VIEW_FACING`，`lifetime=2s` |
| `/radar/enemy/health_array` | `std_msgs/Float32MultiArray` | Radio → Messager | 6 元素 |
| `/radar/enemy/health_json` | `std_msgs/String`(JSON) | Radio → 调试 | — |
| `/radar/enemy/ammo_array` | `std_msgs/Float32MultiArray` | Radio → 调试 | 5 元素 |
| `/radar/enemy/team_status` | `std_msgs/String`(JSON) | Radio → 调试 | — |
| `/radar/enemy/module_status` | `std_msgs/String`(JSON) | Radio → 调试 | — |
| `/radar/enemy/buffs` | `std_msgs/String`(JSON) | Radio → 调试 | — |
| `/radar/enemy/jam_key` | `std_msgs/String` | Radio → Messager | 收到后反转字符串 |
| `/radar/enemy/all_info`, `/radar/raw/frame_hex` | `std_msgs/String` | Radio → 调试 | 预留 |
| `/messge/position` | `std_msgs/String`(JSON) | Messager → 外部 | 可选；**注意 `send_transport: 'ros1'` 在代码里其实没有实现分支**，当前恒为串口 |

> 结论：**所有跨节点载荷都已经是 `std_msgs/String` 里的 JSON**，只有两个真正用到了结构化消息（`PointCloud2` 无人机坐标、`MarkerArray` 可视化）。这大幅降低了迁移难度——不需要迁移任何自定义 `.msg`。

#### 用到的 ROS1 API 面

| API | 出现处（文件:行） |
| --- | --- |
| `rospy.init_node` | `26_main.py:168`、`Detector.py:151,618`、`Receiver.py:108,447`、`Messager.py:160,648`、`guess.py:369`、`init_angle_sender.py:187`、`field_info_publisher.py:34`、`interference_pub.py:7` |
| `rospy.Publisher` | 约 20 处（`field_info_publisher` 10 个、`Messager`/`main`/`guess`/`Detector`/`receiver`/`drone`） |
| `rospy.Subscriber` | 约 14 处（`Messager` 6、`guess` 4、`drone` 2、`main` 1、`Detector` 1） |
| `rospy.Rate` | `26_main.py:215`、`Messager.py:543`、`guess.py:360`、`init_angle_sender.py:1214` |
| `rospy.Timer` / `rospy.Duration` | `init_angle_sender.py:293`、`field_info_publisher.py:113,208,227` |
| `rospy.Time.now()` | `26_main`(无)、`Receiver.py:119`、`Messager.py:212,338,348`、`init_angle_sender.py:1249,1259`、`field_info_publisher.py:191,265,315,373` |
| `rospy.spin` / `wait_for_message` | `init_angle_sender.py:1445,1466`、`field_info_publisher.py:427`、`Detector.py:620` |
| `rospy.is_shutdown` / `signal_shutdown` / `sleep` | 20+ 处 |
| `rospy.loginfo*` | 60+ 处（含 `_throttle`） |
| `sensor_msgs.point_cloud2.read_points` | `Messager.py:~195`、`init_angle_sender.py:426` |
| 手工构造 `PointCloud2` | `init_angle_sender.py:1246-1275` |
| `rospy.exceptions.ROSInitException` | `Messager.py:158`、`Receiver.py:107` |
| `latch=True` | `26_main.py:169`、`Receiver.py:109` |

### 1.3 必须一起处理的非 ROS 约束

| 约束 | 现状 | 迁移影响 |
| --- | --- | --- |
| Python 版本 | `conda activate Radar`，为 ROS Noetic 准备（Py3.8） | **rclpy 是 ABI 绑定到系统 Python 的**（Humble=3.10 / Jazzy=3.12），conda Py3.8 里 `import rclpy` 会失败。必须重做环境 |
| 硬件 | Hikrobot USB3 工业相机、Livox Mid-70、`/dev/ttyUSB0` 串口、NVIDIA GPU | 相机 SDK/串口与 ROS 无关，可直接复用；**Livox 驱动是最大外部风险**（见 §8） |
| 路径解析 | `Tools/Paths.py` 用 `Path(__file__).parents[1]` 锚定仓库根 | 若用 `colcon install` 把代码装到 `install/`，根目录会算错 → 配置/权重/地图全部找不到 |
| 多线程模型 | ROS1 回调由 rospy 内部线程池执行，业务主循环随便 `Rate.sleep()` | **ROS2 默认单线程 executor，`create_rate()` 在不 spin 时会死锁**。这是本次迁移最容易踩的坑（见 §5.3） |
| 相机独占 | main 先标定再释放给 Detector | 与 ROS 无关，逻辑保留 |

---

## 2. 目标与选型

| 项目 | 建议 | 理由 |
| --- | --- | --- |
| ROS2 版本 | **Humble**（Ubuntu 22.04） | 生态最全、Livox/ultralytics/torch/open3d 在 Py3.10 下轮子齐全；Jazzy（24.04，Py3.12）可作为二期目标 |
| RMW | `rmw_cyclonedds_cpp` | Livox `PointCloud2` 帧大，CycloneDDS 对大数据更稳，可用 `CYCLONEDDS_URI` 调 buffer |
| Python 环境 | 系统 `python3.10` + `venv --system-site-packages`（**不要继续用 Py3.8 的 conda Radar**） | `source /opt/ros/humble/setup.bash` 后系统 site-packages 里有 `rclpy`；venv 继承它，再 pip 装 torch/ultralytics/open3d |
| 消息方案 | **保持 `std_msgs/String` + JSON 不变** | 跨节点载荷已是 JSON，零自定义 msg，能让迁移聚焦在 API 而不是接口定义 |
| 包结构 | 新建 `ros2_ws/src/radar_ros2`（ament_python），用 `--symlink-install` 指向本仓库源码 | 避免物理移动 `Car/`、`Lidar/`、`detect/` 等目录，保留 `project_path()` 锚点 |
| 兼容策略 | **分两层**：先上 `rospy` 兼容垫片打通链路，再逐节点收敛成原生 `rclpy` | 降低一次性重构 9 个文件、1500 行脚本的风险，可渐进验证 |

---

## 3. 总体路线（4 个阶段）

```text
阶段 A  环境与基建            ── 1~2 天
  A1 装 Ubuntu22.04 + ROS2 Humble + CycloneDDS
  A2 建 venv 并验证 torch/open3d/ultralytics/cupy
  A3 建 ament_python 包骨架 + launch 骨架（先只起一个 hello 节点）
  A4 修 Tools/Paths.py 支持 RADAR_ROOT 环境变量

阶段 B  API 迁移（可并行）     ── 4~6 天
  B1 兼容垫片 radar_ros2/rospy_compat.py
  B2 9 个文件改 import + init/Publisher/Subscriber/Rate/Time/log
  B3 结构化消息修正：PointCloud2 / MarkerArray / Float32MultiArray
  B4 多线程 executor 修正（Messager / main / Detector）

阶段 C  启动编排与 QoS        ── 2~3 天
  C1 写 radar_full.launch.py / vision_only.launch.py
  C2 参数外置（用 ROS2 参数或继续 YAML）
  C3 QoS 按 §6 表逐话题落地
  C4 替换 26main.sh / launch_radar_system.sh / start_ros.sh

阶段 D  外部依赖与联调        ── 3~5 天（风险最大）
  D1 Livox ROS2 驱动选型（§8.1）
  D2 pclmatcher 迁移或降级（26 赛季未启用）
  D3 rosbag 播放/录制链路
  D4 端到端联调 + 回归对比
```

> 阶段 B 和 C 可在 A 完成后并行；阶段 D 的 Livox 选型必须在 B 之前确定，因为它决定点云字段格式。

---

## 4. ROS1 → ROS2 API 映射表

### 4.1 基本映射

| ROS1 (`rospy`) | ROS2 (`rclpy`) | 备注 |
| --- | --- | --- |
| `import rospy` | `import rclpy` + `from rclpy.node import Node` | 无全局单例，必须有 `Node` 对象 |
| `rospy.init_node('x', anonymous=True, disable_signals=True)` | `rclpy.init()` + `super().__init__('x')` | 无 `anonymous`，见 §4.4 |
| `rospy.Publisher(t, T, queue_size=N)` | `self.create_publisher(T, t, N)` | |
| `rospy.Publisher(..., latch=True)` | `self.create_publisher(T, t, latched_qos(N))` | **必须两端都设 TRANSIENT_LOCAL** |
| `rospy.Subscriber(t, T, cb, queue_size=N)` | `self.create_subscription(T, t, cb, N)` | 注意 ROS2 参数顺序是 `(类型, 话题, 回调, qos)` |
| `rospy.Rate(hz)` | `self.create_rate(hz)` | **需后台 executor 在 spin**，否则 `sleep()` 死锁 |
| `rospy.Timer(rospy.Duration(1/hz), cb)` | `self.create_timer(1/hz, cb)` | |
| `rospy.Time.now()` | `self.get_clock().now()` | |
| `rospy.Time.now().to_sec()` | `self.get_clock().now().nanoseconds / 1e9` | |
| `header.stamp = rospy.Time.now()` | `header.stamp = self.get_clock().now().to_msg()` | `builtin_interfaces/Time` |
| `rospy.Duration(2.0)` | `Duration(sec=2, nanosec=0)` | `builtin_interfaces/msg/Duration` |
| `rospy.sleep(s)` | `time.sleep(s)`（或 clock-aware 写法） | |
| `rospy.spin()` | `rclpy.spin(node)` | |
| `rospy.is_shutdown()` | `rclpy.ok()`（取反） | |
| `rospy.signal_shutdown(reason)` | `rclpy.shutdown()` / `node.destroy_node()` | |
| `rospy.loginfo(msg)` | `self.get_logger().info(msg)` | |
| `rospy.logwarn_throttle(5.0, msg)` | `self.get_logger().warn(msg, throttle_duration_sec=5.0)` | 单位是秒，参数名不同 |
| `rospy.get_name()` | `self.get_name()` | |
| `rospy.wait_for_message(t, T)` | `rclpy.wait_for_message(...)` 或 §4.3 手写 | 必须配 TRANSIENT_LOCAL 才能收到已发布的 latched 消息 |
| `rospy.exceptions.ROSInitException`（try/except 判断是否已初始化） | 用 `rclpy.ok()` + 显式持有 `Node`，**去掉双 init 模式** | 见 §4.2 |
| `rospy.get_param/set_param` | `self.declare_parameter()` / `self.get_parameter()` | 本仓库当前未用 rosparam，可暂不迁移 |

### 4.2 必须消灭的"惰性双 init"模式

`Detector.py` 和 `Messager.py` 现在有两套节点名并靠 `try: rospy.get_name() except ROSInitException` 判断是否已初始化：

* `Detector._ensure_ros_interfaces()` → `vision_detector`
* `Messager._init_ros_subscribers()` → `messager_ros_subscriber`，而 `Messager.main()` → `radar_messager`

ROS1 下 `main()` 先 init 所以侥幸能跑；ROS2 下 `rclpy.init()` 只能调一次、`Node` 必须显式持有。**改法**：每个文件定义唯一的 `Node` 子类，在 `__init__` 里一次性建好 pub/sub，删除所有惰性 init 分支。

### 4.3 `wait_for_message` 的两种写法

`Detector.py:620` 等 `main` 的 `/radar/main_ready`。ROS1 是 latched，所以晚订阅也能收到；ROS2 要同时满足"晚加入"和"阻塞等待"：

```python
# 发布端（26_main）
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def latched_qos(depth=1):
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

self.ready_pub = self.create_publisher(String, ready_topic, latched_qos(1))

# 订阅端（Detector），manual 版本（Humble 全版本可用）
def wait_for_message(node, msg_type, topic, qos, timeout_sec=None):
    box = []
    sub = node.create_subscription(msg_type, topic, lambda m: box.append(m), qos)
    deadline = time.monotonic() + timeout_sec if timeout_sec else None
    while rclpy.ok() and not box:
        rclpy.spin_once(node, timeout_sec=0.1)
        if deadline and time.monotonic() > deadline:
            break
    node.destroy_subscription(sub)
    return box[0] if box else None

node.get_logger().info(f"waiting for main ready topic: {READY_TOPIC}")
wait_for_message(node, String, READY_TOPIC, latched_qos(1))
```

> `rclpy.wait_for_message` 在 Humble 之后可用，但上面手写版行为更可控，建议直接采用。

### 4.4 `anonymous=True` 的替代

rclpy 没有匿名节点。因为本仓库**每个进程就是一个节点**，直接去掉 anonymous 即可；若需同机多实例（如双相机调试），用：

```python
super().__init__(f'radar_vision_main_{os.getpid()}')
# 或 launch 时用 Node(name=..., namespace=...)
```

### 4.5 结构化消息的差异

| 场景 | ROS1 写法 | ROS2 写法 |
| --- | --- | --- |
| 读点云 | `from sensor_msgs import point_cloud2 as pc2; for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)` | `from sensor_msgs_py import point_cloud2 as pc2; pts = pc2.read_points(...)` 返回 **结构化 numpy 数组**；要元组语义用 `pc2.read_points_list(...)` |
| 构造单点 PointCloud2 | `msg.data = struct.pack('ffff', ...)`，`header.stamp = rospy.Time.now()` | 逻辑基本不变；`header.stamp = node.get_clock().now().to_msg()`，`point_step/row_step=16` 保持 |
| Marker | `marker.lifetime = rospy.Duration(2.0)` | `marker.lifetime = Duration(sec=2, nanosec=0)` |
| MultiArray | `std_msgs/Float32MultiArray` + `layout.dim` | 同名同结构，`MultiArrayDimension(label=..., size=..., stride=...)` 不变 |
| Header 导入 | `from std_msgs.msg import Header` | 同样可用（`std_msgs/msg/Header`） |

`Messager._drone_field_callback()` 当前写法：

```python
points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
points = list(points)
x, y, z = points[0]
```

ROS2 下 `points[0]` 是 `numpy.void`，`x, y, z` 需要改成：

```python
pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
if len(pts) == 0:
    return
x, y, z = float(pts[0]['x']), float(pts[0]['y']), float(pts[0]['z'])
```

---

## 5. 逐文件改造清单

### 5.1 `Tools/Paths.py`（先做，防坑）

```python
import os
from pathlib import Path

PROJECT_ROOT = Path(os.environ.get("RADAR_ROOT") or Path(__file__).resolve().parents[1]).resolve()
```

`colcon build --symlink-install` 后，launch 里通过 `SetEnvironmentVariable('RADAR_ROOT', <repo_root>)` 注入；源码直跑时回退到原行为，**完全向后兼容**。

### 5.2 兼容垫片 `ros2_ws/src/radar_ros2/radar_ros2/rospy_compat.py`

先用最小改动让 9 个文件跑起来，把"行为差异"集中到一处，后续再逐节点替换。

```python
"""rospy 兼容层：只覆盖本仓库实际用到的 API。"""
import os, time, json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
import threading

_node = None
_executor = None
_spin_thread = None

def init_node(name, anonymous=False, disable_signals=False):
    global _node
    rclpy.init()
    _node = Node(f"{name}_{os.getpid()}" if anonymous else name)
    return _node

def get_node():
    if _node is None:
        raise RuntimeError("rospy_compat: init_node() not called")
    return _node

def ensure_spinning():          # 关键：让 create_rate/sleep 不死锁
    global _executor, _spin_thread
    if _spin_thread is None:
        _executor = MultiThreadedExecutor()
        _executor.add_node(_node)
        _spin_thread = threading.Thread(target=_executor.spin, daemon=True)
        _spin_thread.start()

class Publisher:
    def __init__(self, topic, msg_type, queue_size=10, latch=False):
        qos = QoSProfile(depth=queue_size, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if latch
                                    else DurabilityPolicy.VOLATILE)
        self._p = get_node().create_publisher(msg_type, topic, qos)
    def publish(self, msg): self._p.publish(msg)

class Subscriber:
    def __init__(self, topic, msg_type, cb, queue_size=10):
        qos = QoSProfile(depth=queue_size, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._s = get_node().create_subscription(msg_type, topic, cb, qos)

def Rate(hz):                                     # 需要 executor 在跑
    ensure_spinning()
    return get_node().create_rate(hz)

class _Time:
    @staticmethod
    def now():
        return get_node().get_clock().now()
def is_shutdown():  return not rclpy.ok()
def spin():         ensure_spinning()
def loginfo(msg, *a, **k): get_node().get_logger().info(msg)
# logwarn/logerr/log*_throttle 同理
```

> 注意：垫片的目标是**降低一次性风险**，不是终态。它把 QoS 决策隐式化了，与阶段 C 的显式 QoS 表冲突，因此每个节点在阶段 C 结束时都应改成原生 `rclpy` 并显式声明 QoS。

### 5.3 多线程模型（全项目最关键的一处）

`Messager.run()`、`26_main.py` 主循环、`guess.run()` 都是"**业务主循环 + 订阅回调改共享变量**"。ROS1 里回调自动在别的线程跑。ROS2 必须显式：

```python
class Messager(Node):
    def __init__(self, cfg):
        super().__init__('radar_messager')
        # ... create_subscription / create_publisher ...

    def run(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        threading.Thread(target=executor.spin, daemon=True).start()   # 回调在后台
        rate = self.create_rate(max(self.main_loop_hz, 1.0))          # 主循环节流
        while rclpy.ok() and self.working_flag:
            ...
            rate.sleep()
```

* 如果偷懒用单线程 `while ... rclpy.spin_once(node, timeout_sec=0)`，主循环频率会被回调拖慢，`map_hz=4.9` / `double_effect_hz=25` 的跳帧逻辑会失准。
* `init_angle_sender.py` 已在 `_publish_angles_timer` 用 `rospy.Timer`，ROS2 用 `self.create_timer(1.0/ANGLE_PUBLISH_HZ, self._publish_angles_timer)`，同样需要 executor 在 spin（`rclpy.spin(node)` 即可，它没有自己的主循环）。
* `init_angle_sender` 还有可视化线程用 `rospy.is_shutdown()` → 改 `rclpy.ok()`。

### 5.4 各文件改造要点

#### `26_main.py`（P0）

* `rospy.init_node('radar_vision_main', anonymous=True, disable_signals=True)` → `rclpy.init()` + `class VisionMain(Node)`。
* `ready_pub` 必须 `latched_qos(1)`（否则 Detector 的 `wait_for_message` 会挂）。
* `main_rate = rospy.Rate(...)` → `self.create_rate(...)`，并把 `MultiThreadedExecutor` 放后台线程。
* `while not rospy.is_shutdown()` → `while rclpy.ok()`。
* `VisionRosBuffer` / `MessagerStatePublisher` 两个类需要传入 `node`（或改成嵌套类），因为 ROS2 没有全局 pub/sub 工厂。
* `detect/Video.py`、`Capture.py`、`Converter.py`、`CarList` 完全不动。

#### `detect/Detector.py`（P0）

* 删除 `_ensure_ros_interfaces` 惰性初始化，改为 `Detector` 继承 `Node`（或组合一个 `DetectorRos` Node）。
* `rospy.wait_for_message(READY_TOPIC, String)` → §4.3。
* `spin()` 里 `while not rospy.is_shutdown(): self.process_once(); rospy.sleep(0.001)` → `while rclpy.ok(): self.process_once(); rclpy.spin_once(self, timeout_sec=0.001)`（这里 YOLO 是阻塞的，单线程足够）。
* `rospy.logwarn` → `self.get_logger().warn`。
* `time.time()` 时间戳建议换成 `self.get_clock().now()`，以便未来 `use_sim_time` 回放视频。

#### `communication/Receiver.py`（P1）

* `state_pub = rospy.Publisher(..., latch=True)` → `latched_qos(20)`。
* `rospy.Time.now().to_sec()` → `self.get_clock().now().nanoseconds / 1e9`，并删掉 `rospy.core.is_initialized()` 判断。
* 两处 `while not rospy.is_shutdown()` → `while rclpy.ok()`。
* 串口读取逻辑、CRC、`parse_cmd_id_batch` 完全不动。

#### `communication/Messager.py`（P0）

* 6 个 `rospy.Subscriber` → `self.create_subscription (...)`。
* `_drone_field_callback` 的 `read_points` 语义变化见 §4.5。
* `rospy.Time.now().to_sec()` 3 处 → clock。
* `rospy.Rate` + 主循环 → §5.3 的 executor 模式。
* `serial` 发送路径 `Sender.send_info`、UDP `interference_level_sender` 不动。

#### `communication/guess.py`（P1）

* 4 个订阅 + 1 个发布 → `create_subscription/create_publisher`。
* `rospy.Rate(20)` 主循环 → 后台 executor + `create_rate`（或直接改 `create_timer(0.05, self.publish_once)`，这个节点最干净，推荐改 timer）。

#### `Counter/init_angle_sender.py`（P0，1508 行，改动集中）

* `rospy.init_node('lidar_tracker')` → `LidarTracker(Node)`。
* `rospy.Subscriber(lidar_topic, PointCloud2, ...)`、`gimbal_feedback_sub`、`yawpitch_pub`、`drone_field_xyz_pub` → `create_*`。
* `rospy.Timer(rospy.Duration(1/ANGLE_PUBLISH_HZ), cb)` → `create_timer`。
* `_publish_drone_field_xyz()`：`Header.stamp = self.get_clock().now().to_msg()`；`struct.pack`/`point_step`/字段定义保留。
* `lidar_callback` 里 `pc2.read_points(...)` 第 426 行按 §4.5 改。
* `rospy.loginfo*` 60+ 处全量替换为 `self.get_logger().*`；`_throttle` 用 `throttle_duration_sec=`。
* `rospy.sleep(0.1/0.2)` → `time.sleep`。
* 底部 `signal_handler` + `rospy.signal_shutdown` → 交给 rclpy 默认 SIGINT 处理，或 `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`。
* **注意**：`/drone_field_xyz` 的消费者是 `Messager`，两端的字段名/`point_step=16`/单位必须对齐，联调时优先验证。

#### `Radio/field_info_publisher.py`（P0）

* `RadarRos1Publisher` → `Node` 子类，类名可保留（改注释）。
* 10 个发布者 → `create_publisher`。
* `rospy.Timer(rospy.Duration(5.0), self._publish_stats)` → `create_timer(5.0, ...)`。
* Marker 的 `marker.header.stamp = self.get_clock().now().to_msg()`；`marker.lifetime = Duration(sec=2, nanosec=0)`。
* `rospy.spin()` → `rclpy.spin(node)`；`except rospy.ROSInterruptException` → `except KeyboardInterrupt`（或 `ExternalShutdownException`）。
* 删除未使用的 `Point/PointStamped/PoseArray/Pose/Odometry` 导入。
* UDP 接收线程与 ROS 无关，不动。

#### `Radio/radar_udp_receiver.py`、`Radio/test/interference_pub.py`

* 前者：删除无用的 `import rospy`（第 10 行）。
* 后者：改用 `rclpy` 或直接删除（`26main.sh` 里 UDP Receiver 段落已被注释）。

#### 完全不涉及 ROS 的文件（不用动）

`detect/Capture.py`（实测无 rospy，`ROS_COMMUNICATION.md` 里 `image_processor_node` 的描述已过期）、`detect/Video.py`、`Lidar/*`、`Car/*`、`Log/*`、`Communication/Sender.py`（纯串口协议）、`Radio/interference_level_sender.py`、`Radio/radar_udp_receiver.py`（除去无用 import）、`camera_locator/*`、`PointTracker/*`。

---

## 6. QoS 设计表

ROS2 里 `queue_size` 只是 depth，可靠性/持久性必须显式声明。按当前 ROS1 语义翻译：

| Topic | ROS1 | ROS2 QoS |
| --- | --- | --- |
| `/radar/main_ready` | latched, q=1 | KEEP_LAST 1 / RELIABLE / **TRANSIENT_LOCAL** |
| `/receiver/state` | latched, q=20 | KEEP_LAST 20 / RELIABLE / **TRANSIENT_LOCAL** |
| `/vision/detect` | q=1，100 Hz | KEEP_LAST 1 / **BEST_EFFORT** / VOLATILE |
| `/vision/result` | q=1 | KEEP_LAST 1 / BEST_EFFORT / VOLATILE |
| `/messager/state` | q=1 | KEEP_LAST 1 / BEST_EFFORT / VOLATILE |
| `/guess/point` | q=1 | KEEP_LAST 1 / RELIABLE / VOLATILE |
| `/drone_field_xyz` | q=5 | KEEP_LAST 5 / BEST_EFFORT / VOLATILE |
| `/init_yawpitch` | q=10, 100 Hz | KEEP_LAST 10 / RELIABLE / VOLATILE |
| `/gimbal/angle_feedback` | 默认 | KEEP_LAST 10 / RELIABLE / VOLATILE |
| `/livox/lidar` | 默认 | KEEP_LAST 5 / **BEST_EFFORT** / VOLATILE |
| `/radar/enemy/health_array` | q=10 | KEEP_LAST 10 / RELIABLE / VOLATILE |
| `/radar/enemy/jam_key` | q=10 | KEEP_LAST 10 / RELIABLE / VOLATILE |
| `/radar/enemy/positions_markers` | q=10 | KEEP_LAST 10 / RELIABLE / VOLATILE |
| `/messge/position` | 可选 | KEEP_LAST 5 / RELIABLE / VOLATILE |

**规则**：
1. 只有 ROS1 的 `latch=True` 才用 `TRANSIENT_LOCAL`；其余一律 `VOLATILE`。
2. 传感器/高频流用 `BEST_EFFORT`，避免慢订阅者反压主循环。
3. 若在 RViz2 里发现 marker 不显示，检查订阅端 QoS 与上表不一致（RViz2 默认 RELIABLE+VOLATILE，与 RELIABLE 匹配即可）。
4. 大点云建议全局切 `rmw_cyclonedds_cpp` 并调大 `CYCLONEDDS_URI` 的 buffer。

---

## 7. 包结构与启动编排

### 7.1 ament_python 包骨架

```text
ros2_ws/
└── src/
    └── radar_ros2/
        ├── package.xml                 # format 3, <depend>rclpy std_msgs sensor_msgs ...</depend>
        ├── setup.py                    # entry_points + data_files 指向仓库 symlink
        ├── setup.cfg
        ├── resource/radar_ros2
        ├── launch/
        │   ├── radar_full.launch.py     # 对应 launch_radar_system.sh
        │   ├── vision_only.launch.py    # 只起 main + detector + messager
        │   └── drone_search.launch.py   # init_angle_sender + livox
        └── config/
            └── radar_params.yaml
```

由于现有代码用 `project_path()` 定位配置，**不要物理搬移** `Car/`、`Lidar/`、`detect/`、`communication/`、`Counter/`、`Radio/`、`configs/`、`weight/`、`RM2026_map.pcd`。做法：

* `setup.py` 的 `data_files` 不复制源码，而是通过 `RADAR_ROOT` 环境变量指向仓库源码根；
* 或在整个仓库根放一个 `ros2_ws/`，`colcon build --symlink-install` 只安装薄薄一层 wrapper。

`package.xml` 依赖：

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>sensor_msgs_py</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>visualization_msgs</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

### 7.2 launch 文件示例

```python
# launch/radar_full.launch.py
import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

REPO = os.environ.get("RADAR_ROOT", os.getcwd())

def generate_launch_description():
    env = [
        SetEnvironmentVariable('RADAR_ROOT', REPO),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
    ]

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch_ROS2', 'rviz_MID360_launch.py')),
    )

    drone = Node(package='radar_ros2', executable='drone_search',
                 name='lidar_tracker', output='screen',
                 parameters=[{'lidar_topic': '/livox/lidar'}])

    receiver = Node(package='radar_ros2', executable='receiver',   name='radar_receiver', output='screen')
    guess    = Node(package='radar_ros2', executable='guess',      name='radar_guess',    output='screen')
    detector = Node(package='radar_ros2', executable='detector',   name='vision_detector',output='screen')
    main     = Node(package='radar_ros2', executable='vision_main',name='radar_vision_main', output='screen')
    messager = Node(package='radar_ros2', executable='messager',   name='radar_messager', output='screen')
    radio    = Node(package='radar_ros2', executable='field_info_publisher',
                    name='radar_ros_publisher', output='screen')

    # 对应原脚本的 sleep 顺序：Detector 等 main_ready，靠 latched QoS 保证不丢
    return LaunchDescription(env + [
        livox,
        TimerAction(period=3.0, actions=[drone, receiver, guess, radio]),
        TimerAction(period=5.0, actions=[main]),
        TimerAction(period=6.0, actions=[detector]),
        TimerAction(period=7.0, actions=[messager]),
    ])
```

`setup.py` 的 entry_points：

```python
entry_points={
    'console_scripts': [
        'vision_main = radar_ros2.entry.vision_main:main',
        'detector    = radar_ros2.entry.detector:main',
        'messager    = radar_ros2.entry.messager:main',
        'receiver    = radar_ros2.entry.receiver:main',
        'guess       = radar_ros2.entry.guess:main',
        'drone_search= radar_ros2.entry.drone_search:main',
        'field_info_publisher = radar_ros2.entry.field_info_publisher:main',
    ],
},
```

其中 `radar_ros2/entry/*.py` 是极薄的 wrapper，`sys.path.insert(0, RADAR_ROOT)` 后 import 原模块的 `main()`，避免大改目录结构。

### 7.3 启动脚本替换

| 旧 | 新 |
| --- | --- |
| `roscore` | 不需要（DDS 自动发现） |
| `roslaunch livox_ros_driver livox_lidar_rviz.launch` | `ros2 launch livox_ros_driver2 rviz_MID360_launch.py`（见 §8.1） |
| `python3 26_main.py` | `ros2 run radar_ros2 vision_main` |
| `rospack find` / `rostopic list` / `roslaunch --files` | `ros2 pkg prefix` / `ros2 topic list` / `ros2 launch --show-args` |
| `26main.sh` 的 6 个 gnome-terminal | 保留一个 shell wrapper（ROS2 launch 不开新终端），内部只跑 `ros2 launch radar_ros2 radar_full.launch.py`，需要分终端时再 `gnome-terminal` 包一层 |

`start_ros.sh` 里的 `pkill -f roscore`、`roscore` 全部删除；`launch_radar_system.sh` 的 `is_livox_setup`/`find_livox_setup` 改为查找 livox_ros_driver2 的 `install/setup.bash`。

### 7.4 rosbag

`26_main.py:17` 的注释提到"播放录制 livox mid-70 的 rosbag"。ROS1 `.bag` **不能**被 ROS2 直接播放：

```bash
pip install rosbags
rosbags-convert old.bag --dst new_ros2/     # 生成 ROS2 bag2 (sqlite3/mcap)
ros2 bag play new_ros2/
ros2 bag record -a -o offline_run
```

视频模式（`MODE: 'video'`）本就不依赖点云，可先用视频模式跑通全链路，再处理 bag。

---

## 8. 外部依赖迁移

### 8.1 Livox 驱动（**最高风险项**）

现状：ROS1 用 `livox_ros_driver`（`~/catkin_ws/ws_livox`）跑 Mid-70，话题 `/livox/lidar`。

官方 ROS2 版本 `livox_ros_driver2` 支持 Humble，但**其 README 的支持列表只列了 HAP / Mid360 / Mid360s / Avia2 / Mid360L，未列 Mid-70**（[livox_ros_driver2 README](https://github.com/Livox-SDK/livox_ros_driver2)）。同时它的 `xfer_format` 语义与 ROS1 版不同：

| `xfer_format` | 含义 |
| --- | --- |
| 0 | Livox PointCloud2 (PointXYZRTLT) |
| 1 | Livox 自定义消息 `livox_ros_driver2/msg/CustomMsg` |
| 2 | 标准 PointCloud2 (PCL PointXYZI)，仅 ROS1 |

**三条路线**（阶段 A 必须先验证）：

1. **直接用 livox_ros_driver2 + Mid-70**：先实测 SDK2 能否识别 Mid-70。若可以，设 `xfer_format: 0`，`/livox/lidar` 仍是 `PointCloud2`，`init_angle_sender` 的 `read_points` 只改导入即可，改动最小。**首选**。
2. **ros1_bridge 过渡**：ROS1 侧继续跑旧驱动，用 `ros2 run ros1_bridge dynamic_bridge` 把 `/livox/lidar` 桥到 ROS2。适合比赛前抢时间，但 ROS1+ROS2 双栈常驻，运维复杂。**临时方案**。
3. **自行移植 livox_ros_driver**：把 ROS1 驱动改 `rospy→rclpy`（它是 C++，改 `roscpp→rclcpp`）。工作量大，仅在 1、2 都失败时考虑。

> 无论哪条路，`init_angle_sender` 都必须确认收到的点云字段里有 `x/y/z`，且 `header.stamp` 可解析（代码 426 行读 xyz，441 行读 `msg.header.stamp.to_sec()`）。如果选了 `xfer_format: 1`，要先写一个 CustomMsg → PointCloud2 的转换节点。

### 8.2 pclmatcher / `/centroid_points`

`start_ros.sh` 会起 `PCLMATCHER` 工作区，`main_config.yaml:22` 的 `lidar_topic_name: "/centroid_points"` 是 2025 赛季的融合链路。**2026 主链路（`26_main.py`）是纯视觉，不使用 `/centroid_points`**。建议：本次迁移**不迁移 pclmatcher**，仅保留 2025 的 `25_main.py` / `Lidar/Lidar_25.py` 在 ROS1 分支；若确需 26 赛季使用，需单独评估（PCL + ROS2 的 `pcl_conversions` 迁移）。

### 8.3 硬件 / 串口 / GPU

* Hikrobot `MvImport` SDK、`/dev/ttyUSB0`、UDP 40001/40002/40003：与 ROS 版本无关，代码里已 try/except 包裹，不改。
* `cupy`/`torch`/`open3d`：在 Py3.10 venv 下重新安装即可；注意 open3d 对 Py3.10 有 wheel，cupy 需与 CUDA 版本匹配。

---

## 9. 风险与对策

| 风险 | 等级 | 对策 |
| --- | --- | --- |
| Livox Mid-70 无官方 ROS2 驱动 | **高** | 阶段 A 第一件事就实测；准备 ros1_bridge 兜底；见 §8.1 |
| conda Py3.8 与 rclpy ABI 不兼容 | **高** | 弃用 conda Radar，改 system py3.10 + venv --system-site-packages；阶段 A 验证 `python -c "import rclpy"` |
| `create_rate` 不 spin 导致主循环死锁 | **高** | §5.3 的 MultiThreadedExecutor 后台线程模式；Messager/main/guess 三处重点回归 |
| `Tools/Paths.py` 锚点被 install 目录改变 | 中 | §5.1 的 `RADAR_ROOT` 环境变量 |
| latched 语义丢失导致 Detector 卡在 `wait_for_message` | 中 | `/radar/main_ready`、`/receiver/state` 两端都设 TRANSIENT_LOCAL |
| `read_points` 返回值类型变化导致无人机坐标解析静默失败 | 中 | 在 `Messager` 和 `init_angle_sender` 各加一条启动自检日志 |
| ROS1 `.bag` 无法直接回放 | 中 | `rosbags-convert`；或先用视频模式验证 |
| QoS 不匹配导致话题"连上了但收不到" | 中 | 联调第一步用 `ros2 topic info -v` / `ros2 topic hz` 逐话题核对 §6 表 |
| 大点云 DDS 丢帧 | 中 | `rmw_cyclonedds_cpp` + 调 buffer |
| 一次改 9 文件、1500 行脚本引入回归 | 中 | 兼容垫片分阶段（先垫片打通→再逐节点原生化），每步单独 commit 可回退 |

---

## 10. 验证与验收清单

### 10.1 分阶段验收

```text
A. 环境
  [ ] source /opt/ros/humble/setup.bash && python -c "import rclpy; print(rclpy.__file__)"
  [ ] venv 内 import torch / ultralytics / open3d / cupy 成功
  [ ] ros2 run demo_nodes_py talker/listener 正常
  [ ] livox_ros_driver2 能出点：ros2 topic hz /livox/lidar

B. API
  [ ] python -c "import radar_ros2..."（无 rospy 残留）
  [ ] grep -rn "import rospy\|rospy\." --include=*.py .  结果为空
  [ ] 每个节点单独 ros2 run 能起、能 destroy，无异常栈

C. 编排
  [ ] ros2 launch radar_ros2 radar_full.launch.py 一次起全链路
  [ ] ros2 node list 有 7 个预期节点
  [ ] ros2 topic list 与 §1.2 表一致

D. 端到端
  [ ] 视频模式（MODE: 'video'）跑通 video → detect → main → messager 全链路
  [ ] /vision/detect 频率 ≈ YOLO 实际帧率，/messager/state 跟随
  [ ] 相机模式标定 → ready → Detector 打开相机，无相机争抢
  [ ] 无人机追踪：/livox/lidar → /drone_field_xyz 坐标数值与 ROS1 版本对齐
  [ ] Radio：/radar/enemy/health_array、/radar/enemy/jam_key 到达 Messager
  [ ] 串口下发帧与 ROS1 版本逐字节一致（用 tests/test_communication_protocol.py 回归）
  [ ] 离线回放 30 分钟无内存增长、无话题断流
```

### 10.2 回归对照方法

建议在迁移前用 ROS1 版本录一段"黄金数据"（`ros2 bag`/`rosbag record` 记录 `/vision/detect`、`/messager/state`、`/receiver/state`、`/drone_field_xyz`），迁移后用相同输入回放，逐字段 diff 输出。重点对齐：

* `send_map_infos` 的 12 个坐标；
* `/init_yawpitch` 的 yaw/pitch；
* 串口帧十六进制。

`tests/test_communication_protocol.py` 与 `tests/test_paths.py` 不依赖 ROS，应保持全绿。

---

## 11. 工作量与排期建议

| 阶段 | 内容 | 人日 |
| --- | --- | --- |
| A | 环境 + 包骨架 + Livox 选型验证 | 1.5 ~ 2 |
| B1 | 兼容垫片 + 9 文件 import/init 机械替换 | 1 ~ 1.5 |
| B2 | 结构化消息修正（PointCloud2 / Marker / MultiArray） | 1 |
| B3 | executor / Rate / Timer 线程模型修正 | 1 |
| B4 | 逐节点原生化 + 删除垫片 | 1.5 ~ 2 |
| C | launch / QoS / 启动脚本 | 2 ~ 3 |
| D | Livox 联调 + radio + 端到端回归 | 3 ~ 5 |
| **合计** | | **12 ~ 17 人日** |

**关键路径**：Livox ROS2 驱动（阶段 A 必须最先验证）→ executor 线程模型（B3）→ 端到端联调（D）。

**建议的分支策略**：从当前 `ros` 分支切 `ros2` 分支，阶段 A–B 保持 `26_main.py` 等文件可被 ROS1 和 ROS2 双跑（用 `RADAR_ROS_VERSION` 或兼容垫片），以便随时回退到已验证的 ROS1 版本参赛；阶段 C 完成后再删除 ROS1 代码路径。

---

## 附录 A：当前 ROS1 与目标 ROS2 对照速查

```text
ROS1                                    ROS2 (Humble)
roscore                               → 无（DDS 自动发现）
rosrun / roslaunch                    → ros2 run / ros2 launch
rostopic echo / hz / info             → ros2 topic echo / hz / info -v
rosbag record / play                  → ros2 bag record / play（bag 需转换）
rospack find                          → ros2 pkg prefix
catkin_make / catkin build            → colcon build
source devel/setup.bash               → source install/setup.bash
/opt/ros/noetic/setup.bash            → /opt/ros/humble/setup.bash
rospy                                 → rclpy
queue_size=N + latch=True             → QoSProfile(depth=N, TRANSIENT_LOCAL)
sensor_msgs.point_cloud2              → sensor_msgs_py.point_cloud2
```
