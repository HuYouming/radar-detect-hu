# Lidar 模块说明文档

该模块主要用于处理激光雷达（Lidar）数据，实现点云的接收、筛选、聚类，并结合相机数据进行坐标系转换，从而获取目标在世界坐标系下的位置。

---

## 1. 文件概览与功能

| 文件名 | 类型 | 主要功能 |
| :--- | :--- | :--- |
| **Lidar_25.py** | **ROS 节点** | 负责订阅雷达话题 `/centroid_points`，接收原始点云数据，并维护点云队列 `PcdQueue`，同时也处理部分 TF 变换逻辑。 |
| **Converter.py** | **工具类** | 核心的坐标转换器。提供雷达坐标系、相机坐标系、图像坐标系与世界坐标系之间的相互转换方法，封装了内参、外参矩阵计算。 |
| **fast_search.py** | **算法类** | 提供基于 GPU 加速的点云搜索与筛选功能。包含 `FastSearch` 类，用于在大量点云中快速找到满足特定时空条件的点集或最近邻点。 |
| **PointCloud.py** | **数据结构/算法** | 定义了点云队列 `PcdQueue` 用于缓存多帧点云，并实现了基于 DBSCAN 的点云聚类算法 `cluster`，用于提取物体中心。 |
| **vision_locator.py** | **视觉算法** | 视觉定位器。包含 2025 赛季地图信息（`rm25_points.yaml`），根据 2D 图像坐标和目标高度推算 3D 世界坐标（利用 PnP/透视变换原理）。 |
| **parameters.yaml** | **配置文件** | 存放相机内参 (`intrinsic`)、外参 (`extrinsic`)、聚类参数 (`cluster`) 和雷达过滤阈值 (`filter`)。 |
| **rm25_points.yaml** | **配置文件** | 存放 2025 赛季场地元素的关键点坐标（如高地、前哨站、坡道等），用于视觉辅助定位。 |

---

## 2. 核心算法说明

### 2.1 坐标系转换 (Converter.py)
利用线性代数矩阵运算，实现多坐标系对齐：
- **Lidar -> Camera**: $P_{cam} = R \cdot P_{lidar} + T$
- **Camera -> Image**: $P_{img} = K \cdot P_{cam}$
- **Distortion**: 应用畸变系数去除镜头畸变。
- **Hardcoded Points**: 硬编码了部分 2025 赛季固定建筑物的世界坐标（如敌方堡垒 `enemy_Base_25`）。

### 2.2 快速最近邻搜索 (fast_search.py)
- **GPU 加速**: 使用 `torch` (CUDA) 或 `Control Flow` 将点云数据移动到 GPU 上进行并行距离计算。
- **空间裁剪 (Spatial Pruning)**: 通过简单的 $x, y, z$ 阈值框 (`box filter`) 初步筛选候选点，极大减少计算量。
- **时序筛选**: 根据时间戳 $t$ 筛选特定时间窗口内的点云。

### 2.3 点云聚类 (PointCloud.py)
- **DBSCAN**: 使用 `open3d.cluster_dbscan` 对点云进行基于密度的聚类。
  - **核心思想**: 只有密度足够高（在半径 `eps` 内至少有 `min_points` 个点）的区域才会被归为一类，有效去除稀疏噪声。
  - **输出**: 返回最大簇的中心点 `centroid` 作为物体位置。

### 2.4 视觉反投影 (vision_locator.py)
- **透视变换 (Perspective Transform)**: 
  - 预先计算并存储不同高度平面 ($h\_list$) 的透视变换矩阵。
  - 根据目标所处的区域（由 `rm25_points.yaml` 定义的区域类型决定），选择对应高度平面的矩阵，将 2D 像素坐标反解为 3D 世界坐标。

---

## 3. 主要接口说明

### Converter 类
```python
converter = Converter(my_color, data_loader_path='parameters.yaml')
```
- `__init__`: 加载配置文件，初始化相机内参、外参和畸变参数。
- `camera_to_field_init(capture)`: 交互选点并初始化相机到赛场的定位矩阵。
- `detection_main(box, t)`: 根据检测框解算赛场坐标。
- `camera_to_image(pc)`: 将相机坐标系点反投影到图像坐标。
- `angle_to_quadrant(angle)`: 将角度映射为哨兵预警象限。

### FastSearch 类
```python
searcher = FastSearch(device="cuda", Radius_upperbound=3)
```
- `select_points(pc, pcd)`: 从历史点云 `pcd` 中筛选出与当前点 `pc` 在时空上接近的子集。
- `find_nearest_point(point_cloud, point_0)`: 输入点集和目标点，返回最近点坐标、距离及是否越界。

### PcdQueue 类
```python
queue = PcdQueue(max_size=10)
```
- `add(pc)`: 添加一帧点云。
- `get_all_pc()`: 获取合并后的所有点云数据（用于聚类）。
- `cluster(pcd)`: 对输入点云执行聚类，返回 `(labels, centroid)`。

### Vision_Locator 类
```python
locator = Vision_Locator(intrinsic_matrix, dist_coeffs, world_rvec, world_tvec, extrinsic_matrix)
```
- `get_2d(input_point, height)`: 输入图像点和假设高度，返回世界坐标系下的 $(x, y)$。

---

## 4. 调参指南

所有的核心参数都集中在 `Lidar/parameters.yaml` 中，修改后需重启程序生效。

### 4.1 雷达过滤参数
```yaml
params:
  max_depth: 40      # 忽略超过此距离的点（米）
  width: 1280
  height: 640

# (注：部分代码中可能直接读取以下字段)
lidar:
  height_threshold: 0.5 # 滤除地面点的高度阈值，保留高于此值的点
  min_distance: 0.2     # 最近盲区
```

### 4.2 标定参数 (Calibration)
决定了雷达点云与相机画面的重合度。**这是最关键的参数**。
```yaml
calib:
  extrinsic:
    R: [...] # 3x3 旋转矩阵 (Row-major)
    T: [...] # 3x1 平移向量 (米)
  intrinsic:
    fx, fy, cx, cy # 相机内参
  distortion: [...] # 畸变系数 [k1, k2, p1, p2, k3]
```

### 4.3 聚类参数 (Cluster)
如果发现物体识别不准（过于细碎或粘连），调整这里：
```yaml
cluster:
  eps: 0.40       # 聚类搜索半径。调小 -> 聚类更严格（物体易被打散）；调大 -> 容易粘连不同物体。
  min_points: 10  # 成为一个簇的最少点数。调大 -> 抗噪能力增强，但可能漏掉小物体。
```

### 4.4 场地参数 (rm25_points.yaml)
定义了 2025 赛季场地中固定元素（如 `Center_high` 中路高地, `Self_Slope` 我方坡道等）的 3D 边界点。
- 用于 `vision_locator.py` 判断目标是否在特定区域内，从而赋予正确的高度 $h$ 进行反投影计算。
- **修改方法**: 实际上场测量或根据官方 CAD 图纸更新坐标点 $(x, y, z)$。
