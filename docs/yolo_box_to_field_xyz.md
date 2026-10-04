# YOLO 检测框到赛场 XYZ 坐标的完整链路

本文说明当前项目如何从 YOLO 的一个检测框得到相对地图/赛场坐标系下的 `[x, y, z]`。

主链路如下：

```text
detect/Detector.py
  Detector.process_once()
    -> Detector.infer(frame)
      -> YOLO stage1 track_infer()
      -> parse_results()
      -> ROI 裁剪
      -> YOLO stage2 classify_infer()
      -> 打包 [xyxy_box, xywh_box, track_id, label, timestamp]
      -> publish_detection()

26_main.py
  VisionRosBuffer.detect_callback()
  VisionRosBuffer.get_results()
  get_new_box(xyxy_box, xywh_box)
  converter.detection_main(new_box, t=stamp)

Lidar/Converter.py
  Converter.detection_main()
    -> Converter.camera_results()

Lidar/vision_locator.py
  Vision_Locator.get_height()
  Vision_Locator.parser()
    -> Vision_Locator.get_2d()
  Vision_Locator.post_process()
```

最终得到：

```text
field_xyz = [field_x, field_y, field_z]
```

单位通常是米，前提是 `solvePnP()` 输入的真实赛场点也是米。

---

## 1. 系统启动时先建立相机到赛场的几何关系

在 `26_main.py` 中，主程序初始化 `Converter` 后会调用：

```python
converter.camera_to_field_init(capture)
```

位置：

```text
26_main.py
Lidar/Converter.py::camera_to_field_init()
```

这个步骤不是每个检测框都执行一次，而是在启动时执行一次。它的作用是建立相机画面和赛场世界坐标之间的映射关系。

### 1.1 读取相机内参

`Converter.__init__()` 读取 `configs/converter_config.yaml`：

```python
self.cx = data_loader['calib']['intrinsic']['cx']
self.cy = data_loader['calib']['intrinsic']['cy']
self.fx = data_loader['calib']['intrinsic']['fx']
self.fy = data_loader['calib']['intrinsic']['fy']
```

组成相机内参矩阵：

```python
self.intrinsic_matrix = np.array([
    [self.fx, 0, self.cx],
    [0, self.fy, self.cy],
    [0, 0, 1],
], dtype=np.float32)
```

数学形式：

```text
K =
[ fx  0  cx
   0 fy  cy
   0  0   1 ]
```

其中：

```text
fx, fy: 焦距，像素单位
cx, cy: 主点坐标，像素单位
```

同时读取畸变参数：

```python
self.distortion_matrix = np.array(data_loader['calib']['distortion']['data'])
```

### 1.2 人工点选赛场锚点

`camera_to_field_init()` 中：

```python
pp.caller(image, anchor)
true_points = np.array(self.real_points_25, dtype=np.float32)
pixel_points = np.array(anchor.vertexes, dtype=np.float32)
```

这里有两组点：

```text
true_points: 赛场世界坐标中的 3D 点 [Xw, Yw, Zw]
pixel_points: 图像中的 2D 像素点 [u, v]
```

它们必须一一对应：

```text
true_points[i] <-> pixel_points[i]
```

当前 `self.real_points_25` 中有 5 个点：

```python
self.real_points_25 = [
    enemy_Base_25,
    enemy_Tower_25,
    self_FORTRESS,
    self_Tower_25,
    enemy_FORTRESS_RIGHT_BACK,
]
```

所以当前 PnP 使用的是 5 对 3D-2D 对应点。

### 1.3 使用 solvePnP 求外参

代码：

```python
_, rotation_vector, translation_vector = cv2.solvePnP(
    true_points,
    pixel_points,
    self.intrinsic_matrix,
    self.distortion_matrix,
    flags=cv2.SOLVEPNP_EPNP,
)
```

`solvePnP()` 求解的是：

```text
s * [u, v, 1]^T = K * [R | T] * [Xw, Yw, Zw, 1]^T
```

其中：

```text
[Xw, Yw, Zw]: 赛场坐标系中的点
[u, v]: 图像像素点
K: 相机内参矩阵
R: 赛场坐标系到相机坐标系的旋转
T: 赛场坐标系到相机坐标系的平移
s: 深度尺度因子
```

也就是：

```text
Pc = R * Pw + T
```

展开为：

```text
[Xc]   [r11 r12 r13] [Xw]   [tx]
[Yc] = [r21 r22 r23] [Yw] + [ty]
[Zc]   [r31 r32 r33] [Zw]   [tz]
```

注意：`translation_vector` 的单位跟 `true_points` 的单位一致。当前真实点是米，则 `translation_vector` 也是米。

### 1.4 Rodrigues 把旋转向量转成旋转矩阵

代码：

```python
rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
```

`solvePnP()` 返回的是旋转向量 `rvec`，不是 3x3 矩阵。`cv2.Rodrigues()` 转换后得到：

```text
R =
[r11 r12 r13
 r21 r22 r23
 r31 r32 r33]
```

### 1.5 拼成 4x4 齐次变换矩阵

代码：

```python
transformation_matrix = np.hstack((rotation_matrix, translation_vector.reshape(-1, 1)))
transformation_matrix = np.vstack((transformation_matrix, [0, 0, 0, 1]))
self.field_to_camera_matrix = transformation_matrix
```

数学形式：

```text
T_field_to_camera =
[r11 r12 r13 tx
 r21 r22 r23 ty
 r31 r32 r33 tz
  0   0   0  1]
```

它表示：

```text
[Xc, Yc, Zc, 1]^T = T_field_to_camera * [Xw, Yw, Zw, 1]^T
```

即：

```text
赛场坐标 -> 相机坐标
```

代码还会求逆：

```python
self.camera_to_field_matrix = np.linalg.inv(transformation_matrix)
```

理论上：

```text
T_camera_to_field = inverse(T_field_to_camera)
```

如果：

```text
T_field_to_camera = [R, T]
```

则：

```text
T_camera_to_field = [R^T, -R^T * T]
```

不过当前纯视觉定位主链路主要使用 `field_to_camera_R/T` 和单应矩阵，不直接用 `camera_to_field_matrix` 来反变换 YOLO 框。

### 1.6 初始化 Vision_Locator

代码：

```python
self.vision_locator = Vision_Locator(
    intrinsic_matrix=self.intrinsic_matrix,
    dist_coeffs=self.distortion_matrix,
    world_rvec=self.field_to_camera_R,
    world_tvec=self.field_to_camera_T,
    extrinsic_matrix=self.field_to_camera_matrix,
    img=img,
)
```

这里把相机内参、畸变、PnP 求出的外参传给 `Vision_Locator`。

---

## 2. Vision_Locator 初始化时做了什么

`Vision_Locator` 是后续把图像点映射到赛场坐标的核心类。

### 2.1 为每个赛场区域创建 Parser_Points

在 `Vision_Locator.__init__()` 中：

```python
self.points_map["Center_high"] = Parser_Points(...)
self.points_map["Enemy_Hero_High"] = Parser_Points(...)
self.points_map["Self_Hero_High"] = Parser_Points(...)
self.points_map["Enemy_Left_High"] = Parser_Points(...)
self.points_map["Self_Left_High"] = Parser_Points(...)
self.points_map["Enemy_Slope"] = Parser_Points(...)
self.points_map["Self_Slope"] = Parser_Points(...)
self.points_map["Enemy_Left_High_Slope"] = Parser_Points(...)
self.points_map["Enemy_Right_High"] = Parser_Points(...)
self.points_map["Self_Right_High"] = Parser_Points(...)
self.points_map["Self_Fortress"] = Parser_Points(...)
self.points_map["Enemy_Fortress"] = Parser_Points(...)
self.points_map["Self_Narrow_Path"] = Parser_Points(...)
```

每个 `Parser_Points` 对应一个赛场区域，例如高地、坡道、堡垒等。

### 2.2 Parser_Points 读取区域 3D 多边形

`Parser_Points.__init__()`：

```python
self.points_3d = self.read_points(name)
self.points_2d = self.world_to_camera()
```

`read_points()` 从 `Lidar/rm25_points.yaml` 读取该区域的世界坐标点：

```python
points.append((x, y, z))
```

这些点是赛场坐标系下的 3D 多边形顶点：

```text
P_region_world_i = [Xw_i, Yw_i, Zw_i]
```

### 2.3 把区域 3D 多边形投影到图像上

`Parser_Points.world_to_camera()`：

```python
temp_2d, _ = cv2.projectPoints(
    points,
    self.world_rvec,
    self.world_tvec,
    self.K,
    self.dist_coeffs,
)
```

数学关系仍然是：

```text
s * [u, v, 1]^T = K * [R | T] * [Xw, Yw, Zw, 1]^T
```

得到该区域在图像里的 2D 多边形：

```text
P_region_image_i = [u_i, v_i]
```

后续会用这个多边形判断检测点落在哪个赛场区域。

### 2.4 给每个区域设置高度

`Vision_Locator.__init__()` 中手动设置：

```python
self.points_map["Center_high"].heights = 0.3
self.points_map["Enemy_Slope"].heights = 0.325
self.points_map["Self_Slope"].heights = 0.325
self.points_map["Enemy_Left_High_Slope"].heights = 0.08
self.points_map["Enemy_Fortress"].heights = 0.151
self.points_map["Self_Fortress"].heights = 0.151
self.points_map["Enemy_Hero_High"].heights = 0.6
self.points_map["Self_Hero_High"].heights = 0.6
self.points_map["Enemy_Right_High"].heights = 0.2
self.points_map["Self_Right_High"].heights = 0.2
self.points_map["Enemy_Left_High"].heights = 0.2
self.points_map["Self_Left_High"].heights = 0.2
self.points_map["Self_Narrow_Path"].heights = 0.0
```

这些高度表示检测点如果落在该区域内，就认为目标底部/所在平面的高度为该值。

### 2.5 预计算不同高度平面的单应矩阵

`Vision_Locator.__init__()`：

```python
self.h_list = [0.0, 0.08, 0.151, 0.2, 0.325, 0.3, 0.6]
self.Perspective_matrix = self._calculate_perspective_matrix()
```

`_calculate_perspective_matrix()` 对每个高度 `h` 做一次单应矩阵计算。

先构造同一高度平面上的 4 个世界点：

```python
world_points = [
    [12, -6, self.armor_height + h],
    [16, -6, self.armor_height + h],
    [16, -8, self.armor_height + h],
    [12, -8, self.armor_height + h],
]
```

其中：

```python
self.armor_height = 0.15
```

所以用于投影的高度是：

```text
Z = armor_height + h
```

再把这 4 个 3D 点投影到图像：

```python
image_points, _ = cv2.projectPoints(
    world_points,
    self.world_rvec,
    self.world_tvec,
    self.K,
    self.dist_coeffs,
)
```

然后定义对应的赛场 2D 平面点：

```python
world_points2D = np.array([
    [12, -6],
    [16, -6],
    [16, -8],
    [12, -8],
], dtype=np.float32)
```

最后求图像平面到赛场平面的透视变换：

```python
ans[h] = cv2.getPerspectiveTransform(
    image_points.reshape(-1, 2),
    world_points2D,
)
```

数学形式：

```text
lambda * [Xw, Yw, 1]^T = H_h * [u, v, 1]^T
```

其中：

```text
H_h: 高度 h 对应的单应矩阵
[u, v]: 图像像素坐标
[Xw, Yw]: 赛场平面坐标
lambda: 齐次坐标尺度
```

展开为：

```text
[x']   [h11 h12 h13] [u]
[y'] = [h21 h22 h23] [v]
[w']   [h31 h32 h33] [1]
```

归一化得到：

```text
Xw = x' / w'
Yw = y' / w'
```

---

## 3. Detector 如何从 YOLO 得到框

检测进程在 `detect/Detector.py::process_once()` 中每次处理一帧：

```python
frame = self.capture.get_frame()
infer_result = self.infer(frame)
result_img, results = infer_result
self.publish_detection(frame, results, stamp)
```

### 3.1 Stage 1：YOLO + ByteTrack

`Detector.infer()` 中：

```python
results = self.track_infer(frame)
```

`track_infer()` 实现：

```python
results = self.model_car.track(
    frame,
    persist=True,
    tracker=self.tracker_path,
    verbose=False,
)
```

输出包含：

```text
results[0].boxes.xywh
results[0].boxes.conf
results[0].boxes.id
```

`parse_results()`：

```python
confidences = results[0].boxes.conf.cpu().numpy()
boxes = results[0].boxes.xywh.cpu().numpy()
track_ids = results[0].boxes.id.int().cpu().tolist()
```

此处 YOLO 的 `xywh` 按 Ultralytics 常规定义是：

```text
box = [x_center, y_center, width, height]
```

### 3.2 Stage 2：裁剪 ROI 做装甲板分类

对每个 YOLO 框：

```python
x, y, w, h = box
x_left = x - w / 2
y_left = y - h / 2
roi = frame[int(y_left): int(y_left + h), int(x_left): int(x_left + w)]
```

数学形式：

```text
x_left = x_center - width / 2
y_left = y_center - height / 2
x_right = x_center + width / 2
y_right = y_center + height / 2
```

当前 ROI 裁剪代码等价于：

```text
roi = image[y_left : y_right, x_left : x_right]
```

然后：

```python
label_list, conf_list = self.classify_infer(roi_list)
```

`classify_infer()` 用第二阶段 YOLO 模型对每个 ROI 分类，得到装甲板类别，并对每个 `track_id` 维护 `Track_value` 投票。

### 3.3 打包输出检测结果

当前代码后面重新取出 `box`：

```python
x, y, h, w = box
```

注意这里和标准 `xywh` 的顺序不一致。标准应该是：

```python
x, y, w, h = box
```

但当前实现是：

```python
x, y, h, w = box
```

这会把 YOLO 输出的宽高交换。后面实际计算为：

```python
x_left = int(x - w / 2)
y_left = int(y - h / 2)
x_right = int(x + w / 2)
y_right = int(y + h / 2)

xywh_box = [x, y, w, h]
xyxy_box = [x_left, y_left, x_right, y_right]
zip_results.append([xyxy_box, xywh_box, track_id, label, time_stamp])
```

因此，按当前代码实际行为：

如果 YOLO 原始框是：

```text
box_yolo = [x, y, w_yolo, h_yolo]
```

那么当前打包后的变量变成：

```text
h = w_yolo
w = h_yolo
```

当前输出的：

```text
xywh_box = [x, y, h_yolo, w_yolo]
xyxy_box = [
    x - h_yolo / 2,
    y - w_yolo / 2,
    x + h_yolo / 2,
    y + w_yolo / 2,
]
```

这很可能是一个宽高写反的 bug。后续文档按当前代码实际行为继续说明，但如果要让定位更符合检测框，应改为：

```python
x, y, w, h = box
```

### 3.4 ROS 发布检测结果

`publish_detection()`：

```python
payload = self._build_detection_payload(frame, results, stamp)
self.detect_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
```

`_build_detection_payload()` 对每个检测结果打包：

```python
detections.append({
    "xyxy": self._to_builtin(xyxy_box),
    "xywh": self._to_builtin(xywh_box),
    "track_id": int(track_id),
    "label": str(label),
    "stamp": float(det_stamp),
})
```

消息发布到配置中的：

```text
/vision/detect
```

---

## 4. 26_main.py 如何消费检测框

`26_main.py` 中创建：

```python
vision_buffer = VisionRosBuffer(detect_topic, result_topic)
```

`VisionRosBuffer` 订阅 `/vision/detect`，回调：

```python
def detect_callback(self, msg):
    payload = json.loads(msg.data)
    self.latest_detection = payload
```

主循环里：

```python
infer_result = vision_buffer.get_results()
```

`get_results()` 将 ROS JSON 转回列表：

```python
results.append([
    item.get("xyxy", []),
    item.get("xywh", []),
    int(item.get("track_id", -1)),
    item.get("label", "NULL"),
    float(item.get("stamp", payload.get("stamp", time.time()))),
])
```

每条结果格式：

```text
[xyxy_box, xywh_box, track_id, label, stamp]
```

主循环中解包：

```python
xyxy_box, xywh_box, track_id, label, stamp = result
```

如果分类没识别出来：

```python
if label == "NULL":
    continue
```

只有识别出车辆标签的框才会继续定位。

---

## 5. get_new_box 如何从 box 取定位点

位置：

```text
26_main.py::get_new_box()
```

代码：

```python
def get_new_box(xyxy, xywh):
    x1, y1, x2, y2 = xyxy
    x, y, w1, h1 = xywh

    new_x = x + w1 / 2
    new_y = y2

    if y > 3036 / 2:
        w = x2 - x1
        h = y2 - y1
        new_x1 = x1 + w / 2
        new_y1 = y1 + h / 2 + h / 3
        new_w = w
        new_h = h
    else:
        w = x2 - x1
        h = y2 - y1
        new_x1 = x1 + w / 2
        new_y1 = y1 + h / 2 + h / 7
        new_w = w
        new_h = h

    return [new_x, new_y, new_x1, new_y1]
```

虽然函数名像是返回新框，但后续真正使用的是返回列表的前两个值。

后续 `Converter.camera_results()` 会这样解包：

```python
x, y, w, h = box
camera_point = np.array([[[min(x, self.width), min(y, self.height)]]], dtype=np.float32)
```

所以 `get_new_box()` 返回的：

```text
new_x -> 被当成图像点 u
new_y -> 被当成图像点 v
```

即：

```text
u = new_x = x + w1 / 2
v = new_y = y2
```

从意图上看，作者想取检测框底部附近的点，因为车辆在地面上，底部点比框中心更接近车辆接地点。

理想的“底边中心点”通常应该是：

```text
u = (x1 + x2) / 2
v = y2
```

或如果 `xywh` 中 `x` 是中心点：

```text
u = x
v = y2
```

但当前代码是：

```text
u = x + w1 / 2
```

如果 `xywh_box[0]` 是中心点，这会取到框的右边缘附近，而不是底边中心。并且由于前面 `Detector.infer()` 可能已经交换了宽高，这里的 `w1` 也可能不是原始宽度。

这是当前定位误差的一个重要风险点。

---

## 6. Converter.detection_main 如何处理这个图像点

主循环调用：

```python
center = converter.detection_main(new_xywh_box, t=stamp)
```

`detection_main()` 实现：

```python
def detection_main(self, box, t):
    return self.camera_results(box, t)
```

### 6.1 camera_results 取像素点

`Converter.camera_results()`：

```python
def camera_results(self, box, t):
    x, y, w, h = box
    camera_point = np.array([[[min(x, self.width), min(y, self.height)]]],
                            dtype=np.float32)
    height = self.vision_locator.get_height(camera_point)
    [x, y] = self.vision_locator.parser(camera_point)
    y += 15
    return [x, y, height + 0.15, t]
```

这里变量名 `camera_point` 容易误导。它不是相机坐标系 3D 点，而是图像像素点：

```text
camera_point = [[[u, v]]]
```

其中：

```text
u = min(x, image_width)
v = min(y, image_height)
```

`self.width`、`self.height` 来自 `configs/converter_config.yaml`：

```text
width: 4024
height: 3036
```

---

## 7. 如何判断检测点处于哪个高度

代码：

```python
height = self.vision_locator.get_height(camera_point)
```

`Vision_Locator.get_height()`：

```python
def get_height(self, input_point):
    for points in self.points_map.values():
        height = points.return_height(input_point)
        if height > 0:
            return height
    return 0
```

它遍历所有 `Parser_Points` 区域，调用：

```python
points.return_height(input_point)
```

`Parser_Points.return_height()`：

```python
if cv2.pointPolygonTest(
    np.array(self.points_2d, dtype=np.int32),
    (int(input_point[0][0][0]), int(input_point[0][0][1])),
    False,
) > 0:
    return self.heights
else:
    return 0
```

数学含义：

```text
给定图像点 p = [u, v]
判断 p 是否在区域多边形 Polygon_i 内
```

如果：

```text
p inside Polygon_i
```

则：

```text
height = region_i.height
```

否则继续判断下一个区域。

如果不在任何特殊区域内：

```text
height = 0
```

---

## 8. 如何从图像点得到赛场平面坐标 x, y

代码：

```python
[x, y] = self.vision_locator.parser(camera_point)
```

`parser()`：

```python
def parser(self, xy):
    temp_height = self.get_height(xy)
    if temp_height > 0.79:
        return [19.322, -1.915]
    return self.get_2d(xy, temp_height)
```

正常情况下走：

```python
return self.get_2d(xy, temp_height)
```

### 8.1 get_2d 选择对应高度的单应矩阵

`get_2d()`：

```python
height = int(height)
Perspective_matrix = self.Perspective_matrix[height]
src_point_mat = np.array([input_point], dtype=np.float32)
src_point_mat = src_point_mat.reshape(1, 1, 2)
dst_point_mat = cv2.perspectiveTransform(src_point_mat, Perspective_matrix)
results = tuple(dst_point_mat[0][0])
return results
```

这里有一个需要注意的实现细节：

```python
height = int(height)
```

如果 `height = 0.325`，`int(height)` 会变成 `0`。但 `self.Perspective_matrix` 的 key 是：

```python
self.h_list = [0.0, 0.08, 0.151, 0.2, 0.325, 0.3, 0.6]
```

字典 key 中有 `0.325`，但没有通过 `int(0.325)` 区分高度。由于 Python 中 `0` 和 `0.0` 作为字典 key 等价，因此当前很多非零小数高度会被转成 `0`，最终使用地面高度的单应矩阵。

按当前代码实际行为：

```text
height = 0.08  -> int(height) = 0 -> 使用 H_0.0
height = 0.151 -> int(height) = 0 -> 使用 H_0.0
height = 0.2   -> int(height) = 0 -> 使用 H_0.0
height = 0.3   -> int(height) = 0 -> 使用 H_0.0
height = 0.325 -> int(height) = 0 -> 使用 H_0.0
height = 0.6   -> int(height) = 0 -> 使用 H_0.0
```

因此当前实现基本上总是用 `H_0.0` 做透视变换。这个行为很可能不是原本意图。

如果按设计意图，应该直接用原始高度：

```python
Perspective_matrix = self.Perspective_matrix[height]
```

或做最近高度匹配。

### 8.2 透视变换公式

`cv2.perspectiveTransform()` 做的是：

```text
lambda * [Xw, Yw, 1]^T = H * [u, v, 1]^T
```

令：

```text
H =
[h11 h12 h13
 h21 h22 h23
 h31 h32 h33]
```

则：

```text
x' = h11*u + h12*v + h13
y' = h21*u + h22*v + h23
w' = h31*u + h32*v + h33
```

归一化：

```text
Xw = x' / w'
Yw = y' / w'
```

所以：

```text
[u, v] -> [Xw, Yw]
```

这一步得到的是赛场坐标系下的二维位置。

---

## 9. 如何组装最终 XYZ

回到 `Converter.camera_results()`：

```python
height = self.vision_locator.get_height(camera_point)
[x, y] = self.vision_locator.parser(camera_point)
y += 15
return [x, y, height + 0.15, t]
```

所以初步结果是：

```text
X = perspective_x
Y = perspective_y + 15
Z = detected_region_height + armor_height
```

其中：

```text
armor_height = 0.15
```

数学表示：

```text
P_field_raw = [X, Y, Z]

X = Xw
Y = Yw + 15
Z = h_region + 0.15
```

加 `15` 的原因是当前 `rm25_points.yaml` 和部分中间计算使用了负 y 区间，最终这里把它平移到更常见的赛场 `0~15` 范围。

`detection_main()` 返回：

```text
[X, Y, Z, timestamp]
```

---

## 10. 根据红蓝方做坐标后处理

主循环中：

```python
center = converter.vision_locator.post_process(center, global_my_color)
```

`post_process()`：

```python
def post_process(self, xy, color):
    if color == 'Blue':
        x, y, z, _ = xy
        [x, y, z] = [28 - x, 15 - y, z]
    else:
        x, y, z, _ = xy
        [x, y, z] = [x, y, z]
    return [x, y, z]
```

如果当前己方颜色是红方：

```text
P_field = [X, Y, Z]
```

如果当前己方颜色是蓝方：

```text
P_field = [28 - X, 15 - Y, Z]
```

这相当于把坐标系绕赛场中心翻转。赛场尺寸按当前代码假设为：

```text
length_x = 28
width_y = 15
```

最终：

```python
field_xyz = center
```

并写入：

```python
carList_results.append([
    track_id,
    carList.get_car_id(label),
    xywh_box,
    1,
    center,
    field_xyz,
])
```

---

## 11. 从一个框到 XYZ 的完整公式串联

假设 YOLO 原始输出：

```text
B_yolo = [xc, yc, bw, bh]
```

按设计意图，检测框角点应为：

```text
x1 = xc - bw / 2
y1 = yc - bh / 2
x2 = xc + bw / 2
y2 = yc + bh / 2
```

取底边中心作为定位像素点：

```text
u = (x1 + x2) / 2 = xc
v = y2
```

但按当前代码实际行为，需要注意两点：

```text
1. Detector.infer() 打包时疑似交换了 w/h
2. get_new_box() 使用 u = x + w1 / 2
```

当前代码实际更接近：

```text
u = xywh_box[0] + xywh_box[2] / 2
v = xyxy_box[3]
```

得到图像点：

```text
p_img = [u, v]
```

判断高度：

```text
h_region = height_i, if p_img inside Polygon_i
h_region = 0, otherwise
```

选择高度平面的单应矩阵：

```text
H = H(h_region)
```

当前代码因为 `int(height)`，实际大概率使用：

```text
H = H(0.0)
```

透视变换：

```text
[x']   [h11 h12 h13] [u]
[y'] = [h21 h22 h23] [v]
[w']   [h31 h32 h33] [1]
```

归一化：

```text
X = x' / w'
Y_raw = y' / w'
```

平移 y：

```text
Y = Y_raw + 15
```

高度：

```text
Z = h_region + 0.15
```

红方输出：

```text
P_field = [X, Y, Z]
```

蓝方输出：

```text
P_field = [28 - X, 15 - Y, Z]
```

---

## 12. 当前实现中影响定位精度的几个关键点

### 12.1 Detector.infer() 疑似宽高交换

当前代码：

```python
x, y, h, w = box
```

更合理的是：

```python
x, y, w, h = box
```

否则 `xyxy_box` 和 `xywh_box` 的宽高会被交换。

### 12.2 get_new_box() 当前取的不是底边中心

当前代码：

```python
new_x = x + w1 / 2
new_y = y2
```

如果 `x` 是中心点，`x + w1 / 2` 是右边缘，不是中心。

更像底边中心的写法是：

```python
new_x = (x1 + x2) / 2
new_y = y2
```

或：

```python
new_x = x
new_y = y2
```

### 12.3 Vision_Locator.get_2d() 把高度强转 int

当前代码：

```python
height = int(height)
Perspective_matrix = self.Perspective_matrix[height]
```

这会导致 `0.08`、`0.151`、`0.2`、`0.3`、`0.325`、`0.6` 全部变成 `0`，多数情况下使用地面单应矩阵。

更合理的是保持高度 key：

```python
Perspective_matrix = self.Perspective_matrix[height]
```

或用最近高度：

```python
height_key = min(self.h_list, key=lambda h: abs(h - height))
Perspective_matrix = self.Perspective_matrix[height_key]
```

### 12.4 PnP 平移单位不要额外除以 1000

如果 `true_points` 是米，`solvePnP()` 输出的 `translation_vector` 也是米。

因此：

```python
self.camera_to_field_matrix[:3, 3] /= 1000
```

在米制输入下不应该执行。虽然当前主定位链路主要不直接用 `camera_to_field_matrix`，但这个矩阵如果被其他逻辑使用，会产生 1000 倍尺度错误。

---

## 13. 一句话总结

当前项目不是通过深度直接从 YOLO 框恢复完整 3D，而是：

```text
YOLO 框 -> 取框底部附近的一个像素点 -> 判断该点所在赛场区域得到高度
       -> 使用该高度平面的图像到赛场单应矩阵得到 [x, y]
       -> z = 区域高度 + 装甲板高度补偿
       -> 根据红蓝方做坐标翻转
       -> 输出赛场 [x, y, z]
```

核心数学工具是：

```text
1. solvePnP: 用人工点选建立相机外参
2. projectPoints: 把赛场区域投影到图像
3. pointPolygonTest: 判断检测点落在哪个区域
4. getPerspectiveTransform + perspectiveTransform: 把图像点映射到赛场平面坐标
```

---

## 14. 定位模式开关（`configs/raycast_config.yaml` 的 `raycast.mode`）

上面 1~13 节描述的是**老透视链路**。现在 raycast 已经并进同一条 `Converter.camera_results()`，
用配置里的一个字段切换三条链路，不必改代码：

| `raycast.mode` | 行为 | 打空/撞结构时 | 是否加载 raycast 网格 |
|---|---|---|---|
| `raycast_only` | 只用 raycast | **丢弃该检测**（返回 `None`），绝不使用老算法 | 是（加载失败直接报错退出） |
| `raycast_fallback` | raycast 优先 | 回退老透视算法（默认，与加 raycast 之前行为一致） | 是 |
| `perspective_only` | 只用老算法 | — | **否**（省内存与启动时间） |

```yaml
raycast:
  enabled: true
  mode: "raycast_fallback"      # 只改这一行
  stats_print_every: 200        # 每 N 次检测打印一次来源统计; 0 = 关闭
```

要点：

- **`raycast_only` 下打空即丢弃**。`Converter.detection_main()` 返回 `None`，`26_main.py` 遇到 `None`
  就 `continue` 跳过本次检测（不进入 `CarList`）。该车不会发出错误坐标，而是靠 `CarList`
  的 `life_span` 生命周期逐渐过期变为不可信。
- **`raycast_only` 不做静默降级**：网格路径写错、open3d 缺失、外参/单位异常都会直接抛错退出，
  避免"以为在用 raycast，其实一直在用老算法"。
- **`raycast_fallback` 与 `perspective_only` 的区别**只在于谁优先；两者都能出结果。
- 旧开关 `enabled` 与 `fallback_to_perspective` 在没有 `mode` 字段时仍按老语义推导，
  老配置文件行为不变（映射关系见 `Lidar/Converter.py: resolve_localization_mode`）。
- 运行时可观察：启动打印 `[loc] 定位模式: ...`，运行中每 `stats_print_every` 次打印
  命中/打空/丢弃/老算法计数，退出时打印 `[loc] 定位统计: {...}`（也可调
  `converter.localization_stats()`）。
