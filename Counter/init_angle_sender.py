import rospy
import numpy as np
import threading
import time
import open3d as o3d
from collections import deque
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import sensor_msgs.point_cloud2 as pc2
import os
import sys
import copy
import yaml
from scipy.spatial.transform import Rotation
import queue
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from Log.Log import RadarLog
from Tools.Paths import project_path, resolve_project_path

# ============ 标定参数 ============
TRANSITION_VECTOR = np.array([-0.04067, -0.24053, 0.0053])
ROTATION_MATRIX = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])

YAW_OFFSET = 0.0
PITCH_OFFSET = -12.0
YAW_RADIOS = 1.15

# ============ 聚类参数 ============
EPS = 0.5
MIN_POINTS = 15
MAX_RADIUS = 2.2

# ============ 赛场范围筛选参数 ============
MIN_HEIGHT = 1.8
MAX_HEIGHT = 3.5
MIN_X = 7.0
MAX_X = 27.5
MIN_Y = 0.1
MAX_Y = 7.0

# MIN_HEIGHT = -0.5
# MAX_HEIGHT = 3.0
# MIN_X = 8.0
# MAX_X = 27.5
# MIN_Y = -1.0
# MAX_Y = 5.0

# ============ 队列参数 ============
MIN_FRAME_COUNT = 5
MAX_DEQUE_SIZE = 10

# ============ 动态目标优先参数 ============
VELOCITY_THRESHOLD = 0.3
DYNAMIC_SCORE_WEIGHT = 0.30
HISTORY_TRACKING_DISTANCE = 3.0

# ============ 背景排除参数 ============
MAP_OVERLAP_THRESHOLD = 0.6
MAP_DISTANCE_THRESHOLD = 0.15
BACKGROUND_PENALTY_WEIGHT = 0.8

# ============ 扁平度优先参数 ============
FLATNESS_WEIGHT = 0.20
MIN_FLATNESS = 1.0
FLATNESS_BONUS_DYNAMIC = 0.15

# ============ 目标锁定参数 ============
LOCK_TIMEOUT = 3.0               # 目标丢失后持续追踪的最长时间（秒）
LOCK_SCORE_MARGIN = 0.15         # 切换目标需要新目标分数超过当前目标的幅度
LOCK_MAX_STATIC_FRAMES = 150     # 静止后最多持续追踪的帧数（约5秒@30Hz）

# ============ 打表模式参数 ============
TABLE_YAW_START = 9.54
TABLE_YAW_END = 30.0
TABLE_YAW_STEP = 0.25
TABLE_PITCH_MIN = 4.23
TABLE_PITCH_MAX = 12.0
TABLE_PITCH_PERIOD = 4.0
TABLE_PUBLISH_HZ = 20.0

DRONE_MAP_PATH = str(project_path("RM2026_map.pcd"))
DRONE_LIDAR_TOPIC = "/livox/lidar"
DRONE_FRAME_ID = "world"
DRONE_ANGLE_MODE = "track"
DRONE_ENABLE_RECORDING = False
DRONE_RECORD_RAW = False
DRONE_RECORD_WORLD = False
DRONE_RECORD_FORMAT = "both"
DRONE_RECORD_DIR = None
DRONE_MAX_RECORD_QUEUE = 50


class DroneRunConfig:
    def __init__(self):
        self.map_path = DRONE_MAP_PATH
        self.lidar_topic = DRONE_LIDAR_TOPIC
        self.frame_id = DRONE_FRAME_ID
        self.angle_mode = DRONE_ANGLE_MODE.strip().lower()
        self.record_raw = DRONE_RECORD_RAW
        self.record_world = DRONE_RECORD_WORLD
        self.enable_recording = DRONE_ENABLE_RECORDING and (self.record_raw or self.record_world)
        self.record_format = DRONE_RECORD_FORMAT
        self.record_dir = DRONE_RECORD_DIR
        self.max_record_queue = DRONE_MAX_RECORD_QUEUE


class TrackedTarget:
    """被锁定的目标对象，跨帧持续追踪"""
    def __init__(self, cluster_id, center_world, cluster_points, score, flatness, velocity, timestamp):
        self.cluster_id = cluster_id
        self.center_world = center_world
        self.cluster_points = cluster_points
        self.initial_score = score          # 首次锁定的分数（历史最高分）
        self.current_score = score          # 当前帧分数
        self.flatness = flatness
        self.velocity = velocity
        self.first_seen = timestamp
        self.last_seen = timestamp
        self.lost_count = 0               # 连续丢失帧数
        self.static_count = 0             # 静止持续帧数
        self.is_active = True             # 是否仍被追踪
        self.history_positions = deque(maxlen=20)  # 位置历史，用于平滑
        
        self.history_positions.append((timestamp, center_world.copy()))

    def update(self, center_world, cluster_points, score, flatness, velocity, timestamp):
        """更新目标状态"""
        self.center_world = center_world
        self.cluster_points = cluster_points
        self.current_score = score
        self.flatness = flatness
        self.velocity = velocity
        self.last_seen = timestamp
        self.lost_count = 0
        self.history_positions.append((timestamp, center_world.copy()))
        
        # 更新静止计数
        if velocity < VELOCITY_THRESHOLD:
            self.static_count += 1
        else:
            self.static_count = 0  # 一旦动起来就重置

    def mark_lost(self):
        """标记一帧未检测到"""
        self.lost_count += 1
        if self.lost_count > 30:  # 约1秒未检测到
            self.is_active = False

    def get_smoothed_position(self):
        """获取平滑后的位置（减少抖动）"""
        if len(self.history_positions) < 3:
            return self.center_world
        
        # 指数加权平均，越新的权重越高
        positions = np.array([p[1] for p in self.history_positions])
        weights = np.exp(np.linspace(-1, 0, len(positions)))
        weights /= weights.sum()
        smoothed = np.average(positions, axis=0, weights=weights)
        return smoothed

    def should_release(self, current_time):
        """判断是否应该释放该目标"""
        # 条件1：丢失太久
        if self.lost_count > 30:
            return True, "lost too long"
        
        # 条件2：静止太久且初始分数不够高（防止低分静止目标一直占用）
        if self.static_count > LOCK_MAX_STATIC_FRAMES and self.initial_score < 0.6:
            return True, "static and low initial score"
        
        # 条件3：超时未更新
        if current_time - self.last_seen > LOCK_TIMEOUT:
            return True, "timeout"
        
        return False, "active"


class LidarTracker:
    def __init__(self, args):
        rospy.init_node('lidar_tracker', anonymous=True)
        self.args = args

        # ========== 配置加载 ==========
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = project_path("configs", "world_points.yaml")

        self.WORLD_FEATURE_POINTS = None
        if config_path.exists():
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                self.WORLD_FEATURE_POINTS = np.array(config['points'])
                rospy.loginfo(f"Loaded world feature points: {self.WORLD_FEATURE_POINTS}")
        else:
            rospy.logerr("找不到配置文件 configs/world_points.yaml，使用默认3点")
            self.WORLD_FEATURE_POINTS = np.array([
                [0, 0, 0],
                [15, 0, 0],
                [0, 28, 0]
            ], dtype=np.float64)
        self.logger = RadarLog("GimbalAngleSender")
        # ========== 参数 ==========
        self.map_path = self._resolve_map_path(args.map_path)
        self.lidar_topic = args.lidar_topic
        self.frame_id = args.frame_id
        self.angle_mode = args.angle_mode

        # ========== 录制参数 ==========
        self.enable_recording = args.enable_recording
        self.record_raw = args.record_raw
        self.record_world = args.record_world
        self.record_format = args.record_format
        self.record_dir = args.record_dir or os.path.join(script_dir, 'recordings')
        self.max_record_queue = args.max_record_queue
        
        self.raw_frame_count = 0
        self.world_frame_count = 0
        self.record_lock = threading.Lock()
        self.record_queue = queue.Queue(maxsize=self.max_record_queue)
        self.record_thread = None
        self.stop_recording = threading.Event()
        self.stop_table_mode = threading.Event()
        self.table_thread = None

        if self.enable_recording:
            self._init_recording()

        # ========== 状态机 ==========
        self.phase = "CALIBRATION"
        self.T_lidar_to_world = None
        self.calibration_done = False

        # ========== 点云数据 ==========
        self.current_lidar_pcd = None
        self.lidar_lock = threading.Lock()

        self.point_queue = deque(maxlen=MAX_DEQUE_SIZE)
        self.cloud_lock = threading.Lock()

        self.map_pcd_world = None
        self.map_kdtree = None
        self.map_points_array = None

        # ========== 目标锁定系统 ==========
        self.locked_target = None           # 当前锁定的目标
        self.target_lock = threading.Lock()
        self.next_cluster_id = 0

        # ========== 追踪状态（兼容旧接口）==========
        self.tracked_cluster = None
        self.tracked_center_world = None
        self.tracked_center_gimbal = None
        self.tracked_confidence = 0.0

        self.lidar2gimbal = np.eye(4)
        self.lidar2gimbal[:3, :3] = ROTATION_MATRIX
        self.lidar2gimbal[:3, 3] = TRANSITION_VECTOR

        # ========== 可视化数据缓冲 ==========
        self.vis_data_lock = threading.Lock()
        self.vis_all_points = None
        self.vis_tracked_cluster = None
        self.vis_has_new_data = False
        self.vis_velocity = 0.0
        self.vis_is_background = False
        self.vis_flatness = 0.0
        self.vis_is_locked = False           # 新增：是否处于锁定状态
        self.vis_static_count = 0

        self.vis_all_points_pcd = o3d.geometry.PointCloud()
        self.vis_tracked_bbox = o3d.geometry.LineSet()
        self.vis_map_pcd = None

        # ========== ROS 通信 ==========
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, PointCloud2, self.lidar_callback)
        self.yawpitch_pub = rospy.Publisher("/init_yawpitch", Float32MultiArray, queue_size=10)
        self.drone_field_xyz_pub = rospy.Publisher("/drone_field_xyz", PointCloud2, queue_size=10)

        rospy.loginfo("=" * 60)
        rospy.loginfo("LiDAR Tracker Node [目标锁定版]")
        rospy.loginfo(f"地图: {self.map_path}")
        rospy.loginfo(f"输入: {self.lidar_topic}")
        rospy.loginfo(f"角度模式: {self.angle_mode}")
        if self.enable_recording:
            rospy.loginfo(f"录制目录: {self.record_dir}")
        rospy.loginfo("追踪策略: 锁定高分目标，静止后持续追踪")
        rospy.loginfo("释放条件: 丢失1秒 / 静止5秒且低分 / 出现明显更高分目标")
        rospy.loginfo("=" * 60)

    def _resolve_map_path(self, path):
        return str(resolve_project_path(path))

    def _init_recording(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.record_dir = os.path.join(self.record_dir, f"recording_{timestamp}")
        os.makedirs(self.record_dir, exist_ok=True)
        
        if self.record_raw:
            self.raw_dir = os.path.join(self.record_dir, "raw_lidar")
            os.makedirs(self.raw_dir, exist_ok=True)
            os.makedirs(os.path.join(self.raw_dir, "pcd"), exist_ok=True)
            os.makedirs(os.path.join(self.raw_dir, "numpy"), exist_ok=True)
        
        if self.record_world:
            self.world_dir = os.path.join(self.record_dir, "world")
            os.makedirs(self.world_dir, exist_ok=True)
            os.makedirs(os.path.join(self.world_dir, "pcd"), exist_ok=True)
            os.makedirs(os.path.join(self.world_dir, "numpy"), exist_ok=True)
        
        meta = {
            'start_time': timestamp,
            'lidar_topic': self.lidar_topic,
            'record_raw': self.record_raw,
            'record_world': self.record_world,
            'format': self.record_format,
            'transition_vector': TRANSITION_VECTOR.tolist(),
            'rotation_matrix': ROTATION_MATRIX.tolist(),
            'flatness_weight': FLATNESS_WEIGHT,
            'min_flatness': MIN_FLATNESS,
            'lock_timeout': LOCK_TIMEOUT,
            'lock_score_margin': LOCK_SCORE_MARGIN,
        }
        meta_path = os.path.join(self.record_dir, "metadata.yaml")
        with open(meta_path, 'w') as f:
            yaml.dump(meta, f, default_flow_style=False, allow_unicode=True)
        
        rospy.loginfo(f"录制目录创建: {self.record_dir}")

    def _record_worker(self):
        rospy.loginfo("录制线程启动...")
        
        while not self.stop_recording.is_set() or not self.record_queue.empty():
            try:
                record_item = self.record_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            
            frame_type = record_item['type']
            points = record_item['points']
            timestamp = record_item['timestamp']
            frame_num = record_item['frame_num']
            
            time_str = f"{timestamp:.6f}"
            base_name = f"frame_{frame_num:06d}_t{time_str}"
            
            try:
                if frame_type == 'raw' and self.record_raw:
                    if self.record_format in ('pcd', 'both'):
                        pcd_path = os.path.join(self.raw_dir, "pcd", f"{base_name}.pcd")
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(points)
                        o3d.io.write_point_cloud(pcd_path, pcd)
                    
                    if self.record_format in ('numpy', 'both'):
                        npy_path = os.path.join(self.raw_dir, "numpy", f"{base_name}.npy")
                        np.save(npy_path, points)
                
                elif frame_type == 'world' and self.record_world:
                    if self.record_format in ('pcd', 'both'):
                        pcd_path = os.path.join(self.world_dir, "pcd", f"{base_name}.pcd")
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(points)
                        o3d.io.write_point_cloud(pcd_path, pcd)
                    
                    if self.record_format in ('numpy', 'both'):
                        npy_path = os.path.join(self.world_dir, "numpy", f"{base_name}.npy")
                        np.save(npy_path, points)
                
                if frame_num % 100 == 0:
                    rospy.loginfo_throttle(5.0, f"已录制 {frame_num} 帧 ({frame_type})")
                    
            except Exception as e:
                rospy.logerr_throttle(5.0, f"录制保存失败 frame {frame_num}: {e}")
        
        rospy.loginfo("录制线程结束")

    def _enqueue_frame(self, frame_type, points, timestamp):
        if not self.enable_recording:
            return
        
        with self.record_lock:
            if frame_type == 'raw':
                self.raw_frame_count += 1
                frame_num = self.raw_frame_count
            else:
                self.world_frame_count += 1
                frame_num = self.world_frame_count
        
        record_item = {
            'type': frame_type,
            'points': points.copy(),
            'timestamp': timestamp,
            'frame_num': frame_num
        }
        
        try:
            self.record_queue.put(record_item, block=False)
        except queue.Full:
            try:
                self.record_queue.get_nowait()
                self.record_queue.put(record_item, block=False)
            except queue.Empty:
                pass

    def msg_to_o3d(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
        return pcd

    def lidar_callback(self, msg):
        try:
            pcd = self.msg_to_o3d(msg)
            
            if self.enable_recording and self.record_raw:
                raw_points = np.asarray(pcd.points, dtype=np.float32)
                valid_mask = (np.linalg.norm(raw_points, axis=1) > 0.01) & np.all(np.isfinite(raw_points), axis=1)
                raw_points_valid = raw_points[valid_mask]
                if len(raw_points_valid) > 0:
                    self._enqueue_frame('raw', raw_points_valid, msg.header.stamp.to_sec())
            
            with self.lidar_lock:
                self.current_lidar_pcd = pcd

            if self.angle_mode != "table" and self.phase == "TRACKING" and self.T_lidar_to_world is not None:
                xyz_lidar = np.asarray(pcd.points, dtype=np.float32)
                valid_mask = (np.linalg.norm(xyz_lidar, axis=1) > 0.01) & np.all(np.isfinite(xyz_lidar), axis=1)
                xyz_lidar = xyz_lidar[valid_mask]

                if len(xyz_lidar) == 0:
                    return

                xyz_world = self.transform_lidar_to_world(xyz_lidar, self.T_lidar_to_world)
                
                if self.enable_recording and self.record_world:
                    self._enqueue_frame('world', xyz_world, msg.header.stamp.to_sec())

                with self.cloud_lock:
                    self.point_queue.append(xyz_world)

                if len(self.point_queue) >= MIN_FRAME_COUNT:
                    self.process_clusters()

        except Exception as e:
            rospy.logerr_throttle(5.0, f"LiDAR callback error: {e}")

    def transform_lidar_to_world(self, points_lidar, T):
        ones = np.ones((points_lidar.shape[0], 1))
        points_homo = np.hstack([points_lidar, ones])
        points_world_homo = (T @ points_homo.T).T
        return points_world_homo[:, :3]

    def transform_points_lidar_to_gimbal(self, points_lidar):
        ones = np.ones((points_lidar.shape[0], 1))
        points_homo = np.hstack([points_lidar, ones])
        points_gimbal_homo = (self.lidar2gimbal @ points_homo.T).T
        return points_gimbal_homo[:, :3]

    # ==================== 标定相关 ====================

    def load_map(self):
        rospy.loginfo(f"加载地图: {self.map_path}")
        if not os.path.exists(self.map_path):
            rospy.logerr(f"地图文件不存在: {self.map_path}")
            return None

        try:
            map_pcd = o3d.io.read_point_cloud(self.map_path)
            if len(map_pcd.points) == 0:
                rospy.logerr("地图为空！")
                return None

            pts = np.asarray(map_pcd.points)
            rospy.loginfo(f"地图点数: {len(map_pcd.points)}")
            rospy.loginfo(f"地图范围: X=[{pts[:,0].min():.2f},{pts[:,0].max():.2f}], "
                         f"Y=[{pts[:,1].min():.2f},{pts[:,1].max():.2f}], "
                         f"Z=[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]")

            max_range = np.max(np.abs(pts))
            if max_range > 1000:
                rospy.logwarn(f"坐标范围大({max_range:.1f})，转换为米")
                map_pcd.points = o3d.utility.Vector3dVector(pts / 1000.0)
                pts = np.asarray(map_pcd.points)

            map_pcd_down = map_pcd.voxel_down_sample(voxel_size=0.03)
            rospy.loginfo(f"降采样后: {len(map_pcd_down.points)}")

            self.map_pcd_world = map_pcd_down
            self.map_points_array = np.asarray(map_pcd_down.points)
            self.map_kdtree = o3d.geometry.KDTreeFlann(map_pcd_down)
            rospy.loginfo(f"地图KD-Tree构建完成: {len(self.map_points_array)} 点")

            return map_pcd_down

        except Exception as e:
            rospy.logerr(f"加载失败: {e}")
            return None

    @staticmethod
    def kabsch_algorithm(P, Q):
        assert P.shape == Q.shape and P.shape[0] >= 3
        centroid_P = np.mean(P, axis=0)
        centroid_Q = np.mean(Q, axis=0)
        P_centered = P - centroid_P
        Q_centered = Q - centroid_Q
        H = P_centered.T @ Q_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = centroid_Q - R @ centroid_P
        error = np.mean(np.linalg.norm(Q - ((R @ P.T).T + t), axis=1))
        return R, t, error

    def build_transform_matrix(self, R, t):
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    @staticmethod
    def color_by_height(points, z_min=None, z_max=None):
        z = points[:, 2]
        if z_min is None:
            z_min = np.min(z)
        if z_max is None:
            z_max = np.max(z)
        if z_max - z_min < 0.001:
            t = np.zeros_like(z)
        else:
            t = np.clip((z - z_min) / (z_max - z_min), 0, 1)
        colors = np.zeros((len(points), 3))
        for i, ti in enumerate(t):
            if ti < 0.25:
                colors[i] = [0, ti * 4, 1]
            elif ti < 0.5:
                colors[i] = [0, 1, 1 - (ti - 0.25) * 4]
            elif ti < 0.75:
                colors[i] = [(ti - 0.5) * 4, 1, 0]
            else:
                colors[i] = [1, 1 - (ti - 0.75) * 4, 0]
        return colors

    def perform_calibration(self):
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("阶段1: 手动标定")
        rospy.loginfo("=" * 60)

        map_pcd = self.load_map()
        if map_pcd is None:
            return False

        rospy.loginfo("等待收集多帧LiDAR数据用于标定...")
        calibration_frames = 10
        frame_count = 0
        collected_points = []
        
        wait_start = time.time()
        timeout = 30.0
        
        while frame_count < calibration_frames and not rospy.is_shutdown():
            if time.time() - wait_start > timeout:
                rospy.logerr(f"收集点云超时，只收集到{frame_count}帧")
                break
            
            with self.lidar_lock:
                has_data = self.current_lidar_pcd is not None
            
            if not has_data:
                rospy.sleep(0.1)
                continue
            
            with self.lidar_lock:
                pcd = self.current_lidar_pcd
            
            xyz = np.asarray(pcd.points, dtype=np.float32)
            valid_mask = (np.linalg.norm(xyz, axis=1) > 0.01) & np.all(np.isfinite(xyz), axis=1)
            xyz = xyz[valid_mask]
            
            if len(xyz) > 0:
                collected_points.append(xyz)
                frame_count += 1
                rospy.loginfo(f"收集第 {frame_count}/{calibration_frames} 帧: {len(xyz)} 点")
            
            rospy.sleep(0.2)
        
        if len(collected_points) == 0:
            rospy.logerr("没有收集到任何点云数据！")
            return False
        
        merged_points = np.vstack(collected_points)
        rospy.loginfo(f"合并完成: {len(collected_points)} 帧, 共 {len(merged_points)} 点")
        
        merged_pcd = o3d.geometry.PointCloud()
        merged_pcd.points = o3d.utility.Vector3dVector(merged_points)
        
        if len(merged_points) > 50000:
            rospy.loginfo(f"合并点云 {len(merged_points)} 点，降采样用于显示...")
            voxel_size = 0.05
            while True:
                display_pcd = merged_pcd.voxel_down_sample(voxel_size=voxel_size)
                if len(display_pcd.points) <= 50000 or voxel_size > 1.0:
                    break
                voxel_size += 0.05
            rospy.loginfo(f"降采样后: {len(display_pcd.points)} 点 (voxel={voxel_size:.2f}m)")
        else:
            display_pcd = merged_pcd

        rospy.loginfo("创建标定窗口...")
        vis_edit = o3d.visualization.VisualizerWithEditing()
        vis_edit.create_window(
            window_name="【标定】在合并点云上选3个特征点 (Shift+左键, Q结束) - 高度着色:蓝低->红高",
            width=1400,
            height=900
        )

        display_pts = np.asarray(display_pcd.points)
        colors = self.color_by_height(display_pts)
        display_pcd.colors = o3d.utility.Vector3dVector(colors)
        vis_edit.add_geometry(display_pcd)
        vis_edit.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))

        opt = vis_edit.get_render_option()
        opt.point_size = 3.0
        opt.background_color = np.array([0.1, 0.1, 0.1])
        opt.show_coordinate_frame = True

        ctr = vis_edit.get_view_control()
        ctr.set_zoom(0.5)
        ctr.set_lookat([0, 0, 2])

        z_min = np.min(display_pts[:, 2])
        z_max = np.max(display_pts[:, 2])
        print(f"\n{'='*50}")
        print("【标定操作说明】")
        print(f"  当前显示: 合并 {len(collected_points)} 帧雷达点云（共 {len(display_pcd.points)} 点）")
        print("  颜色说明: 蓝色=低高度, 青色, 绿色, 黄色, 红色=高高度")
        print(f"  高度范围: Z=[{z_min:.2f}, {z_max:.2f}]")
        print("  操作: 按住 Shift + 左键点击点云选取特征点")
        print("  结束: 按 'Q' 完成选取")
        print(f"  目标：选取 {len(self.WORLD_FEATURE_POINTS)} 个特征点")
        print("  注意：必须按照与实际地图坐标相同的顺序选取！")
        print(f"{'='*50}\n")

        rospy.loginfo("等待用户选点...")
        vis_edit.run()

        picked_indices = vis_edit.get_picked_points()
        vis_edit.destroy_window()

        rospy.loginfo(f"选中了 {len(picked_indices)} 个点")

        if len(picked_indices) != 3:
            rospy.logerr(f"需要3个点，实际选了{len(picked_indices)}个")
            return False

        lidar_points = np.asarray(display_pcd.points)[picked_indices]
        rospy.loginfo("LiDAR特征点:")
        for i, p in enumerate(lidar_points):
            rospy.loginfo(f"  点{i+1}: [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]")

        R, t, error = self.kabsch_algorithm(lidar_points, self.WORLD_FEATURE_POINTS)
        self.T_lidar_to_world = self.build_transform_matrix(R, t)

        rospy.loginfo("\n--- 标定结果 ---")
        rospy.loginfo(f"旋转矩阵 R:\n{R}")
        rospy.loginfo(f"平移向量 t: {t}")
        rospy.loginfo(f"配准误差: {error:.6f} m")
        rospy.loginfo(f"四元数: {Rotation.from_matrix(R).as_quat()}")

        lidar_transformed = (R @ lidar_points.T).T + t
        rospy.loginfo("\n验证：")
        for i, (orig, trans, world) in enumerate(zip(lidar_points, lidar_transformed, self.WORLD_FEATURE_POINTS)):
            rospy.loginfo(f"  点{i+1}: [{orig[0]:.3f},{orig[1]:.3f},{orig[2]:.3f}] → "
                         f"[{trans[0]:.3f},{trans[1]:.3f},{trans[2]:.3f}] → "
                         f"[{world[0]:.3f},{world[1]:.3f},{world[2]:.3f}]")

        self.calibration_done = True
        self.phase = "TRACKING"
        rospy.loginfo("\n标定完成！进入追踪阶段...")
        rospy.loginfo("=" * 60)

        return True

    # ==================== 背景排除核心方法 ====================

    def check_cluster_background_overlap(self, cluster_points_world):
        if self.map_kdtree is None or len(cluster_points_world) == 0:
            return 0.0, False

        if len(cluster_points_world) > 500:
            pcd_temp = o3d.geometry.PointCloud()
            pcd_temp.points = o3d.utility.Vector3dVector(cluster_points_world)
            pcd_temp = pcd_temp.voxel_down_sample(voxel_size=0.05)
            query_points = np.asarray(pcd_temp.points)
        else:
            query_points = cluster_points_world

        map_near_count = 0
        for pt in query_points:
            [k, idx, dist] = self.map_kdtree.search_radius_vector_3d(pt, MAP_DISTANCE_THRESHOLD)
            if k > 0:
                map_near_count += 1

        overlap_ratio = map_near_count / len(query_points) if len(query_points) > 0 else 0.0
        is_background = overlap_ratio > MAP_OVERLAP_THRESHOLD
        return overlap_ratio, is_background

    def remove_background_points(self, points_world):
        if self.map_kdtree is None:
            return points_world

        foreground_mask = np.ones(len(points_world), dtype=bool)
        batch_size = 100
        for i in range(0, len(points_world), batch_size):
            batch = points_world[i:i+batch_size]
            for j, pt in enumerate(batch):
                [k, idx, dist] = self.map_kdtree.search_radius_vector_3d(pt, MAP_DISTANCE_THRESHOLD)
                if k > 3:
                    foreground_mask[i + j] = False
        return points_world[foreground_mask]

    # ==================== 扁平度计算核心方法 ====================

    def calculate_flatness(self, cluster_points):
        if len(cluster_points) < 3:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        
        x_min, x_max = np.min(cluster_points[:, 0]), np.max(cluster_points[:, 0])
        y_min, y_max = np.min(cluster_points[:, 1]), np.max(cluster_points[:, 1])
        z_min, z_max = np.min(cluster_points[:, 2]), np.max(cluster_points[:, 2])
        
        x_span = x_max - x_min
        y_span = y_max - y_min
        z_span = z_max - z_min
        
        epsilon = 0.01
        z_span_safe = max(z_span, epsilon)
        
        xy_area = x_span * y_span
        flatness_ratio = xy_area / z_span_safe
        
        flatness_score = np.tanh(flatness_ratio / 5.0)
        
        return flatness_score, flatness_ratio, x_span, y_span, z_span

    # ==================== 目标锁定核心逻辑 ====================

    def find_matching_cluster(self, candidates, locked_target, current_time):
        """
        在候选聚类中寻找与锁定目标匹配的那个
        返回: (matched_candidate_index, distance) 或 (None, inf)
        """
        if locked_target is None or not locked_target.is_active:
            return None, float('inf')
        
        best_idx = None
        best_dist = HISTORY_TRACKING_DISTANCE  # 最大关联距离
        
        for i, cand in enumerate(candidates):
            dist = np.linalg.norm(cand['center'] - locked_target.center_world)
            if dist < best_dist:
                # 额外检查：高度差异不能太大（避免跨楼层误关联）
                height_diff = abs(cand['center'][2] - locked_target.center_world[2])
                if height_diff < 1.0:  # 1米内高度变化允许
                    best_dist = dist
                    best_idx = i
        
        return best_idx, best_dist

    def should_switch_target(self, locked_target, best_candidate, current_time):
        """
        判断是否应该从当前锁定目标切换到新的高分目标
        返回: (should_switch, reason)
        """
        if locked_target is None or not locked_target.is_active:
            return True, "no active lock"
        
        # 当前锁定目标的当前分数（考虑静止衰减）
        locked_current_score = locked_target.current_score
        
        # 新候选目标的分数
        new_score = best_candidate['score']
        
        # 条件1：新目标分数显著更高
        if new_score > locked_current_score + LOCK_SCORE_MARGIN:
            # 额外检查：新目标必须也是动态或扁平的（避免切换到噪声）
            if best_candidate['velocity'] > VELOCITY_THRESHOLD or best_candidate['flatness_ratio'] > MIN_FLATNESS:
                return True, f"new score {new_score:.2f} >> locked {locked_current_score:.2f}"
        
        # 条件2：当前目标丢失太久
        if locked_target.lost_count > 10:
            # 只有新目标分数不太差才切换
            if new_score > 0.3:
                return True, "locked target lost"
        
        # 条件3：当前目标静止太久且初始分不高
        if locked_target.static_count > LOCK_MAX_STATIC_FRAMES and locked_target.initial_score < 0.6:
            if new_score > locked_current_score:
                return True, "locked target static and low score"
        
        return False, "keep locked"

    # ==================== 评分函数 ====================

    def calculate_score(self, point_count, height, radius, center, velocity, consistency,
                        overlap_ratio, is_background, flatness_score, flatness_ratio):
        # 1. 动态分数 (30%)
        velocity_score = min(velocity / 3.0, 1.0)
        consistency_bonus = consistency * 0.3
        dynamic_score = (velocity_score + consistency_bonus) * 0.30
        
        # 2. 扁平度分数 (20%)
        flatness_base = flatness_score * 0.15
        flatness_bonus = 0.0
        if velocity > 0.5 and flatness_ratio > 5.0:
            flatness_bonus = FLATNESS_BONUS_DYNAMIC
        flatness_total = min(flatness_base + flatness_bonus, 0.20)
        
        # 3. 点数密度 (15%)
        point_score = min(point_count / 50.0, 1.0) * 0.15
        
        # 4. 高度匹配 (15%)
        height_score = np.exp(-((height - 2.5) ** 2) / 0.5) * 0.15
        
        # 5. 半径匹配 (10%)
        radius_score = np.exp(-((radius - 0.5) ** 2) / 0.2) * 0.10
        
        # 6. 距离分数 (10%)
        distance = np.linalg.norm(center)
        distance_score = max(0, 1.0 - distance / 50.0) * 0.10
        
        # 7. 背景惩罚
        background_penalty = 1.0 - (overlap_ratio * BACKGROUND_PENALTY_WEIGHT)
        background_penalty = max(0.1, background_penalty)
        
        if velocity > 1.0:
            background_penalty = 1.0 - (overlap_ratio * BACKGROUND_PENALTY_WEIGHT * 0.3)
            background_penalty = max(0.5, background_penalty)
        
        base_score = dynamic_score + flatness_total + point_score + height_score + radius_score + distance_score
        total_score = base_score * background_penalty
        
        breakdown = (f"dyn={dynamic_score:.2f}, flat={flatness_total:.2f}(r={flatness_ratio:.1f}), "
                    f"pts={point_score:.2f}, h={height_score:.2f}, "
                    f"r={radius_score:.2f}, dist={distance_score:.2f}, "
                    f"bg={background_penalty:.2f}")
        
        return total_score, breakdown

    # ==================== 主处理流程（目标锁定版）====================

    def process_clusters(self):
        """目标锁定版聚类处理"""
        try:
            with self.cloud_lock:
                all_points_world = np.vstack(list(self.point_queue))

            rospy.loginfo(f"Processing {len(all_points_world)} points from {len(self.point_queue)} frames")

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(all_points_world)
            pcd_down = pcd.voxel_down_sample(voxel_size=0.05)
            down_points = np.asarray(pcd_down.points)

            if len(down_points) < MIN_POINTS:
                rospy.logwarn("Too few points after downsampling")
                self._update_visualization_empty()
                return

            labels = np.array(pcd_down.cluster_dbscan(eps=EPS, min_points=MIN_POINTS))
            unique_labels = np.unique(labels)

            rospy.loginfo(f"Found {len(unique_labels) - (1 if -1 in unique_labels else 0)} clusters")

            current_time = time.time()
            candidates = []  # 所有通过筛选的候选聚类

            # ========== 第一步：过滤背景 + 坐标筛选 + 计算分数 ==========
            for label in unique_labels:
                if label == -1:
                    continue

                indices = np.where(labels == label)[0]
                cluster_points_world = down_points[indices]

                if len(cluster_points_world) < MIN_POINTS:
                    continue

                center_world = np.mean(cluster_points_world, axis=0)
                x_mean, y_mean, z_mean = center_world

                # 坐标范围硬性过滤
                if not (MIN_X <= x_mean <= MAX_X and MIN_Y <= y_mean <= MAX_Y and 
                        MIN_HEIGHT <= z_mean <= MAX_HEIGHT):
                    continue

                distances = np.linalg.norm(cluster_points_world - center_world, axis=1)
                max_radius = np.max(distances)
                if max_radius > MAX_RADIUS:
                    continue

                # 扁平度筛选
                flatness_score, flatness_ratio, x_span, y_span, z_span = self.calculate_flatness(cluster_points_world)
                if flatness_ratio < MIN_FLATNESS:
                    continue

                # 背景排除
                overlap_ratio, is_background = self.check_cluster_background_overlap(cluster_points_world)
                # 动态目标可豁免背景判定
                velocity_mag = 0.0  # 临时，后面会重新计算
                dynamic_override = False  # 临时

                if is_background:
                    # 先计算速度，再判断是否豁免
                    pass  # 速度在后面统一计算

                # 计算分数
                score, breakdown = self.calculate_score(
                    point_count=len(cluster_points_world),
                    height=z_mean,
                    radius=max_radius,
                    center=center_world,
                    velocity=0.0,  # 临时，后面更新
                    consistency=0.0,
                    overlap_ratio=overlap_ratio,
                    is_background=is_background,
                    flatness_score=flatness_score,
                    flatness_ratio=flatness_ratio
                )

                candidates.append({
                    'label': label,
                    'points': cluster_points_world,
                    'center': center_world,
                    'score': score,
                    'flatness_score': flatness_score,
                    'flatness_ratio': flatness_ratio,
                    'max_radius': max_radius,
                    'overlap_ratio': overlap_ratio,
                    'is_background': is_background,
                    'velocity': 0.0,  # 后面更新
                    'breakdown': breakdown
                })

            # ========== 第二步：计算各候选的速度（需要历史数据）==========
            with self.target_lock:
                locked = self.locked_target

            for cand in candidates:
                # 如果有锁定目标，计算相对于锁定目标的速度
                # 否则计算相对于上一帧最近邻的速度
                if locked is not None and locked.is_active:
                    dist = np.linalg.norm(cand['center'] - locked.center_world)
                    if dist < HISTORY_TRACKING_DISTANCE:
                        dt = current_time - locked.last_seen
                        if dt > 0.001:
                            velocity = np.linalg.norm((cand['center'] - locked.center_world)[:2]) / dt
                            cand['velocity'] = velocity
                        else:
                            cand['velocity'] = locked.velocity
                    else:
                        # 新目标，速度未知，给一个中等估计
                        cand['velocity'] = 0.5
                else:
                    cand['velocity'] = 0.5  # 无历史时默认中等速度

                # 重新计算分数（含速度）
                score, breakdown = self.calculate_score(
                    point_count=len(cand['points']),
                    height=cand['center'][2],
                    radius=cand['max_radius'],
                    center=cand['center'],
                    velocity=cand['velocity'],
                    consistency=0.0,
                    overlap_ratio=cand['overlap_ratio'],
                    is_background=cand['is_background'],
                    flatness_score=cand['flatness_score'],
                    flatness_ratio=cand['flatness_ratio']
                )
                cand['score'] = score
                cand['breakdown'] = breakdown

            # ========== 第三步：目标锁定逻辑 ==========
            with self.target_lock:
                locked = self.locked_target

            # 如果有锁定目标，先尝试匹配
            matched_idx = None
            if locked is not None and locked.is_active:
                matched_idx, match_dist = self.find_matching_cluster(candidates, locked, current_time)
                
                if matched_idx is not None:
                    # 匹配成功：更新锁定目标
                    cand = candidates[matched_idx]
                    locked.update(
                        center_world=cand['center'],
                        cluster_points=cand['points'],
                        score=cand['score'],
                        flatness=cand['flatness_ratio'],
                        velocity=cand['velocity'],
                        timestamp=current_time
                    )
                    rospy.loginfo(f"LOCKED target updated: score={cand['score']:.2f}, "
                                f"vel={cand['velocity']:.2f}, flat={cand['flatness_ratio']:.1f}, "
                                f"static_count={locked.static_count}")
                else:
                    # 未匹配到：标记丢失一帧
                    locked.mark_lost()
                    rospy.loginfo(f"LOCKED target lost frame {locked.lost_count}, "
                                f"last_pos=[{locked.center_world[0]:.1f},{locked.center_world[1]:.1f}]")
                    
                    # 检查是否应该释放
                    should_release, reason = locked.should_release(current_time)
                    if should_release:
                        rospy.loginfo(f"RELEASE locked target: {reason}")
                        self.locked_target = None
                        locked = None

            # 如果没有锁定目标（或被释放了），选择最优候选
            if (locked is None or not locked.is_active) and len(candidates) > 0:
                # 按分数排序
                candidates.sort(key=lambda x: x['score'], reverse=True)
                best = candidates[0]
                
                # 创建新锁定目标
                self.next_cluster_id += 1
                new_target = TrackedTarget(
                    cluster_id=self.next_cluster_id,
                    center_world=best['center'],
                    cluster_points=best['points'],
                    score=best['score'],
                    flatness=best['flatness_ratio'],
                    velocity=best['velocity'],
                    timestamp=current_time
                )
                self.locked_target = new_target
                locked = new_target
                
                rospy.loginfo(f"NEW LOCK: id={new_target.cluster_id}, score={best['score']:.2f}, "
                            f"pos=[{best['center'][0]:.1f},{best['center'][1]:.1f},{best['center'][2]:.1f}], "
                            f"flat={best['flatness_ratio']:.1f}, vel={best['velocity']:.2f}")

            # 如果有锁定目标，检查是否应该切换到更高分的新目标
            elif locked is not None and locked.is_active and len(candidates) > 0:
                # 找除匹配项外的最高分候选
                candidates.sort(key=lambda x: x['score'], reverse=True)
                best_new = candidates[0]
                
                should_switch, reason = self.should_switch_target(locked, best_new, current_time)
                if should_switch:
                    rospy.loginfo(f"SWITCH: {reason}")
                    self.next_cluster_id += 1
                    new_target = TrackedTarget(
                        cluster_id=self.next_cluster_id,
                        center_world=best_new['center'],
                        cluster_points=best_new['points'],
                        score=best_new['score'],
                        flatness=best_new['flatness_ratio'],
                        velocity=best_new['velocity'],
                        timestamp=current_time
                    )
                    self.locked_target = new_target
                    locked = new_target

            # ========== 第四步：发布追踪结果 ==========
            with self.target_lock:
                locked = self.locked_target

            msg = Float32MultiArray()

            if locked is not None and locked.is_active:
                # 使用平滑后的位置
                smooth_center = locked.get_smoothed_position()
                
                self.tracked_cluster = locked.cluster_points
                self.tracked_center_world = smooth_center

                T_inv = np.linalg.inv(self.T_lidar_to_world)
                center_lidar_homo = T_inv @ np.array([smooth_center[0], smooth_center[1], smooth_center[2], 1.0])
                center_lidar = center_lidar_homo[:3]
                center_gimbal = self.transform_points_lidar_to_gimbal(center_lidar.reshape(1, -1))[0]
                self.tracked_center_gimbal = center_gimbal

                yaw, pitch = self.calculate_yaw_pitch(center_gimbal)
                yaw_deg = np.degrees(yaw) + YAW_OFFSET
                pitch_deg = -(np.degrees(pitch)) + PITCH_OFFSET
                msg.data = [yaw_deg, pitch_deg]
                self.yawpitch_pub.publish(msg)
                self.logger.log(f"Publishing gimbal angles: yaw={yaw_deg:.1f}°, pitch={pitch_deg:.1f}°")

                status = "LOCKED" if locked.static_count < 10 else "LOCKED-STATIC"
                rospy.loginfo(f"{status}: id={locked.cluster_id}, "
                            f"world=[{smooth_center[0]:.2f},{smooth_center[1]:.2f},{smooth_center[2]:.2f}], "
                            f"init_score={locked.initial_score:.2f}, cur_score={locked.current_score:.2f}, "
                            f"static={locked.static_count}, lost={locked.lost_count}, "
                            f"yaw={yaw_deg:.1f}°, pitch={pitch_deg:.1f}°")

                self._publish_drone_field_xyz(smooth_center)
            else:
                rospy.logwarn("No locked target")
                msg.data = [0.0, 0.0]
                self.yawpitch_pub.publish(msg)

            # ========== 更新可视化 ==========
            foreground_points = self.remove_background_points(all_points_world.copy())
            
            with self.vis_data_lock:
                self.vis_all_points = foreground_points
                if locked is not None and locked.is_active:
                    self.vis_tracked_cluster = locked.cluster_points.copy()
                    self.vis_velocity = locked.velocity
                    self.vis_is_background = False
                    self.vis_flatness = locked.flatness
                    self.vis_is_locked = True
                    self.vis_static_count = locked.static_count
                else:
                    self.vis_tracked_cluster = None
                    self.vis_velocity = 0.0
                    self.vis_is_background = False
                    self.vis_flatness = 0.0
                    self.vis_is_locked = False
                    self.vis_static_count = 0
                self.vis_has_new_data = True

        except Exception as e:
            rospy.logerr(f"Process error: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def _update_visualization_empty(self):
        """更新可视化（无数据时）"""
        with self.vis_data_lock:
            self.vis_all_points = None
            self.vis_tracked_cluster = None
            self.vis_has_new_data = True
            self.vis_velocity = 0.0
            self.vis_is_background = False
            self.vis_flatness = 0.0
            self.vis_is_locked = False
            self.vis_static_count = 0

    def _get_table_yaw_values(self):
        yaw_values = list(np.arange(TABLE_YAW_START, TABLE_YAW_END, TABLE_YAW_STEP))
        if not yaw_values or abs(yaw_values[-1] - TABLE_YAW_END) > 1e-6:
            yaw_values.append(TABLE_YAW_END)
        return yaw_values

    def _table_angle_sender_thread(self):
        yaw_values = self._get_table_yaw_values()
        pitch_mid = (TABLE_PITCH_MIN + TABLE_PITCH_MAX) / 2.0
        pitch_amp = (TABLE_PITCH_MAX - TABLE_PITCH_MIN) / 2.0
        period = TABLE_PITCH_PERIOD
        rate = rospy.Rate(TABLE_PUBLISH_HZ)
        start_time = time.time()
        yaw_idx = 0
        yaw_direction = 1

        rospy.loginfo("打表模式已启动: yaw线扫, pitch正弦扫")

        while not rospy.is_shutdown() and not self.stop_table_mode.is_set():
            now = time.time()
            elapsed = now - start_time
            yaw_deg = yaw_values[yaw_idx]
            pitch_deg = pitch_mid + pitch_amp * np.sin(2.0 * np.pi * elapsed / period)

            msg = Float32MultiArray()
            msg.data = [float(yaw_deg), float(pitch_deg)]
            self.yawpitch_pub.publish(msg)
            self.logger.log(f"Table angles: yaw={yaw_deg:.2f}°, pitch={pitch_deg:.2f}°")

            if yaw_idx == len(yaw_values) - 1:
                yaw_direction = -1
            elif yaw_idx == 0:
                yaw_direction = 1

            yaw_idx += yaw_direction
            rate.sleep()

    def calculate_yaw_pitch(self, center_gimbal):
        x, y, z = center_gimbal
        distance_xy = np.sqrt(x**2 + y**2)
        if distance_xy < 0.001:
            return 0.0, 0.0
        yaw = -(np.arctan2(y, x))
        pitch = np.arctan2(z, distance_xy)
        return yaw, pitch

    def _publish_drone_field_xyz(self, center_world):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('timestamp', 12, PointField.FLOAT32, 1),
        ]

        t = rospy.Time.now().to_sec()
        point_data = struct.pack('ffff', float(center_world[0]), float(center_world[1]),
                                  float(center_world[2]), float(t))

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = 1
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16
        msg.is_dense = True
        msg.data = point_data

        self.drone_field_xyz_pub.publish(msg)
        rospy.loginfo_throttle(2.0, f"Published /drone_field_xyz: [{center_world[0]:.3f}, {center_world[1]:.3f}, {center_world[2]:.3f}]")

    def create_bounding_box(self, points):
        min_bound = np.min(points, axis=0)
        max_bound = np.max(points, axis=0)
        padding = 0.05
        min_bound -= padding
        max_bound += padding

        vertices = np.array([
            [min_bound[0], min_bound[1], min_bound[2]],
            [max_bound[0], min_bound[1], min_bound[2]],
            [max_bound[0], max_bound[1], min_bound[2]],
            [min_bound[0], max_bound[1], min_bound[2]],
            [min_bound[0], min_bound[1], max_bound[2]],
            [max_bound[0], min_bound[1], max_bound[2]],
            [max_bound[0], max_bound[1], max_bound[2]],
            [min_bound[0], max_bound[1], max_bound[2]],
        ])

        lines = np.array([
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ])

        bbox = o3d.geometry.LineSet()
        bbox.points = o3d.utility.Vector3dVector(vertices)
        bbox.lines = o3d.utility.Vector2iVector(lines)
        return bbox

    # ==================== 追踪阶段可视化线程 ====================

    def _tracking_visualization_thread(self):
        rospy.loginfo("追踪可视化线程启动...")

        wait_start = time.time()
        while not rospy.is_shutdown():
            with self.vis_data_lock:
                has_data = self.vis_all_points is not None
            if has_data:
                break
            if time.time() - wait_start > 60.0:
                rospy.logwarn("等待追踪数据超时")
                break
            time.sleep(0.1)

        vis = None
        frame_count = 0

        try:
            rospy.loginfo("创建追踪可视化窗口...")
            vis = o3d.visualization.Visualizer()
            vis.create_window(
                window_name="LiDAR Tracking - Target Lock Mode",
                width=1400,
                height=900
            )

            self.vis_all_points_pcd = o3d.geometry.PointCloud()
            self.vis_tracked_bbox = o3d.geometry.LineSet()

            if self.map_pcd_world is not None:
                self.vis_map_pcd = copy.deepcopy(self.map_pcd_world)
                map_colors = np.tile([0.3, 0.5, 0.8], (len(self.map_pcd_world.points), 1))
                map_colors = map_colors * 0.5
                self.vis_map_pcd.colors = o3d.utility.Vector3dVector(map_colors)
                vis.add_geometry(self.vis_map_pcd)

            vis.add_geometry(self.vis_all_points_pcd)
            vis.add_geometry(self.vis_tracked_bbox)
            vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))

            render_option = vis.get_render_option()
            render_option.point_size = 3.0
            render_option.background_color = np.array([0.05, 0.05, 0.1])

            ctr = vis.get_view_control()
            ctr.set_zoom(0.6)
            ctr.set_front([-1, 0, -1])
            ctr.set_up([0, 0, 1])
            ctr.set_lookat([0, 0, 2])

            rospy.loginfo("追踪可视化窗口创建成功")

            while not rospy.is_shutdown():
                with self.vis_data_lock:
                    all_pts = self.vis_all_points.copy() if self.vis_all_points is not None else None
                    tracked_pts = self.vis_tracked_cluster.copy() if self.vis_tracked_cluster is not None else None
                    has_new = self.vis_has_new_data
                    current_velocity = self.vis_velocity
                    is_locked = self.vis_is_locked
                    static_count = self.vis_static_count
                    current_flatness = self.vis_flatness
                    self.vis_has_new_data = False

                if has_new and all_pts is not None:
                    self.vis_all_points_pcd.points = o3d.utility.Vector3dVector(all_pts)
                    self.vis_all_points_pcd.paint_uniform_color([0.0, 0.8, 0.2])
                    vis.update_geometry(self.vis_all_points_pcd)

                    if tracked_pts is not None and len(tracked_pts) > 0:
                        bbox = self.create_bounding_box(tracked_pts)
                        self.vis_tracked_bbox.points = bbox.points
                        self.vis_tracked_bbox.lines = bbox.lines
                        
                        # 目标锁定颜色编码：
                        # 亮绿色 = 锁定中且动态（最优）
                        # 橙色 = 锁定中但已静止（持续追踪）
                        # 青色 = 高扁平度动态（未锁定时的候选）
                        # 红色 = 动态但不扁
                        # 灰色 = 无锁定
                        if is_locked:
                            if static_count < 10:
                                color = [0.0, 1.0, 0.0]  # 亮绿：锁定+动态
                            else:
                                color = [1.0, 0.65, 0.0]  # 橙色：锁定+静止（持续追踪中）
                        elif current_flatness > 5.0 and current_velocity > VELOCITY_THRESHOLD:
                            color = [0.0, 1.0, 1.0]  # 青色：高扁+动态（候选）
                        elif current_velocity > VELOCITY_THRESHOLD:
                            color = [1.0, 0.0, 0.0]  # 红色：动态但不扁
                        else:
                            color = [0.5, 0.5, 0.5]  # 灰色
                            
                        self.vis_tracked_bbox.colors = o3d.utility.Vector3dVector([color] * len(bbox.lines))
                    else:
                        self.vis_tracked_bbox.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
                        self.vis_tracked_bbox.lines = o3d.utility.Vector2iVector(np.empty((0, 2)))

                    vis.update_geometry(self.vis_tracked_bbox)

                vis.poll_events()
                vis.update_renderer()

                frame_count += 1
                if frame_count % 100 == 0:
                    rospy.loginfo_throttle(5.0, f"追踪可视化运行中... (frames: {frame_count})")

                time.sleep(0.03)

        except Exception as e:
            rospy.logerr(f"追踪可视化线程错误: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
        finally:
            if vis is not None:
                try:
                    vis.destroy_window()
                except:
                    pass
            rospy.loginfo("追踪可视化线程结束")

    # ==================== 主运行流程 ====================

    def run(self):
        rospy.loginfo("=" * 60)
        rospy.loginfo("主线程运行")
        rospy.loginfo("=" * 60)

        if self.enable_recording:
            self.record_thread = threading.Thread(target=self._record_worker)
            self.record_thread.daemon = True
            self.record_thread.start()
            rospy.loginfo("录制线程已启动")

        if self.angle_mode == "table":
            self.table_thread = threading.Thread(target=self._table_angle_sender_thread)
            self.table_thread.daemon = True
            self.table_thread.start()
            rospy.loginfo("打表模式跳过标定与目标追踪")
            rospy.spin()
            self.stop_table_mode.set()
            if self.table_thread is not None:
                self.table_thread.join(timeout=2.0)
            if self.enable_recording:
                self.stop_recording.set()
                self.record_thread.join(timeout=5.0)
            return 0

        if not self.perform_calibration():
            rospy.logerr("标定失败，退出")
            return 1

        rospy.loginfo("启动追踪...")

        vis_thread = threading.Thread(target=self._tracking_visualization_thread)
        vis_thread.daemon = True
        vis_thread.start()
        rospy.loginfo("追踪可视化线程已启动")

        rospy.loginfo("主线程进入 rospy.spin()...")
        rospy.spin()

        if self.enable_recording:
            rospy.loginfo("停止录制...")
            self.stop_recording.set()
            self.record_thread.join(timeout=5.0)
            if self.record_thread.is_alive():
                rospy.logwarn("录制线程未正常结束")
            
            with self.record_lock:
                stats = {
                    'raw_frames': self.raw_frame_count,
                    'world_frames': self.world_frame_count,
                    'end_time': time.strftime("%Y%m%d_%H%M%S")
                }
            stats_path = os.path.join(self.record_dir, "stats.yaml")
            with open(stats_path, 'w') as f:
                yaml.dump(stats, f, default_flow_style=False)
            
            rospy.loginfo(f"录制完成: {self.record_dir}")
            rospy.loginfo(f"  原始帧: {stats['raw_frames']}")
            rospy.loginfo(f"  世界帧: {stats['world_frames']}")

        rospy.loginfo("ROS shutdown, waiting for visualization thread...")
        vis_thread.join(timeout=3.0)

        return 0


if __name__ == '__main__':
    import signal

    def signal_handler(sig, frame):
        rospy.loginfo(f"收到信号 {sig}，退出...")
        rospy.signal_shutdown("Signal received")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    args = DroneRunConfig()
    tracker = LidarTracker(args)
    sys.exit(tracker.run())
