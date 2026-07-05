import rospy
import numpy as np
import open3d as o3d
import threading
import copy
import time
import os
import sys
import signal
from scipy.spatial.transform import Rotation

# ROS消息
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, Float32MultiArray

import yaml

# ========== 配置加载 ==========
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, 'configs', 'world_points.yaml')
if not os.path.exists(CONFIG_PATH):
    CONFIG_PATH = './configs/world_points.yaml'

if not os.path.exists(CONFIG_PATH):
    rospy.logerr("找不到配置文件 configs/world_points.yaml")
    sys.exit(1)

with open(CONFIG_PATH, 'r') as f:
    config = yaml.safe_load(f)
    WORLD_FEATURE_POINTS = np.array(config['points'])


class PointPicker:
    """Open3D标定选点窗口 - 必须在主线程运行，阻塞式"""
    def __init__(self, pcd, window_name="选取特征点 (按Q结束)", timeout_sec=300):
        self.pcd = pcd
        self.window_name = window_name
        self.picked_points = []
        self.vis = o3d.visualization.VisualizerWithEditing()
        self.timeout_sec = timeout_sec
        
    def run(self):
        self.vis.create_window(
            window_name=self.window_name, 
            width=1024, 
            height=768,
            left=50,
            top=50
        )
        self.vis.add_geometry(self.pcd)
        
        opt = self.vis.get_render_option()
        opt.point_size = 3.0
        opt.background_color = np.asarray([0.1, 0.1, 0.1])
        opt.show_coordinate_frame = True
        
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        self.vis.add_geometry(mesh_frame)
        self.vis.reset_view_point(True)
        
        print(f"\n{'='*50}")
        print(f"[{self.window_name}]")
        print("操作说明：")
        print("  1. 按住 Shift + 左键点击点云选取特征点")
        print("  2. 按 'Q' 结束选取")
        print(f"  目标：选取 {len(WORLD_FEATURE_POINTS)} 个特征点")
        print("  注意：必须按照与实际地图坐标相同的顺序选取！")
        print(f"{'='*50}\n")
        
        self.vis.run()
        
        self.picked_points = self.vis.get_picked_points()
        self.vis.destroy_window()
        
        return self.picked_points


class LidarMapCalibrator:
    """
    标定器：只负责计算并发布雷达→赛场的变换矩阵
    """

    def __init__(self):
        self.map_path = self._resolve_map_path(
            rospy.get_param('~map_path', '/home/radar/Radar/code/Hust_Radar_2026/RM2026_map.pcd')
        )
        self.lidar_topic = rospy.get_param('~lidar_topic', '/livox/lidar')
        self.transform_topic = rospy.get_param('~transform_topic', '/lidar_to_world_transform')
        self.frame_id = rospy.get_param('~frame_id', 'world')
        
        self.T_lidar_to_world = None
        self.map_pcd = None
        self.map_pcd_world = None
        self.current_lidar_pcd = None
        self.calibration_done = False
        
        self.pub_transform = None
        self.sub_lidar = None
        
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._shutdown_event = threading.Event()
        
        rospy.loginfo("="*60)
        rospy.loginfo("Livox雷达-地图标定节点")
        rospy.loginfo(f"地图: {self.map_path}")
        rospy.loginfo(f"输入: {self.lidar_topic}")
        rospy.loginfo(f"输出: {self.transform_topic} (4x4变换矩阵)")
        rospy.loginfo("="*60)
    
    def _resolve_map_path(self, path):
        if os.path.isabs(path) and os.path.exists(path):
            return path
        if os.path.exists(path):
            return os.path.abspath(path)
        script_relative = os.path.join(SCRIPT_DIR, path)
        if os.path.exists(script_relative):
            return os.path.abspath(script_relative)
        try:
            pkg_name = rospy.get_param('~map_pkg', None)
            if pkg_name:
                import rospkg
                rospack = rospkg.RosPack()
                pkg_path = rospack.get_path(pkg_name)
                pkg_relative = os.path.join(pkg_path, path)
                if os.path.exists(pkg_relative):
                    return os.path.abspath(pkg_relative)
        except Exception:
            pass
        return path

    def load_map(self):
        rospy.loginfo(f"加载地图: {self.map_path}")
        if not os.path.exists(self.map_path):
            rospy.logerr(f"地图文件不存在: {self.map_path}")
            return False
        
        try:
            self.map_pcd = o3d.io.read_point_cloud(self.map_path)
            if len(self.map_pcd.points) == 0:
                rospy.logerr("地图为空！")
                return False
            
            pts = np.asarray(self.map_pcd.points)
            rospy.loginfo(f"地图点数: {len(self.map_pcd.points)}")
            rospy.loginfo(f"地图范围: X=[{pts[:,0].min():.2f},{pts[:,0].max():.2f}], "
                         f"Y=[{pts[:,1].min():.2f},{pts[:,1].max():.2f}], "
                         f"Z=[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]")
            
            max_range = np.max(np.abs(pts))
            if max_range > 1000:
                rospy.logwarn(f"坐标范围大({max_range:.1f})，转换为米")
                self.map_pcd.points = o3d.utility.Vector3dVector(pts / 1000.0)
                pts = np.asarray(self.map_pcd.points)
            
            self.map_pcd_down = self.map_pcd.voxel_down_sample(voxel_size=0.03)
            rospy.loginfo(f"降采样后: {len(self.map_pcd_down.points)}")
            self.map_pcd_world = self.map_pcd_down
            return True
        except Exception as e:
            rospy.logerr(f"加载失败: {e}")
            return False
    
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
    
    def pick_lidar_points(self):
        """在主线程执行选点"""
        rospy.loginfo("等待LiDAR数据...")
        wait_start = time.time()
        max_wait = 30.0
        
        while self.current_lidar_pcd is None and not rospy.is_shutdown():
            if time.time() - wait_start > max_wait:
                rospy.logerr("等待LiDAR数据超时！")
                return None
            rospy.sleep(0.1)
        
        if rospy.is_shutdown():
            return None
        
        rospy.loginfo("选取3个特征点...")
        display_pcd = copy.deepcopy(self.current_lidar_pcd)
        
        pts = np.asarray(display_pcd.points)
        if len(pts) > 50000:
            display_pcd = display_pcd.voxel_down_sample(voxel_size=0.05)
        
        picker = PointPicker(display_pcd, "【标定】选取3个LiDAR特征点", timeout_sec=300)
        indices = picker.run()
        
        if len(indices) != 3:
            rospy.logerr(f"需要3个点，实际{len(indices)}个")
            return None
        
        points = np.asarray(display_pcd.points)[indices]
        rospy.loginfo("LiDAR特征点:")
        for i, p in enumerate(points):
            rospy.loginfo(f"  点{i+1}: [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]")
        
        return points
    
    def perform_calibration(self):
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("开始手动标定...")
        rospy.loginfo("="*60)
        
        lidar_points = self.pick_lidar_points()
        if lidar_points is None:
            return False
        
        R, t, error = self.kabsch_algorithm(lidar_points, WORLD_FEATURE_POINTS)
        self.T_lidar_to_world = self.build_transform_matrix(R, t)
        
        rospy.loginfo("\n--- 变换矩阵 T_lidar → world ---")
        rospy.loginfo(f"旋转矩阵 R:\n{R}")
        rospy.loginfo(f"平移向量 t: {t}")
        rospy.loginfo(f"配准误差: {error:.6f} m")
        rospy.loginfo(f"四元数: {Rotation.from_matrix(R).as_quat()}")
        
        lidar_transformed = (R @ lidar_points.T).T + t
        rospy.loginfo("\n验证：")
        for i, (orig, trans, world) in enumerate(zip(lidar_points, lidar_transformed, WORLD_FEATURE_POINTS)):
            rospy.loginfo(f"  点{i+1}: [{orig[0]:.3f},{orig[1]:.3f},{orig[2]:.3f}] → "
                         f"[{trans[0]:.3f},{trans[1]:.3f},{trans[2]:.3f}] → "
                         f"[{world[0]:.3f},{world[1]:.3f},{world[2]:.3f}]")
        
        self.calibration_done = True
        rospy.loginfo("\n标定完成！")
        
        # 发布变换矩阵
        self._publish_transform()
        
        return True
    
    def _publish_transform(self):
        """发布 4x4 变换矩阵 (Float32MultiArray, 16元素按行展开)"""
        if self.T_lidar_to_world is None or self.pub_transform is None:
            return
        
        msg = Float32MultiArray()
        msg.data = self.T_lidar_to_world.flatten().astype(np.float32).tolist()
        self.pub_transform.publish(msg)
        
        rospy.loginfo(f"已发布变换矩阵到 {self.transform_topic}")
        rospy.loginfo(f"矩阵: \n{self.T_lidar_to_world}")

    def msg_to_o3d(self, msg):
        points = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
        return pcd
    
    def lidar_callback(self, msg):
        try:
            pcd = self.msg_to_o3d(msg)
            self.current_lidar_pcd = pcd
        except Exception as e:
            rospy.logerr_throttle(5.0, f"LiDAR处理错误: {e}")
    
    def _data_loop(self):
        """子线程：维持运行状态"""
        rospy.loginfo("标定数据流线程已启动")
        
        while self._running and not rospy.is_shutdown():
            if self._shutdown_event.wait(0.1):
                break
        
        rospy.loginfo("标定数据流线程停止中...")
        try:
            if self.sub_lidar:
                self.sub_lidar.unregister()
                self.sub_lidar = None
        except Exception:
            pass
        rospy.loginfo("标定数据流线程已结束")
    
    def start(self):
        """启动数据流（非阻塞）"""
        with self._lock:
            if self._running:
                rospy.logwarn("标定节点已在运行")
                return False
            self._running = True
            self._shutdown_event.clear()
        
        if not self.load_map():
            self._running = False
            return False
        
        # 发布变换矩阵
        self.pub_transform = rospy.Publisher(self.transform_topic, Float32MultiArray, queue_size=1, latch=True)
        self.sub_lidar = rospy.Subscriber(self.lidar_topic, PointCloud2, self.lidar_callback)
        
        self._thread = threading.Thread(target=self._data_loop)
        self._thread.daemon = True
        self._thread.start()
        
        rospy.loginfo("标定器数据流已启动")
        return True
    
    def calibrate(self):
        """在主线程执行标定（阻塞，含可视化窗口）"""
        if not self._running:
            rospy.logerr("标定器未启动，请先调用 start()")
            return False
        
        success = self.perform_calibration()
        return success
    
    def stop(self, timeout=5.0):
        """停止"""
        with self._lock:
            if not self._running:
                return True
            self._running = False
        
        self._shutdown_event.set()
        
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=timeout)
        
        rospy.loginfo("标定节点已停止")
        return True
    
    def is_running(self):
        with self._lock:
            return self._running
    
    def is_calibrated(self):
        return self.calibration_done
    
    def get_transform(self):
        return self.T_lidar_to_world


def standalone_main():
    calibrator = None
    
    def signal_handler(sig, frame):
        rospy.loginfo(f"收到信号 {sig}，退出...")
        if calibrator is not None:
            calibrator.stop()
        rospy.signal_shutdown("用户中断")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rospy.init_node('lidar_map_calibration', anonymous=True)
    
    calibrator = LidarMapCalibrator()
    
    if not calibrator.start():
        rospy.logerr("启动失败")
        return 1
    
    # 执行标定（阻塞，含唯一可视化窗口）
    calibrator.calibrate()
    
    # 标定完成后保持运行，持续发布矩阵（latch=True确保新订阅者能收到）
    rospy.loginfo("标定完成，按Ctrl+C退出...")
    rospy.spin()
    
    calibrator.stop()
    return 0


if __name__ == '__main__':
    sys.exit(standalone_main())