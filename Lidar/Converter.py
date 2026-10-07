# 相机到赛场坐标解算工具：负责选点初始化、视觉定位、重投影和哨兵预警角度分区。
import numpy as np
from camera_locator.anchor import Anchor
from camera_locator.point_picker import PointsPicker
from .vision_locator import Vision_Locator
from .units import require_meter_unit, validate_meter_vector
from .raycast_locator import RaycastLocator
import cv2
import yaml
from Log.Log import RadarLog
from Tools.Paths import resolve_project_path

# 定位模式(配置在 configs/raycast_config.yaml 的 raycast.mode):
#   raycast_only     只用 raycast; 打空/撞结构/不可用 → 丢弃该检测, 绝不使用老算法
#   raycast_fallback raycast 优先, 未命中回退老透视算法(默认)
#   perspective_only 只用老透视算法, 不加载 raycast 网格(省内存/启动时间)
LOCALIZATION_MODES = ('raycast_only', 'raycast_fallback', 'perspective_only')
DEFAULT_LOCALIZATION_MODE = 'raycast_fallback'


def resolve_localization_mode(raycast_cfg):
    """把 raycast 配置解析成三种定位模式之一。

    优先读 raycast.mode; 没有该字段时按旧配置推导, 保证老 yaml 行为不变:
        enabled: false                        -> perspective_only
        enabled: true,  fallback_to_perspective: false -> raycast_only
        enabled: true,  fallback_to_perspective: true  -> raycast_fallback
    """
    mode = str(raycast_cfg.get('mode', '') or '').strip()
    if mode:
        if mode not in LOCALIZATION_MODES:
            raise ValueError(
                "raycast.mode 非法: %r; 只能是 %s" % (mode, " / ".join(LOCALIZATION_MODES)))
        return mode
    if not bool(raycast_cfg.get('enabled', False)):
        return 'perspective_only'
    if not bool(raycast_cfg.get('fallback_to_perspective', True)):
        return 'raycast_only'
    return DEFAULT_LOCALIZATION_MODE


class Converter:
    def __init__(self, my_color, data_loader_path='parameters.yaml'):
        self.logger = RadarLog("Converter")
        # 传入data_loader路径,用data_loader初始化类
        enemy_Base_25 = [25.50932, -7.5, 1.043 + 0.2]
        enemy_Tower_25 = [16.92483, -3.64301, 1.342 + 0.4]
        self_FORTRESS = [6.600, -7.5, 0.151]
        self_Tower_25 = [10.91891, -11.17852, 0.46769 + 0.4]# 我方前哨靠我侧血条底部
        enemy_FORTREES_RIGHT_FRONT = [20.83546, -8.47781, 0.0] # 敌方堡垒右前角
        enemy_FORTRESS_RIGHT_BACK = [21.96454, -8.47781, 0.0] # 敌方堡垒右后角
        big_BUFF = [13.76595, -7.26594, 2.6]
        enemy_HERO_HIGH = [22.33747, -12.07767, 0.6] # 对面英雄吊射高地
        # test
        self.point_1 = [9.553, -6.08209, 0.2]
        self.point_2 = [10.018, -9.92556, 0.2]
        self.point_3= [17.77462, -11.94595, 0.8]
        self.point_4 = [20.35278, -2.168, 0.30176]

        self.global_color = my_color
        data_loader_path = resolve_project_path(data_loader_path)
        with open(data_loader_path, 'r',encoding='utf-8', errors='ignore') as file:
            data_loader = yaml.safe_load(file)
        # 2025
        self.real_points_25 = [enemy_Base_25, enemy_Tower_25, self_FORTRESS, self_Tower_25, enemy_FORTREES_RIGHT_FRONT]
        # 2026
        self.real_points_26 = [big_BUFF, enemy_Tower_25, self_FORTRESS, self_Tower_25, enemy_HERO_HIGH]
        #test
        self.test_points = [enemy_FORTREES_RIGHT_FRONT, enemy_FORTRESS_RIGHT_BACK,self.point_3,self.point_4]
        # 获取相机坐标系到激光雷达坐标系的外参
        # 获取R和T，并将它们转换为NumPy数组
        self.R = np.array(data_loader['calib']['extrinsic']['R']['data']).reshape(
            (data_loader['calib']['extrinsic']['R']['rows'], data_loader['calib']['extrinsic']['R']['cols']))
        self.T = np.array(data_loader['calib']['extrinsic']['T']['data']).reshape(
            (data_loader['calib']['extrinsic']['T']['rows'], data_loader['calib']['extrinsic']['T']['cols']))
        # 获取相机内参
        self.cx = data_loader['calib']['intrinsic']['cx']
        self.cy = data_loader['calib']['intrinsic']['cy']
        self.fx = data_loader['calib']['intrinsic']['fx']
        self.fy = data_loader['calib']['intrinsic']['fy']
        self.max_depth = data_loader['params']['max_depth']
        self.width = data_loader['params']['width']
        self.height = data_loader['params']['height']
        # 获取聚类参数
        self.eps = data_loader['cluster']['eps']
        self.min_points = data_loader['cluster']['min_points']
        self.print_cluster_progress = data_loader['cluster']['print_progress']
        # 获取滤波参数
        self.nb_neighbors = data_loader['filter']['nb_neighbors']
        self.std_ratio = data_loader['filter']['std_ratio']
        self.voxel_size = data_loader['filter']['voxel_size']
        # 去畸变参数
        self.distortion_matrix = np.array(data_loader['calib']['distortion']['data'])
        # 相机坐标系到图像坐标系的内参矩阵，3*3的矩阵
        self.intrinsic_matrix = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]],dtype=np.float32)
        # 图像坐标系到相机坐标系的内参矩阵，3*3的矩阵
        self.intrinsic_matrix_inv = np.linalg.inv(self.intrinsic_matrix)
        # 激光雷达到相机的外参矩阵，4*4的矩阵，前三列为旋转矩阵，第四列为平移矩阵
        self.extrinsic_matrix = np.hstack((self.R, self.T))
        self.extrinsic_matrix = np.vstack((self.extrinsic_matrix, [0, 0, 0, 1]))
        # 相机到激光雷达的外参矩阵，4*4的矩阵，前三列为旋转矩阵，第四列为平移矩阵
        self.extrinsic_matrix_inv = np.linalg.inv(self.extrinsic_matrix)
        self.logger.log(
            "lidar_to_camera extrinsic_matrix:\n"
            + np.array2string(self.extrinsic_matrix, precision=8, suppress_small=False)
        )
        self.logger.log(
            "camera_to_lidar extrinsic_matrix_inv:\n"
            + np.array2string(self.extrinsic_matrix_inv, precision=8, suppress_small=False)
        )
        # 相机到赛场坐标系的外参矩阵，4*4的矩阵，前三列为旋转矩阵，第四列为平移矩阵
        self.camera_to_field_R = None  # 后面初始化
        self.camera_to_field_T = None  # 后面初始化
        self.camera_to_field_matrix = None  # 后面初始化
        self.field_to_camera_R = None  # 后面初始化
        self.field_to_camera_T = None  # 后面初始化
        self.field_to_camera_matrix = None  # 后面初始化
        print(self.extrinsic_matrix)
        print(self.intrinsic_matrix)
        # 视觉定位类
        self.vision_locator = None
        self.armor_height = 0.15

        # ---- raycast 配置(可选;缺省关闭,单位缺省米) ----
        self.raycast_locator = None
        self.raycast_config = {}
        raycast_cfg_path = resolve_project_path('configs/raycast_config.yaml')
        if raycast_cfg_path.exists():
            with open(str(raycast_cfg_path), 'r', encoding='utf-8') as rf:
                raycast_cfg = yaml.safe_load(rf) or {}
            self.raycast_config = raycast_cfg.get('raycast', {})
        self.mesh_unit = require_meter_unit(
            self.raycast_config.get('unit', 'm'), 'Converter raycast mesh')

        # ---- 定位模式开关(改 configs/raycast_config.yaml 的 raycast.mode) ----
        self.localization_mode = resolve_localization_mode(self.raycast_config)
        self.use_raycast = self.localization_mode != 'perspective_only'
        self.fallback_to_perspective = self.localization_mode == 'raycast_fallback'
        # 定位来源统计(每 stats_print_every 次检测打印一行; 设 0 关闭)
        self.stats_print_every = int(self.raycast_config.get('stats_print_every', 200))
        self.raycast_hit_count = 0
        self.raycast_miss_count = 0
        self.raycast_drop_count = 0
        self.perspective_count = 0
        self._loc_processed = 0
        print('[loc] 定位模式: %s (raycast=%s, 回退老算法=%s)' % (
            self.localization_mode, self.use_raycast, self.fallback_to_perspective))

    def localization_stats(self):
        """定位来源统计, 便于确认当前到底在走哪条链路。"""
        return {
            'mode': self.localization_mode,
            'raycast_hit': self.raycast_hit_count,
            'raycast_miss': self.raycast_miss_count,
            'raycast_dropped': self.raycast_drop_count,
            'perspective_used': self.perspective_count,
        }

    def _note_localization(self):
        self._loc_processed += 1
        if self.stats_print_every <= 0:
            return
        if self._loc_processed % self.stats_print_every == 0:
            print('[loc] mode=%s 已处理 %d: raycast命中 %d | 打空 %d | 丢弃 %d | 老算法 %d' % (
                self.localization_mode, self._loc_processed, self.raycast_hit_count,
                self.raycast_miss_count, self.raycast_drop_count, self.perspective_count))



    def _calibration_hints_25(self):
        """real_points_25 标定点点选提示: 结构名 + 当前阵营视角下的方位。"""
        base = ["敌方基地", "敌方前哨塔", "我方堡垒", "我方前哨塔", "敌方堡垒右前角"]
        if self.global_color == "Blue":
            tips = ["最远端端线中央高台", "近-中高柱", "最远端低台",
                    "远-中高柱", "近端半场堡垒角"]
        else:
            tips = ["最远端端线中央高台", "远-中高柱", "近端低台(脚下侧)",
                    "近-中高柱", "远端半场堡垒角"]
        return ["%s(%s)" % (b, t) for b, t in zip(base, tips)]

    def camera_to_field_init(self, capture=None, img = None):
        # 初始化要用的类
        anchor = Anchor()
        pp = PointsPicker()
        while True:
            # 获得一张图片
            image = capture.get_frame()
            if image is None:
                # 视频读完，重置到开头重新取帧
                if hasattr(capture, 'cap'):
                    capture.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    image = capture.get_frame()
                if image is None:
                    print("无法获取帧")
                    return
            # 把image resize为1920*1080
            show_image = cv2.resize(image, (1920, 1080))
            cv2.imshow("clear press y else n", show_image)
            # 接收按键，如果y则进入下一步，否则重选一张
            key = cv2.waitKey(0)
            if key == ord('y'):
                calib_hints = self._calibration_hints_25()
                print("请按顺序点击 5 个标定点(点选窗口会显示当前第几点):")
                for i, hint in enumerate(calib_hints):
                    print("  %d. %s" % (i + 1, hint))
                pp.caller(image, anchor, names=[h.split('(')[0] for h in calib_hints])
                true_points = np.array(self.real_points_25, dtype=np.float32)
                pixel_points = np.array(anchor.vertexes, dtype=np.float32)
                print(pixel_points)
                ok, rotation_vector, translation_vector = cv2.solvePnP(true_points, pixel_points,
                                                                       self.intrinsic_matrix,
                                                                       self.distortion_matrix, flags=cv2.SOLVEPNP_EPNP)
                if not ok:
                    print("solvePnP EPNP failed")
                    continue
                ok, rotation_vector, translation_vector = cv2.solvePnP(true_points, pixel_points,
                                                                       self.intrinsic_matrix,
                                                                       self.distortion_matrix,
                                                                       rotation_vector,
                                                                       translation_vector,
                                                                       useExtrinsicGuess=True,
                                                                       flags=cv2.SOLVEPNP_ITERATIVE)
                if not ok:
                    print("solvePnP ITERATIVE refine failed")
                    continue
                rotation_matrix = cv2.Rodrigues(rotation_vector)[0]  # 从赛场到相机的旋转矩阵
                self.field_to_camera_R = rotation_matrix
                # 米制防线: 标定点/透视世界点均为米,拦截毫米或异常数量级平移
                self.field_to_camera_T = validate_meter_vector(
                    translation_vector, 'PnP translation').reshape(3, 1)
                if np.linalg.norm(self.field_to_camera_T) > 100.0:
                    print('PnP translation implausibly large for meters; '
                          'check calibration-point units (must be meters).')
                    continue
                # self.field_to_camera_T = np.array([x * 1000 for x in translation_vector],dtype=np.float32)
                # 将旋转矩阵R和平移向量T合并成一个4x4的齐次坐标变换矩阵
                # 注意这里使用 rotation_matrix 和 translation_vector，前者是赛场到相机的旋转矩阵，后者是对应的平移向量
                transformation_matrix = np.hstack((rotation_matrix, translation_vector.reshape(-1, 1)))  # 创建包含R和T的3x4矩阵

                transformation_matrix = np.vstack((transformation_matrix, [0, 0, 0, 1]))  # 添加一个[0, 0, 0, 1]行向量
                self.field_to_camera_matrix = transformation_matrix
                self.vision_locator_init(image)
                self.logger.log(str(self.field_to_camera_matrix))
                self.logger.log(str(self.field_to_camera_R))
                self.logger.log(str(self.field_to_camera_T))
                break
            else:
                continue
    ###-------------------------------------2025---------------------------------------###
    def vision_locator_init(self,img=None):
        self.vision_locator = Vision_Locator(intrinsic_matrix=self.intrinsic_matrix,
                                             dist_coeffs=self.distortion_matrix,
                                             world_rvec=self.field_to_camera_R,
                                             world_tvec=self.field_to_camera_T,
                                             extrinsic_matrix=self.field_to_camera_matrix,img=img)
        self._init_raycast_locator()

    # ------------------------------------raycast------------------------------------#
    def _legacy_to_field_pose(self):
        """Convert the PnP pose (legacy frame) to the field frame.

        标定点/透视世界点位于 legacy 系(y_legacy = y_field - 15)。由
            X_cam = R·X_leg + T_leg,  X_leg = X_field - (0, 15, 0)
        得场地系 pose:
            R_field = R,  T_field = T_leg - R·(0, 15, 0)
        """
        offset = np.array([0.0, 15.0, 0.0])
        t_field = self.field_to_camera_T.reshape(3) - self.field_to_camera_R @ offset
        return self.field_to_camera_R, t_field.reshape(3, 1)

    def _init_raycast_locator(self):
        cfg = self.raycast_config
        if not self.use_raycast:
            print('[loc] perspective_only: 不加载 raycast 网格')
            return
        mesh_path = resolve_project_path(cfg.get('mesh_path', 'RM2026_map_m.ply'))
        if not mesh_path.exists():
            msg = '找不到 raycast 网格: %s' % mesh_path
            if self.localization_mode == 'raycast_only':
                raise RuntimeError(
                    msg + '(raycast_only 模式不允许回退, 请修正 configs/raycast_config.yaml '
                          '的 mesh_path, 或把 mode 改成 raycast_fallback)')
            print(msg + ' -> 本帧起只能走老算法')
            return
        try:
            r_field, t_field = self._legacy_to_field_pose()
            self.raycast_locator = RaycastLocator(
                camera_matrix=self.intrinsic_matrix,
                dist_coeffs=self.distortion_matrix,
                field_to_camera_R=r_field,
                field_to_camera_T=t_field,
                mesh_path=mesh_path,
                coordinate_system=cfg.get('coordinate_system', 'field'),
                unit=cfg.get('unit', self.mesh_unit),
            )
            print('Raycast locator initialized: %s' % mesh_path)
        except (ImportError, ValueError, RuntimeError, OSError) as exc:
            self.raycast_locator = None
            if self.localization_mode == 'raycast_only':
                raise RuntimeError(
                    'raycast_only 模式下 raycast 初始化失败, 拒绝回退老算法: %s' % exc)
            print('Raycast unavailable; using perspective fallback: %s' % exc)

    def _raycast_result(self, box, t):
        """raycast 命中 → [x, y, z, t](场地系,米);未命中/撞结构/禁用返回 None。

        防护(raycast 缺陷补偿):
        - 多射线: 底边像素向下多打几根, 取命中 z 最小(最贴地面)的一根,
          绕开高台侧壁与重建孔洞的穿透;
        - z 过滤: 所有命中高度均高于 ground_max_z 时视为撞到结构(非地面),
          丢弃本帧, 由调用方回退透视/卡尔曼。
        """
        if self.raycast_locator is None:
            return None
        values = np.asarray(box, dtype=np.float64).reshape(-1)
        if values.size < 2:
            return None
        u, v = float(values[0]), float(values[1])
        max_z = float(self.raycast_config.get('ground_max_z', 0.30))
        probe_px = self.raycast_config.get('probe_offsets_px', [6.0, 12.0])
        layers = self.raycast_config.get('height_layers', None)

        # ---- 分层模式(仿老算法分层): 命中z归入最近高度层, 输出层平面交点 ----
        if layers:
            main = self.raycast_locator.pixel_to_world((u, v))
            if main is None:
                return None
            if main[2] > max(layers) + 0.30:
                return None  # 命中远高于最高层 → 撞结构/异常, 回退透视
            layer = min(layers, key=lambda zk: abs(zk - main[2]))
            print('[ray] u=%.0f v=%.0f mesh_z=%.4f -> layer=%.2f' % (u, v, main[2], layer)) 
            point = self.raycast_locator.plane_intersect((u, v), layer)
            if point is None:
                return None
            return [float(point[0]), float(point[1]), float(layer), t]

        # ---- 单射线模式: 主射线优先, 备用射线兜底 ----
        main = self.raycast_locator.pixel_to_world((u, v))
        if main is not None and main[2] <= max_z:
            return [float(main[0]), float(main[1]), float(main[2]), t]
        for offset in probe_px:
            p = self.raycast_locator.pixel_to_world((u, v + float(offset)))
            if p is not None and p[2] <= max_z:
                return [float(p[0]), float(p[1]), float(p[2]), t]
        return None


    def camera_results(self, box,t):
        '''
        Args:
            box: 一个检测框的结果
        Returns: 坐标值 [x,y,z,t]; 但 raycast_only 模式下未命中/撞结构时返回 None
                     (调用方应丢弃该检测, 不要当成坐标使用)
        定位模式由 configs/raycast_config.yaml 的 raycast.mode 决定:
            raycast_only / raycast_fallback / perspective_only
        '''
        mode = self.localization_mode

        # ---- 模式 1/2: raycast 优先 ----
        if mode != 'perspective_only':
            raycast_result = self._raycast_result(box, t)
            if raycast_result is not None:
                self.raycast_hit_count += 1
                self._note_localization()
                return raycast_result
            self.raycast_miss_count += 1
            if mode == 'raycast_only':
                # 只用 raycast: 打空/撞结构就丢弃本帧该检测(老算法完全不参与),
                # CarList 的生命周期机制会让该车逐渐过期, 不会发出错误坐标
                self.raycast_drop_count += 1
                self._note_localization()
                return None
            self._note_localization()

        # ---- 模式 2/3: 老透视算法 ----
        x, y, w, h = box
        # 原图中装甲板的中心下沿作为待仿射变化的点
        u = np.clip(x, 0, self.width - 1)
        v = np.clip(y, 0, self.height - 1)
        camera_point = np.array([[[u, v]]], dtype=np.float32)
        height = self.vision_locator.get_height(camera_point)
        [x, y] = self.vision_locator.parser(camera_point, height)
        y += 15 # 平移坐标系
        self.perspective_count += 1
        return [x, y, height + self.vision_locator.armor_height, t]

    def detection_main(self, box,t):
        '''

        Args:
            box: yolo给的bbox，整车
        Returns: 定位坐标值[x,y,z]; raycast_only 模式下打空/撞结构时为 None

        '''
        return self.camera_results(box,t)

    def camera_to_image(self, pc):  # 传入相机坐标系点，返回图像坐标系下的u,v和z
        # 相机坐标系下的点云批量乘以内参矩阵，得到图像坐标系下的u,v和z,类似于深度图的生成

        # 确保为numpy数组
        pc = np.asarray(pc)

        xyz = np.dot(pc, self.intrinsic_matrix.T)  # 得到的uvz是一个n*3的矩阵，n是点云的数量，是np.array格式的
        # 之前深度图没正确生成是因为没有提取z出来，导致原来的uv错误过大了
        # 要获得u,v,z，需要将xyz的第三列除以第三列
        uvz = np.zeros(xyz.shape)
        uvz[:, 0] = xyz[:, 0] / xyz[:, 2]
        uvz[:, 1] = xyz[:, 1] / xyz[:, 2]
        uvz[:, 2] = xyz[:, 2]

        return uvz

    # 求一个点[x,y,z]的距离
    def get_distance(self, point):
        point = np.array(point)
        return np.sqrt(np.sum(point ** 2))
