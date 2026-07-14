# 相机到赛场坐标解算工具：负责选点初始化、视觉定位、重投影和哨兵预警角度分区。
import numpy as np
from camera_locator.anchor import Anchor
from camera_locator.point_picker import PointsPicker
from .vision_locator import Vision_Locator
import cv2
import yaml
from Log.Log import RadarLog

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
                pp.caller(image, anchor)
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
                self.field_to_camera_T = translation_vector
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


    def camera_results(self, box,t):
        '''
        Args:
            box: 一个检测框的结果
        Returns: 坐标值
        '''
        x, y, w, h = box
        # 原图中装甲板的中心下沿作为待仿射变化的点
        camera_point = np.array([[[min(x, self.width), min(y, self.height)]]],
                                dtype=np.float32)
        height = self.vision_locator.get_height(camera_point)
        [x, y] = self.vision_locator.parser(camera_point)
        y += 15 # 平移坐标系
        return [x, y, height + 0.15, t]

    def detection_main(self, box,t):
        '''

        Args:
            box: yolo给的bbox，整车
        Returns: 定位坐标值[x,y,z]

        '''
        return self.camera_results(box,t)

###-------------------------------------2025---------------------------------------###

    # 将角度转为象限 , -22.5-22.5为0,顺时针22.5-67.5为1，以此类推
    def angle_to_quadrant(self, angle):
        if -22.5 <= angle < 22.5:
            return 0
        if 22.5 <= angle < 67.5:
            return 1
        if 67.5 <= angle < 112.5:
            return 2
        if 112.5 <= angle < 157.5:
            return 3
        if angle >= 157.5 or angle < -157.5:
            return 4
        if -157.5 <= angle < -112.5:
            return 5
        if -112.5 <= angle < -67.5:
            return 6
        if -67.5 <= angle < -22.5:
            return 7

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
