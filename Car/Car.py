# 定义Car类和CarList类
import threading
import time

import cv2
import numpy as np


# 检测类为主线程，Lidar类通过ros不断接收雷达数据是子线程，主线程detect中目标检测完毕从子线程中获取雷达数据，进行坐标解算，写入CarList类中
# 决策与通信为子线程从CarList类中获取信息，进行决策解算后统一发信


class FieldKalmanFilter2D:
    """
    赛场平面坐标滤波器，参数参考 RM2025-Radar-Algorithm 的 KalmanFilter2d：
    q_std=2.0, r_std=1.0, dt=0.1。
    """

    def __init__(self, q_std=2.0, r_std=1.0, dt=0.1):
        self.q_std = float(q_std)
        self.r_std = float(r_std)
        self.dt = float(dt)
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0]],
            dtype=np.float32
        )
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * (self.r_std ** 2)
        self.initialized = False
        self.reset()

    def _update_transition_and_process_noise(self, dt):
        self.kf.transitionMatrix = np.array(
            [[1, 0, dt, 0],
             [0, 1, 0, dt],
             [0, 0, 1, 0],
             [0, 0, 0, 1]],
            dtype=np.float32
        )
        self.kf.processNoiseCov = np.array(
            [[dt ** 4 / 4, 0, dt ** 3 / 2, 0],
             [0, dt ** 4 / 4, 0, dt ** 3 / 2],
             [dt ** 3 / 2, 0, dt ** 2, 0],
             [0, dt ** 3 / 2, 0, dt ** 2]],
            dtype=np.float32
        ) * (self.q_std ** 2)

    def reset(self, initial_pos=None):
        self._update_transition_and_process_noise(self.dt)
        self.kf.statePost = np.zeros((4, 1), dtype=np.float32)
        self.kf.statePre = np.zeros((4, 1), dtype=np.float32)
        if initial_pos is not None:
            self.kf.statePost[0, 0] = float(initial_pos[0])
            self.kf.statePost[1, 0] = float(initial_pos[1])
            self.kf.statePre[:] = self.kf.statePost
            self.initialized = True
        else:
            self.initialized = False
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 100.0
        self.kf.errorCovPre = self.kf.errorCovPost.copy()

    def predict(self, dt=None):
        if dt is None:
            dt = self.dt
        self._update_transition_and_process_noise(float(dt))
        prediction = self.kf.predict()
        self.kf.statePost = self.kf.statePre.copy()
        self.kf.errorCovPost = self.kf.errorCovPre.copy()
        return [float(prediction[0, 0]), float(prediction[1, 0])]

    def update(self, pos):
        if not self.initialized:
            self.reset(pos)
            return self.get_position()
        measurement = np.array([[np.float32(pos[0])], [np.float32(pos[1])]])
        corrected = self.kf.correct(measurement)
        return [float(corrected[0, 0]), float(corrected[1, 0])]

    def get_position(self):
        state = self.kf.statePost
        return [float(state[0, 0]), float(state[1, 0])]

    def get_velocity(self):
        state = self.kf.statePost
        return [float(state[2, 0]), float(state[3, 0])]


# Car类，单个车辆的信息，有R1-R5，R7，B1-B5，B7
class Car:
    def __init__(self, car_id , life_span_init = 20, filter_config=None):
        filter_config = filter_config or {}
        # 主键 , 依照串口通信协议，车辆ID
        self.car_id = car_id
        # 此车颜色 , 可以直接由car_id计算
        self.color = "Red" if car_id < 100 else "Blue"
        # 与图像解算的关联
        self.track_id = -1 # 外键，车的追踪编号,认为相信追踪器的编号
        self.conf = 0 # 当前追踪器的置信度评分 , 以追踪器类的标准
        self.image_xywh = [] # 在追踪器得到的车辆在图像中的中心宽高位置，归一化坐标 ,float
        self.image_xyxy = [] # 在追踪器得到的车辆在图像中的左上角和右下角位置，归一化坐标 ,float
        self.center_xy = [] # 在追踪器得到的车辆在图像中的中心位置，归一化坐标 ,float
        # 相机坐标系下的三维坐标
        self.camera_xyz = [] # 相机坐标系下的三维坐标 , 单位是m
        # 赛场坐标系下的坐标
        self.field_xyz = [] # 赛场坐标系下的三维坐标 , 单位是m
        self.field_xy = [] # 赛场坐标系下的二维坐标 , 单位是m , float
        self.field_xy_velocity = [] # 赛场坐标系下的二维速度估计 , 单位是m/s , float
        self.field_filter_enabled = bool(filter_config.get("enabled", True))
        self.field_filter_outlier_threshold = float(filter_config.get("outlier_threshold", 1.5))
        self.field_filter_last_update_time = None
        self.field_filter = FieldKalmanFilter2D(
            q_std=float(filter_config.get("q_std", 2.0)),
            r_std=float(filter_config.get("r_std", 1.0)),
            dt=float(filter_config.get("dt", 0.1)),
        )
        self.last_field_measurement = []
        self.last_field_measurement_rejected = False
        # 当前信息可信生命周期,每次图像检测帧对所有车辆生命周期进行刷新，如果生命周期为0,则初始化所有解算信息
        self.life_span_max = life_span_init # 最大生命周期
        self.life_span = 0 # 当前生命周期，每次检测帧对所有车辆生命周期进行刷新，如果生命周期为0,则认为车辆信息不可信，但是不初始化，只是不发给哨兵，但是还是发给裁判系统
        # 车辆信息是否可信
        self.trust = False

    # 写入车辆图像追踪信息
    def set_track_info(self, track_id, conf, xywh):
        self.track_id = track_id
        self.conf = conf
        self.image_xywh = xywh
        self.center_xy = [xywh[0], xywh[1]]
        self.image_xyxy = [xywh[0] - xywh[2] / 2, xywh[1] - xywh[3] / 2, xywh[0] + xywh[2] / 2, xywh[1] + xywh[3] / 2]
        self.life_span = self.life_span_max # 重置生命周期
        self.trust = True

    # 写入相机坐标系下的三维坐标
    def set_camera_xyz(self, xyz):
        # TODO：超范围判断
        self.camera_xyz = xyz

    # 写入赛场坐标系下的三维坐标
    def set_field_xyz(self, xyz):
        if xyz is None or len(xyz) < 2:
            self.field_xyz = []
            self.field_xy = []
            self.field_xy_velocity = []
            self.last_field_measurement_rejected = False
            return

        measurement_xy = [float(xyz[0]), float(xyz[1])]
        self.last_field_measurement = measurement_xy
        z = float(xyz[2]) if len(xyz) > 2 else 0.0

        if not self.field_filter_enabled:
            self.field_xyz = [measurement_xy[0], measurement_xy[1], z]
            self.field_xy = measurement_xy
            self.field_xy_velocity = []
            self.last_field_measurement_rejected = False
            return

        now = time.time()
        if self.field_filter_last_update_time is None or not self.field_filter.initialized:
            self.field_filter.reset(measurement_xy)
            filtered_xy = self.field_filter.get_position()
            self.field_filter_last_update_time = now
            self.last_field_measurement_rejected = False
        else:
            dt = max(1e-3, now - self.field_filter_last_update_time)
            predicted_xy = self.field_filter.predict(dt)
            distance = np.linalg.norm(np.array(measurement_xy) - np.array(predicted_xy))
            if distance <= self.field_filter_outlier_threshold:
                filtered_xy = self.field_filter.update(measurement_xy)
                self.last_field_measurement_rejected = False
            else:
                filtered_xy = predicted_xy
                self.last_field_measurement_rejected = True
            self.field_filter_last_update_time = now

        self.field_xy_velocity = self.field_filter.get_velocity()
        self.field_xyz = [filtered_xy[0], filtered_xy[1], z]
        self.field_xy = [filtered_xy[0], filtered_xy[1]]



    # 中间refresh函数,初始化所有解算信息
    def refresh(self):
        # self.track_id = -1
        # self.conf = 0
        # self.image_xywh = []
        # self.image_xyxy = []
        # self.center_xy = []
        # self.camera_xyz = []
        # self.field_xyz = []
        # self.field_xy = []
        self.field_filter.reset()
        self.field_filter_last_update_time = None
        self.field_xy_velocity = []
        self.last_field_measurement_rejected = False
        self.trust = False

    # 如果本帧没有检测此车，生命周期减一
    def life_down(self):
        # if self.track_id == -1: # 如果本来就掉追踪了，直接返回
        #     return
        if not self.trust: # 如果已经不可信了
            return
        self.life_span -= 1
        if self.life_span < 0:
            print("refresh",self.car_id)
            self.refresh()
    # 中间方法，为了让没有刷新的车辆生命周期减一，先对刷新的车辆生命周期+1,再对所有车辆生命周期-1
    def life_up(self):

        self.life_span += 1

    # 获取车辆中心
    def get_center(self): # 返回车辆中心图像坐标，[x,y],float , 归一化
        return self.center_xy

    # 获取车辆赛场坐标
    def get_field_xy(self): # 返回车辆赛场坐标，[x,y],float , 单位是m
        return self.field_xy

    # 获取车xyz坐标
    def get_field_xyz(self):
        return self.field_xyz

    # 获取当前置信度
    def get_conf(self):
        return self.conf

    # 判断是否为敌方车辆
    def is_enemy(self , my_color):
        return self.color != my_color










# CarList类，车辆列表，用于存储所有车辆的信息 , 串口通信和决策从这里获取信息，所有的解算向这里写入信息
class CarList:
    def __init__(self , cfg):
        # 配置文件
        self.my_color = cfg["global"]["my_color"] # 我方颜色 , "Red" or "Blue"
        if self.my_color == "Red":
            self.sentinel_id = 7
            self.enemy_ids = [101, 102, 103, 104, 105, 107]
        else:
            self.sentinel_id = 107
            self.enemy_ids = [1, 2, 3, 4, 5, 7]
        self.sentinel_min_alert_distance = 0.1 # 最近预警距离
        self.sentinel_max_alert_distance = 8.0 # 最远预警距离
        self.life_span = cfg["car"]["life_span"] # 车辆信息可信生命周期
        self.field_filter_config = cfg["car"].get("field_filter", {})
        self.RedCarsID = {1: 1, 2: 2, 3: 3, 4: 4, 5: 5 , 7:7} # 红方车辆序号和车辆ID的对应关系
        self.BlueCarsID = {1: 101, 2: 102, 3: 103, 4: 104, 5: 105 , 7:107} # 蓝方车辆序号和车辆ID的对应关系
        self.label2ID = {"R1":1, "R2":2, "R3":3, "R4":4, "R5":5, "R7":7, "B1":101, "B2":102, "B3":103, "B4":104, "B5":105, "B7":107}
        # 初始化车辆信息
        self.cars = {self.RedCarsID[1]: Car(self.RedCarsID[1], self.life_span, self.field_filter_config),
                     self.RedCarsID[2]: Car(self.RedCarsID[2], self.life_span, self.field_filter_config),
                     self.RedCarsID[3]: Car(self.RedCarsID[3], self.life_span, self.field_filter_config),
                     self.RedCarsID[4]: Car(self.RedCarsID[4], self.life_span, self.field_filter_config),
                     self.RedCarsID[5]: Car(self.RedCarsID[5], self.life_span, self.field_filter_config),
                     self.RedCarsID[7]: Car(self.RedCarsID[7], self.life_span, self.field_filter_config),
                     self.BlueCarsID[1]: Car(self.BlueCarsID[1], self.life_span, self.field_filter_config),
                     self.BlueCarsID[2]: Car(self.BlueCarsID[2], self.life_span, self.field_filter_config),
                     self.BlueCarsID[3]: Car(self.BlueCarsID[3], self.life_span, self.field_filter_config),
                     self.BlueCarsID[4]: Car(self.BlueCarsID[4], self.life_span, self.field_filter_config),
                     self.BlueCarsID[5]: Car(self.BlueCarsID[5], self.life_span, self.field_filter_config),
                     self.BlueCarsID[7]: Car(self.BlueCarsID[7], self.life_span, self.field_filter_config)}
        # 对CarList实例多线程锁，为了尽量减少上锁时间，把数据处理好再写入公共区域
        self.lock = threading.Lock()
        print(self.cars[self.BlueCarsID[1] if self.my_color == "Blue" else self.RedCarsID[1]].get_field_xyz())

    # 每一个检测循环，刷新所有车辆信息,检测线程进行到写入部分时，会计算好每个追踪器和车辆的对应关系，确保一个车辆类型只被一个追踪器对应
    # 并计算好车辆的所有位置信息 ， 打包为results传入
    # result in results: [track_id , car_id , xywh , conf ,  camera_xyz , filed_xyz ]
    # 注意，car_id需要转为[1,2,3,4,5,7,101,102,103...]的形式再打包
    def update_car_info(self , results):
        with self.lock:
            for result in results:
                track_id, car_id, xywh, conf, camera_xyz, field_xyz = result
                print(result)
                # 根据car_id找到对应的Car对象
                car = self.get_car_by_id(car_id)
                if car is not None:
                    # 更新Car对象的信息
                    car.set_track_info(track_id, conf, xywh)
                    car.set_camera_xyz(camera_xyz)
                    car.set_field_xyz(field_xyz)
                    car.life_span = car.life_span_max # 重置生命周期
                    car.life_up() # 刷新的车辆生命周期先+1
            # 对所有车辆生命周期减一,这样没有刷新的车辆生命周期减一
            for car in self.cars.values():
                # print("life down",car.life_span)
                car.life_down()

    # 根据car_id获取车对象，中间方法
    def get_car_by_id(self, car_id):
        return self.cars.get(car_id, None)

    # 读取发送小地图和决策信息，返回results
    # result in results:[car_id , [field_x , field_y , field_z]]
    def get_map_info(self):
        # 读取所有车辆的赛场坐标信息
        results = []
        with self.lock:
            for car in self.cars.values():
                if car.field_xyz != []:
                    results.append([car.car_id, car.field_xyz])
        return results

    # 获取所有信息，如果车的信息可信,返回车的id，中心坐标，相机坐标，赛场坐标 , 颜色
    # result in results:[car_id , center_xy , camera_xyz , field_xyz ， color]
    def get_all_info(self):
        results = []
        with self.lock:
            for car in self.cars.values():
                results.append([car.track_id , car.car_id, car.center_xy, car.camera_xyz, car.field_xyz , car.color , car.trust])
        return results

    # 获取所有车辆生命周期信息，用于视觉结果话题发布
    def get_life_info(self):
        results = []
        with self.lock:
            for car in self.cars.values():
                results.append({
                    "track_id": car.track_id,
                    "car_id": car.car_id,
                    "life_span": car.life_span,
                    "life_span_max": car.life_span_max,
                    "trust": car.trust,
                })
        return results

    # 简易辅助哨兵决策测试用，获取图像坐标系下我方哨兵(7号车）和敌方车辆的中心点信息,返回results
    # result:[sentinel_xy , enemy_infos] , sentinel_xy:[sentinel_x,sentinel_y] , 归一化
    # enemy_info in enemy_infos:[enemy_id , center_x , center_y] # enemy_id为1-5,7 或101-105,107
    def get_center_info(self):
        results = []
        sentinel_id = self.sentinel_id
        enemy_ids = self.enemy_ids
        with self.lock:
            sentinel = self.get_car_by_id(sentinel_id)
            sentinel_xy = sentinel.get_center()
            enemy_infos = []
            for enemy_id in enemy_ids:
                enemy = self.get_car_by_id(enemy_id)
                enemy_infos.append([enemy_id, enemy.get_center()])
            results = [sentinel_xy, enemy_infos]

        return results

    # 由标签获取车ID，如输入"R1”，返回1
    def get_car_id(self , label):
        return self.label2ID[label]
