from .Sender import Sender
from .Topo_map.topo_lidar import *
from random import randint

import copy
import json
import time
from shapely.geometry import Point, Polygon
import numpy as np
import cv2
from ruamel.yaml import YAML
from Log.Log import RadarLog
from Tools.Tools import Tools
from Radio.interferance_level_sender import InterferenceSender

# ROS 导入
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from .assit_yaw_pitch import BallisticTrajectory
from .alert_our_hero import is_point_nearby_numpy
from .predictor import CarKalmanPredictor
from .hero_topo_predictor.predict_hero import Predictor as Hero_Predictor
from .engine_topo_predictor.predict_engine import Predictor

MAIN_CONFIG_PATH = "./configs/main_config.yaml"


# from .assit_yaw_pitch import Hero_Assit

class Messager:
    def __init__(self, cfg):
        # 全局变量
        self.is_debug = cfg["global"]["is_debug"]
        messager_cfg = cfg.get('messager', {})
        self.state_topic = messager_cfg.get('state_topic', '/messager/state')
        communication_cfg = cfg.get('communication', {})
        self.receiver_state_topic = communication_cfg.get('receiver_state_topic', '/receiver/state')
        self.main_loop_hz = float(messager_cfg['main_loop_hz'])
        self.map_hz = float(messager_cfg['map_hz'])
        self.sentry_hz = float(messager_cfg['sentry_hz'])
        self.enemy_hp_hz = float(messager_cfg['enemy_hp_hz'])
        self.double_effect_hz = float(messager_cfg['double_effect_hz'])
        # ROS 订阅数据缓存
        self._drone_field_xyz = None  # [x, y, z] 赛场坐标系下的无人机坐标
        self._drone_field_timestamp = 0.0

        self.interferance_level = 1  # 当前干扰等级，默认1级
        self.interferance_level_list = [1, 2, 3]  # 可用的干扰等级列表
        self.is_key_update = 0
        self._jam_key = None  # 干扰波密钥
        self._jam_key_timestamp = 0.0

        # 敌方血量订阅缓存（替代 UDP 获取）
        self._enemy_health_array = None  # [hero, engineer, infantry_3, infantry_4, reserved, sentry]
        self._health_timestamp = 0.0
        self.dart_target_times = [0, 0, 0]
        # log部分
        self.logger = RadarLog("Messager")
        self.status_logger = RadarLog("Messager_Status")

        # 发送部分
        self.position_topic = communication_cfg.get('position_topic', '/messge/position')
        self.use_ros_position = communication_cfg.get('send_transport', 'serial') == 'ros1'
        sender_cfg = copy.deepcopy(cfg)
        self.sender = Sender(sender_cfg)
        self.position_pub = None
        self.position_seq = 0
        self.double_effect_times = 0  # 第几次发送双倍易伤效果决策,第一次发送值为1，第二次发送值为2，每局最多只能发送到2,不能发送3
        self.udp_sender = InterferenceSender("192.168.3.99", 40003)  # 用于发送干扰等级的UDP发送器，独立于主Sender，专门发送int类型的干扰等级数据

        # 特殊flag
        # self.super_flag = cfg['others']['is_vs_qd'] # 打青岛大学的特殊flag
        # self.is_vs_hg = cfg['others']['is_vs_hg']
        self.send_double_time_threshold = 240  # 发送双倍易伤的时间阈值，单位秒

        # 初始化 ROS 订阅（非阻塞回调方式）
        self._init_ros_subscribers()

        # 数据存储
        self.our_car_infos = []  # 我方车辆信息 , our_car_id , our_center_xy , our_camera_xyz , our_field_xyz , our_color
        self.our_drone_info = [0., 0.] # 我方无人机信息
        self.enemy_car_infos = []  # 敌方车辆信息，enemy_car_id , enemy_center_xy , enemy_camera_xyz , enemy_field_xyz , enemy_color = enemy_car_info
        self.enemy_drone_info = [0., 0.] # 敌方无人机信息
        self.sentinel_alert_info = []  # 哨兵预警信息，匹配sender的generate_sentinel_alert_info(self , carID , distance , quadrant):
        self.send_hero_assit_info = {}  # 英雄协助信息
        self.time_left = -1  # 剩余时间
        self.last_time_left = -1  # 上次剩余时间 , 用于判断是否更新

        # predictor
        self.predictor = {1: CarKalmanPredictor(0, 0), 101: CarKalmanPredictor(0, 0),
                          2: CarKalmanPredictor(0, 0), 102: CarKalmanPredictor(0, 0),
                          3: CarKalmanPredictor(0, 0), 103: CarKalmanPredictor(0, 0),
                          4: CarKalmanPredictor(0, 0), 104: CarKalmanPredictor(0, 0),
                          5: CarKalmanPredictor(0, 0), 106: CarKalmanPredictor(0, 0),
                          7: CarKalmanPredictor(0, 0), 107: CarKalmanPredictor(0, 0)}

        self.predictor_times = {1: 0, 101: 0, 2: 0, 102: 0, 3: 0, 103: 0, 4: 0, 104: 0, 5: 0, 106: 0, 7: 0, 107: 0}

        # 次数记录
        self.hero_enter_times = 0

        # 我方颜色
        self.my_color = self.sender.my_color

        # 敌我初始化
        if self.my_color == "Red":  # 红方是1-7 ， 蓝方是101-107
            self.enemy_hero_id = 101
            self.enemy_id = [101, 102, 103, 104, 106, 107]
            self.my_cars_id = [1, 2, 3, 4, 6, 7]
            self.my_sentinel_id = 7
        elif self.my_color == "Blue":
            self.enemy_hero_id = 1
            self.enemy_id = [1, 2, 3, 4, 6, 7]
            self.my_cars_id = [101, 102, 103, 104, 106, 107]
            self.my_sentinel_id = 107

        else:
            print("检查main_config里己方颜色是否大写！")
            exit(0)

        # 时间记录
        self.last_send_double_effect_time = time.time()
        self.last_send_map_time = time.time()
        self.last_send_sentry_time = time.time()
        self.last_update_time_left_time = time.time()
        self.last_main_loop_time = time.time()
        self.first_big_buff_send = False
        self.second_big_buff_send = False
        # 创建一个1-6,7,101-106,107的上次发送时间的字典
        if self.sender.my_color == "Blue":
            self.last_send_time_map = {1: time.time(), 2: time.time(), 3: time.time(), 4: time.time(), 6: time.time(),
                                       7: time.time()}
        elif self.sender.my_color == "Red":
            self.last_send_time_map = {101: time.time(), 102: time.time(), 103: time.time(), 104: time.time(),
                                       106: time.time(), 107: time.time()}
        else:
            print("color error , check upper character")
            exit(0)

        # 创建一个区域列表

        self.area_list_len = cfg["area"][self.my_color]["length"]
        self.area_list = []
        for i in range(self.area_list_len):
            area = cfg["area"][self.my_color][f"area{i}"]
            self.area_list.append(area)

        # 英雄预警相关
        self.find_hero_times = 0  # 英雄在区域内的次数
        self.hero_times_threshold = cfg["area"]["hero_times_threshold"]  # 英雄在区域内的次数阈值
        self.send_double_threshold = cfg["area"]["send_double_threshold"]  # 发送双倍易伤的阈值
        self.is_alert_hero = False  # 是否预警英雄

        # self.our_hero_area = cfg["area"]["our_hero_area"]
        self.our_hero_xyz = None
        # 发射速度
        self.hero_v0 = 16 + 0.1 * randint(a=-1, b=1)
        self.hero_assit = None
        self.is_assit_hero = False
        self.secure_our_hero = False
        self.hero_state = 0  # 0,1,2
        self.hero_shooting_points = {1: [18.0, 4.85], 2: [18.75, 11.1]} if self.my_color == 'Blue' else {
            1: [10.0, 10.15], 2: [9.25, 3.9]}
        self.hero_predictor = Hero_Predictor(self.my_color, 'hero')
        self.engine_predictor = Predictor(self.my_color, 'engine')
        self.send_map_infos_bkp = [2.39681, 2.36].copy() if self.sender.my_color == 'Blue' else [25.60419, 12.6389].copy()

        # 发送小地图历史记录
        self.send_map_infos = [[0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.], \
                               [0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.]]  # 哨兵全局感知
        self.send_map_info_is_latest = [0, 0, 0, 0, 0, 0, \
                                        0, 0, 0, 0, 0, 0]  # 是否是最新的小地图信息

        # 赛场状态
        # 双倍易伤相关
        self.have_double_effect_times = 0  # 拥有的双倍易伤次数
        self.is_activating_double_effect = False  # 正在激活双倍易伤

        self.already_activate_double_effect_times = 0  # 已经激活了双倍易伤次数,请求时标号为这个数+1
        # 标记进度
        self.mark_progress = [0, 0, 0, 0, 0, 0]  # 标记进度,对应对方1，2，3，4，无人机和哨兵
        self.hero_is_marked = False  # 1号英雄是否被标记
        self.engineer_is_marked = False  # 2号工程车是否被标记
        self.standard_3_is_marked = False  # 3号步兵车是否被标记
        self.standard_4_is_marked = False  # 4号步兵车是否被标记
        self.drone_is_marked = False  # 无人机是否被标记
        self.sentinel_is_marked = False  # 7号哨兵是否被标记
        self.marked_num = 0  # 被标记的数量
        self.my_health_info = [100, 100, 100, 100, 100, 0, 1500, 5000]
        self.enemy_health_info = [100, 100, 100, 100, 100]
        self.hero_is_dead = False  # 1号英雄是否死亡

        # 飞镖目标
        self.dart_target = 0

        # 拓扑图
        # self.topo_map = Topology(self.my_color)
        # 打印区域列表

        # for area in self.area_list:
        #     print(area)

        # flag
        self.working_flag = False

    # 判断是否为下一秒
    def is_next_second(self):
        if self.time_left != self.last_time_left:
            self.last_time_left = self.time_left
            return True
        return False

    # Receiver 状态由 ROS 回调更新，这里保留为空以兼容主循环调用。
    def update_receiver_info(self):
        return

    # 更新飞镖目标
    def update_receiver_dart_target(self, index=None):
        if index is None:
            return
        if not (self.dart_target == index):
            self.dart_target_times[index] += 1
        if self.dart_target_times[index] >= 2:
            self.dart_target = index
            self.dart_target_times = [0, 0, 0]

    # 更新己方血量信息
    def update_receiver_my_health_info(self):
        return

    # 更新标记进度
    def update_receiver_mark_progress(self):
        return

    # 更新双倍易伤次数
    def update_receiver_have_double_effect_times(self):
        return

    # 更新双倍易伤相关的flag，如果是下降沿，已发送次数+1
    def update_receiver_is_activating_double_effect_flags(self, is_active=None):
        if is_active is None:
            return
        is_active = bool(is_active)
        if self.is_activating_double_effect == True and is_active == False:
            self.is_activating_double_effect = False
            self.already_activate_double_effect_times += 1
        else:
            self.is_activating_double_effect = is_active

    # 更新剩余时间
    def update_receiver_time_left(self):
        return

    # hero_alert的辅助函数，将真实世界坐标转换为图像坐标
    def convert_to_image_coords(self, x, y, img_width, img_height, real_width, real_height):
        img_x = int((x / real_width) * img_width)
        img_y = int(img_height - (y / real_height) * img_height)
        return img_x, img_y

    # 解析hero_xyz信息
    def parse_hero_xyz(self):
        for info in self.our_car_infos:
            track_id, our_car_id, center_xy, camera_xyz, our_field_xyz, color, is_valid = info
            if (our_car_id == self.my_cars_id[0]):
                self.our_hero_xyz = our_field_xyz

    def parse_ene_hero_xyz(self):
        for info in self.enemy_car_infos:
            track_id, our_car_id, center_xy, camera_xyz, our_field_xyz, color, is_valid = info
            if (our_car_id == self.enemy_id[0]):
                # self.our_hero_xyz = our_field_xyz
                if our_field_xyz == [] or is_valid == False:
                    break
                x, y = our_field_xyz[0], our_field_xyz[1]
                self.hero_predictor.update_cord([x, y])
                break

    def parse_engine_xyz(self):
        for info in self.our_car_infos:
            track_id, our_car_id, center_xy, camera_xyz, our_field_xyz, color, is_valid = info
            if (our_car_id == self.enemy_id[1]):
                # self.our_hero_xyz = our_field_xyz
                if our_field_xyz == [] and is_valid == False:
                    break
                x, y = our_field_xyz[0], our_field_xyz[1]
                self.engine_predictor.update_cord([x, y])
                break

    # def topo_info(self):
    #     return self.topo_map.find_attackable_regions(self.enemy_car_infos)

    # 包含可视化展示，仅DEBUG使用
    def hero_alert(self, image):
        # Function to convert real-world coordinates to image coordinates

        if self.find_hero_times < 0:  # 因为自然衰减机制，在小于0时，重置为0
            self.find_hero_times = 0

        if self.my_color == "Red":
            color = (255, 0, 0)
        elif self.my_color == "Blue":
            color = (0, 0, 255)
        else:
            color = (0, 255, 0)
        enemy_car_infos = self.enemy_car_infos

        if enemy_car_infos == []:
            self.logger.log("enemy_car_infos is empty-----------------------")

            return

        hero_x = -1
        hero_y = -1

        for enemy_car_info in enemy_car_infos:
            # 提取car_id和field_xyz
            track_id, car_id, field_xyz, is_valid = enemy_car_info[0], enemy_car_info[1], enemy_car_info[4], \
                enemy_car_info[6]
            # self.logger.log(f"car_id:{car_id},field_xyz{field_xyz}")
            # print("field_xyz" , field_xyz)
            # from array to list
            field_xyz = list(field_xyz)
            # self.logger.log(f"car_id is {car_id} , enemy_hero_id is {self.enemy_hero_id}")
            if car_id == self.enemy_hero_id:
                # self.logger.log(f"find hero at {field_xyz}")
                if field_xyz == []:
                    self.logger.log(f"field_xyz is empty")
                    # print("field_xyz is empty")
                    return
                # self.logger.log(f"cross and find hero at {field_xyz}")
                hero_x = field_xyz[0]
                hero_y = field_xyz[1]
                hero_x = max(0, min(hero_x, 28))
                hero_y = max(0, min(hero_y, 15))
                # self.logger.log(f"find hero at {hero_x} {hero_y}")
                # print(f"find hero at {hero_x} , {hero_y}")
                # 可视化处理，DEBUG

                if self.is_debug:
                    pixel_coord = self.convert_to_image_coords(hero_x, hero_y, image.shape[1], image.shape[0], 28, 15)
                    pixel_x = pixel_coord[0]
                    pixel_y = pixel_coord[1]
                    cv2.putText(image, f'{car_id}', (int(pixel_x), int(pixel_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 2)

        # 对hero_x和hero_y的判空在内部进行了，其他地方若使用hero_x和hero_y，需要注意判空
        if self.is_in_areas(hero_x, hero_y):
            self.find_hero_times += 2

        # Draw the areas , DEBUG
        if self.is_debug:
            for index, points in enumerate(self.area_list):
                # 检查敌方hero的field_xyz是否在区域内
                img_points = [self.convert_to_image_coords(x, y, image.shape[1], image.shape[0], 28, 15) for x, y in
                              points]
                img_points = np.array(img_points, dtype=np.int32)
                cv2.polylines(image, [img_points], isClosed=True, color=color, thickness=2)
                if self.find_hero_times >= self.hero_times_threshold:
                    # 将本区域填充为color色,DEBUG
                    cv2.fillPoly(image, [img_points], color)

        # 判断是否预警英雄并自然衰减
        if self.find_hero_times >= self.hero_times_threshold:
            self.is_alert_hero = True
            self.logger.log("Alert hero")
        else:
            self.is_alert_hero = False

        self.find_hero_times -= 1

    # 判断车辆是否在指定区域内
    def is_in_areas(self, x, y):
        if x < 0 or y < 0:
            return False
        point = Point(x, y)
        for area in self.area_list:
            polygon = Polygon(area)
            if polygon.contains(point):
                return True
        return False

    def generate_send_map_infos(self, enemy_infos):
        send_map_infos = [[0, 0] for _ in range(len(self.enemy_id))]
        for enemy_car_info in enemy_infos:
            # 提取car_id和field_xyz
            track_id, car_id, field_xyz, is_valid = enemy_car_info[0], enemy_car_info[1], enemy_car_info[4], \
                enemy_car_info[6]
            if field_xyz == []:
                # self.logger.log("send map field_xyz is empty")
                continue
            # 将所有信息打印
            # print("car_id:",car_id , "field_xyz:",field_xyz , "is_valid:",is_valid)
            # 提取x和y

            x, y = field_xyz[0], field_xyz[1]
            # x的控制边界，让他在[0,28]m , y控制在[0,15]m
            x = max(0, min(x, 28))
            y = max(0, min(y, 15))

            # 将所有车的x，y信息打包
            for i, enemy_id in enumerate(self.enemy_id):
                if car_id == enemy_id:
                    send_map_infos[i] = [x, y]
                    break
        return send_map_infos

    def assit_hero(self):
        if self.our_hero_xyz is None:
            return
        self.hero_assit = BallisticTrajectory(self.our_hero_xyz, self.hero_v0)
        pitch, yaw = self.hero_assit.find_optimal_parameters()
        self.send_hero_assit_info["pitch"] = pitch
        self.send_hero_assit_info["yaw"] = yaw
        self.is_assit_hero = True

    def alert_our_hero(self,info):
        if self.our_hero_xyz is None:
            return

        self.secure_our_hero, self.enemy_distance = is_point_nearby_numpy(self.our_hero_xyz,
                                                                          self.generate_send_map_infos(self.enemy_car_infos))



    # ==================== ROS 订阅接口 ====================

    def _init_ros_subscribers(self):
        try:
            # 检查 ROS 是否已初始化（ROS1 没有 is_initialized，用 try/except 判断）
            try:
                # 如果节点已初始化，rospy.get_name() 不会报错
                rospy.get_name()
                self.logger.log(f"[ROS] 节点已初始化: {rospy.get_name()}")
            except rospy.exceptions.ROSInitException:
                # 未初始化，执行初始化
                rospy.init_node('messager_ros_subscriber', anonymous=True, disable_signals=True)
                self.logger.log("[ROS] 节点初始化成功")

            # 订阅无人机赛场坐标 /drone_field_xyz
            rospy.Subscriber("/drone_field_xyz", PointCloud2, self._drone_field_callback, queue_size=5)
            self.logger.log("[ROS] 已订阅 /drone_field_xyz")

            # 订阅干扰波密钥 /radar/enemy/jam_key
            rospy.Subscriber("/radar/enemy/jam_key", String, self._jam_key_callback, queue_size=5)
            self.logger.log("[ROS] 已订阅 /radar/enemy/jam_key")

            # 订阅敌方血量 /radar/enemy/health_array
            rospy.Subscriber("/radar/enemy/health_array", Float32MultiArray, self._health_array_callback, queue_size=5)
            self.logger.log("[ROS] 已订阅 /radar/enemy/health_array")

            rospy.Subscriber(self.state_topic, String, self._vision_state_callback, queue_size=1)
            self.logger.log(f"[ROS] 已订阅 {self.state_topic}")

            rospy.Subscriber(self.receiver_state_topic, String, self._receiver_state_callback, queue_size=20)
            self.logger.log(f"[ROS] 已订阅 {self.receiver_state_topic}")

            self.position_pub = rospy.Publisher(self.position_topic, String, queue_size=20)
            self.logger.log(f"[ROS] 已发布 {self.position_topic}")

        except Exception as e:
            self.logger.log(f"[ROS] 订阅初始化失败: {e}")

    def _to_builtin(self, value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, bytes):
            return value.hex()
        if isinstance(value, (list, tuple)):
            return [self._to_builtin(item) for item in value]
        if isinstance(value, dict):
            return {key: self._to_builtin(item) for key, item in value.items()}
        return value

    def publish_position(self, msg_type, payload, rate_hz=None, tx_buff=None):
        if tx_buff is not None and not self.use_ros_position:
            self.sender.send_info(tx_buff)
            return
        if self.position_pub is None:
            return
        msg = {
            "seq": self.position_seq,
            "stamp": rospy.Time.now().to_sec() if rospy.core.is_initialized() else time.time(),
            "type": msg_type,
            "rate_hz": rate_hz,
            "payload": self._to_builtin(payload),
        }
        if tx_buff is not None:
            msg["frame_hex"] = tx_buff.hex()
        self.position_pub.publish(String(data=json.dumps(msg, separators=(',', ':'))))
        self.position_seq += 1

    def _drone_field_callback(self, msg):
        try:
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = list(points)
            if len(points) == 0:
                return

            # 取第一个点（单点）
            x, y, z = points[0]

            self._drone_field_xyz = [float(x), float(y), float(z)]
            if self.my_color == "Blue":
                self._drone_field_xyz[0] = 28 - self._drone_field_xyz[0]
                self._drone_field_xyz[1] = 15 - self._drone_field_xyz[1]
            self._drone_field_timestamp = msg.header.stamp.to_sec()

        except Exception as e:
            self.logger.log(f"[ROS] /drone_field_xyz 解析错误: {e}")

    def _jam_key_callback(self, msg):
        try:
            self._jam_key = ''.join(reversed(msg.data))
            self.logger.log(f"received jam key: {msg.data}, send jam key: {self._jam_key}")
            self._jam_key_timestamp = rospy.Time.now().to_sec()

        except Exception as e:
            self.logger.log(f"[ROS] /radar/enemy/jam_key 解析错误: {e}")

    def _vision_state_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.logger.log(f"[ROS] {self.state_topic} JSON解析错误: {exc}")
            return
        self.update_enemy_car_infos(payload.get("enemy_car_infos", []))
        self.update_our_car_infos(payload.get("our_car_infos", []))
        self.update_sentinel_alert_info(payload.get("sentinel_alert_info", []))

    def _receiver_state_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.logger.log(f"[ROS] {self.receiver_state_topic} JSON解析错误: {exc}")
            return

        state = payload.get("state", {})
        event_payload = payload.get("payload", {})
        event_type = payload.get("type", "")

        if "my_health" in state:
            health = state["my_health"]
            if isinstance(health, list) and len(health) >= 8:
                self.my_health_info = [int(v) for v in health[:8]]

        if "mark_progress" in state:
            mark_progress = state["mark_progress"]
            if isinstance(mark_progress, list) and len(mark_progress) >= 6:
                self.mark_progress = [int(v) for v in mark_progress[:6]]

        if "have_double_effect_times" in state:
            self.have_double_effect_times = int(state["have_double_effect_times"])

        if "is_activating_double_effect" in state:
            self.update_receiver_is_activating_double_effect_flags(state["is_activating_double_effect"])

        if "time_left" in state:
            time_left = int(state["time_left"])
            if time_left != self.time_left:
                self.time_left = time_left
                self.logger.log(f"update time left{self.time_left}")

        if event_type == "dart_target" and "dart_target" in event_payload:
            index = int(event_payload["dart_target"])
            if 0 <= index < len(self.dart_target_times):
                self.update_receiver_dart_target(index)

        if "interference_level" in state:
            self.interferance_level = int(state["interference_level"])

        if "is_key_update" in state:
            self.is_key_update = int(state["is_key_update"])

    def get_drone_field_xyz(self):
        if self.my_color == "Blue":
            default_xyz = [0.5, 14.5, 1]  # 敌方停机坪坐标
        else:
            default_xyz = [27.5, 0.5, 1]  # 敌方停机坪坐标
        if self._drone_field_xyz is not None:
            return self._drone_field_xyz.copy()
        return default_xyz

    def get_jam_key(self):
        return self._jam_key

    def is_drone_data_fresh(self, timeout=1.0):
        if self._drone_field_xyz is None:
            return False
        return (rospy.Time.now().to_sec() - self._drone_field_timestamp) < timeout

    def _health_array_callback(self, msg):
        try:
            self._enemy_health_array = [int(v) for v in msg.data]
            self._health_timestamp = rospy.Time.now().to_sec()
        except Exception as e:
            self.logger.log(f"[ROS] /radar/enemy/health_array 解析错误: {e}")

    def get_enemy_health_from_ros(self):
        return self._enemy_health_array.copy() if self._enemy_health_array is not None else None

    def is_health_data_fresh(self, timeout=2.0):
        if self._enemy_health_array is None:
            return False
        return (rospy.Time.now().to_sec() - self._health_timestamp) < timeout

    # 更新剩余时间
    def update_time_left(self):
        return

    # 更新敌方车辆信息
    def update_enemy_car_infos(self, enemy_car_infos):
        # 如果为空，直接返回
        self.enemy_car_infos = enemy_car_infos
        self.parse_ene_hero_xyz()
        self.parse_engine_xyz()
        # self.update_enemy(enemy_car_infos)
        # print(f"enemy car info{self.enemy_car_infos}")
        # self.logger.log(f"update enemy car infos{self.enemy_car_infos}")

    def update_enemy(self, enemy_infos):
        for enemy_info in enemy_infos:
            track_id, car_id, field_xyz, is_valid = enemy_info[0], enemy_info[1], enemy_info[4], enemy_info[6]
            if field_xyz == []:
                continue
            else:
                x, y = field_xyz[0], field_xyz[1]
                if car_id in self.predictor.keys():
                    self.predictor[car_id].update(x, y)
                else:
                    continue

    # 更新我方车辆信息
    def update_our_car_infos(self, our_car_infos):
        self.our_car_infos = our_car_infos
        self.parse_hero_xyz()

    # 更新我方无人机的坐标（已弃用，改为 ROS 订阅 /drone_field_xyz）
    def update_our_drone_info(self, our_drone_info):
        self.our_drone_info = our_drone_info

    # 更新敌方无人机的坐标（已弃用，改为 ROS 订阅 /drone_field_xyz）
    def update_enemy_drone_info(self, enemy_drone_info):
        self.enemy_drone_info = enemy_drone_info

    # 更新哨兵预警信息
    def update_sentinel_alert_info(self, sentinel_alert_info):
        # print("update",sentinel_alert_info)
        self.sentinel_alert_info = sentinel_alert_info

    # 新版本发送车辆位置，一次性发送全部车辆，需要补全
    def send_map(self, infos):
        tx_buff = self.sender.generate_all_location_info(infos)
        self.publish_position(
            "map",
            {
                "positions": infos,
                "enemy_ids": self.enemy_id,
                "our_ids": self.my_cars_id,
            },
            rate_hz=self.map_hz,
            tx_buff=tx_buff,
        )
        self.logger.log(f'Sent map info: {infos}')

    # 发送哨兵预警角信息
    def send_sentry_alert_angle(self):

        sentinel_alert_info = self.sentinel_alert_info
        # print("alert info",sentinel_alert_info)
        if sentinel_alert_info == []:
            return
        carID, distance, quadrant = sentinel_alert_info
        tx_buff = self.sender.generate_sentinel_alert_info(carID, distance, quadrant)
        self.publish_position(
            "sentinel_alert",
            {
                "car_id": carID,
                "distance": distance,
                "quadrant": quadrant,
            },
            tx_buff=tx_buff,
        )
        self.logger.log(f'Sent sentinel_alert_info {sentinel_alert_info}')
        # print("send_sentinel_alert_info")

    def send_sentry_perception(self, map_infos):
        # 构造哨兵全局感知数据: 6个 [[x, y]] 格式
        # 顺序: 敌方1号, 敌方2号, 敌方3号, 敌方4号, 敌方6号, 敌方7号
        car_infos = []
        
        # 敌方6辆车 (map_infos 索引 0-5 对应 敌方1,2,3,4,6,7号)
        for info in map_infos:
            car_infos.append([float(info[0]), float(info[1])])
        
        tx_buff = self.sender.generate_sentinel_field_info(car_infos)
        self.publish_position(
            "sentry_perception",
            {
                "positions": car_infos,
                "enemy_ids": self.enemy_id,
            },
            rate_hz=self.sentry_hz,
            tx_buff=tx_buff,
        )

    def send_sentinel_enemy_HP(self, enemy_health_info):
        # 构造哨兵敌方HP数据: 5个 [HP] 格式
        # 顺序: 敌方1号, 敌方2号, 敌方3号, 敌方4号, 敌方7号
        if enemy_health_info is None:
            enemy_health_info = [100, 100, 100, 100, 100]
        hp_infos = []
        for hp in enemy_health_info:
            hp_infos.append(int(hp))
        
        tx_buff = self.sender.generate_enemy_HP_info(hp_infos)
        self.publish_position(
            "enemy_hp",
            {
                "hp": hp_infos,
                "enemy_ids": [self.enemy_id[0], self.enemy_id[1], self.enemy_id[2], self.enemy_id[3], self.enemy_id[5]],
            },
            rate_hz=self.enemy_hp_hz,
            tx_buff=tx_buff,
        )

    # 发送哨兵预警英雄信息
    def send_sentinel_alert_hero(self):
        if self.is_alert_hero:
            tx_buff = self.sender.generate_hero_alert_info(self.is_alert_hero)
            self.publish_position(
                "hero_alert",
                {"is_alert": self.is_alert_hero},
                tx_buff=tx_buff,
            )

    def send_hero_assit(self):
        if self.is_assit_hero:
            tx_buff = self.sender.generate_hero_assit_info(self.send_hero_assit_info, self.is_assit_hero)
            self.publish_position(
                "hero_assist",
                {
                    "is_assist": self.is_assit_hero,
                    "info": self.send_hero_assit_info,
                },
                tx_buff=tx_buff,
            )
            self.logger.log("Send Hero Assist")

    def send_secure_our_hero(self,infos):
        self.alert_our_hero(infos)
        if self.secure_our_hero:
            tx_buff = self.sender.generate_alert_hero(self.enemy_distance)
            self.publish_position(
                "secure_our_hero",
                {
                    "secure": self.secure_our_hero,
                    "enemy_distance": self.enemy_distance,
                },
                tx_buff=tx_buff,
            )

    # 更新flag，将 ROS 回调更新的信息解析为本地flag
    def update_flags(self):
        # 更新双倍易伤相关flag
        # self.update_double_effect_flags()
        # 更新标记进度
        self.parse_mark_process()
        # 更新血量信息
        # self.parse_enemy_health_info()

    # 更新血量信息
    def parse_my_health_info(self):
        if self.my_health_info[0] <= 0:
            self.hero_is_dead = True
        else:
            self.hero_is_dead = False

    # 更新标记进度，用单flag太唐了
    def parse_mark_process(self):
        if self.mark_progress[0]:
            self.hero_is_marked = True
        else:
            self.hero_is_marked = False

        if self.mark_progress[1]:
            self.engineer_is_marked = True
        else:
            self.engineer_is_marked = False

        if self.mark_progress[2]:
            self.standard_3_is_marked = True
        else:
            self.standard_3_is_marked = False

        if self.mark_progress[3]:
            self.standard_4_is_marked = True
        else:
            self.standard_4_is_marked = False

        if self.mark_progress[4]:
            self.drone_is_marked = True
        else:
            self.drone_is_marked = False

        if self.mark_progress[5]:
            self.sentinel_is_marked = True
        else:
            self.sentinel_is_marked = False

        # 更新被标记的数量
        self.marked_num = sum([
            self.hero_is_marked,
            self.engineer_is_marked,
            self.standard_3_is_marked,
            self.standard_4_is_marked,
            self.drone_is_marked,
            self.sentinel_is_marked
        ])
        self.logger.log(f'Marked situation: {self.mark_progress}')

    # 根据已发送情况自动标号发双倍易伤
    def auto_send_double_effect_decision(self, analysis_result):
        # self.sender.send_radar_double_effect_info(self.already_activate_double_effect_times + 1)
        # cv2.imshow("map", map_image)
        self.send_double_effect_analysis_result(2, analysis_result)
        self.send_double_effect_analysis_result(1, analysis_result)

    def send_double_effect_analysis_result(self, times=1, analysis_result='000000'):
        tx_buff = self.sender.generate_double_effect_analysis_result_info(times, analysis_result)
        self.publish_position(
            "double_effect_decision",
            {
                "times": times,
                "analysis_result": analysis_result,
            },
            rate_hz=self.double_effect_hz,
            tx_buff=tx_buff,
        )

    # 新双倍易伤发送机制
    def send_double_effect_decision(self):
        #--------------------------------------25赛季自主决策逻辑-----------------------------------------
        """
        # 如果没有双倍易伤机会或正在触发双倍易伤或已用完次数，只更新密钥，不请求双倍易伤
        if self.have_double_effect_times == 0 or self.is_activating_double_effect or self.already_activate_double_effect_times == 2:
            self.sender.send_double_effect_analysis_result_info(0, jam_key)
            # self.logger.log(f"Sent jam key update only (no double effect request): {jam_key}")
            return

        if (self.is_alert_hero and (self.hero_is_marked or self.find_hero_times >= self.send_double_threshold)):
            self.auto_send_double_effect_decision(jam_key)
            if self.hero_is_marked:
                self.auto_send_double_effect_decision(jam_key)
                return
            else:
                pass
        else:
            self.sender.send_double_effect_analysis_result_info(0, jam_key)

        if (self.hero_is_marked or self.standard_3_is_marked or self.standard_4_is_marked) and (self.my_health_info[6] <= 1200 or self.my_health_info[7] <= 4800):
            self.auto_send_double_effect_decision(jam_key)
            self.logger.log(
                f'Sent double effect info: {self.already_activate_double_effect_times + 1} because hero or standard_3 or standard_4 is marked')
            return
        else:
            self.sender.send_double_effect_analysis_result_info(0, jam_key)

        if self.dart_target == 2 or self.dart_target == 3:
            self.auto_send_double_effect_decision(jam_key)
            return
        else:
            self.sender.send_double_effect_analysis_result_info(0, jam_key)

        if self.time_left <= 250 and self.have_double_effect_times != 0 and not self.is_activating_double_effect:
            self.auto_send_double_effect_decision(jam_key)
            return
        else:
            self.sender.send_double_effect_analysis_result_info(0, jam_key)
            self.logger.log(f"Sent jam key update only (time left > {self.send_double_time_threshold}): {jam_key}")
        """

        #--------------------------------------26赛季自主决策逻辑-----------------------------------------

        # 获取最新密钥
        jam_key = self.get_jam_key()
        if jam_key is None:
            jam_key = '123456'  # 默认密钥

        chance = self.is_chance_double_effect()
        if chance:
            self.auto_send_double_effect_decision(jam_key)    
        else:
            self.send_double_effect_analysis_result(0, jam_key)    
    
    def is_chance_double_effect(self): # TODO
        flag = False
        conditions = 0
        if not self.is_activating_double_effect:
            if self.dart_target == 2 or self.dart_target == 3:
                conditions += 1
            if self.my_health_info[6] <= 1200 or self.my_health_info[7] <= 4800:
                conditions += 1
            if self.time_left <= self.send_double_time_threshold and self.have_double_effect_times != 0:
                conditions += 1
        else:
            conditions = 0

        if conditions != 0:
            flag = True

        return flag

    # 根据时间发送自主决策信息
    # 发送双倍易伤信息
    def send_double_effect_times_to_car(self):
        first_car_id = self.my_sentinel_id - 3
        second_car_id = self.my_sentinel_id - 4
        for car_id in (first_car_id, second_car_id):
            tx_buff = self.sender.generate_double_effect_times_to_car(car_id, self.have_double_effect_times)
            self.publish_position(
                "double_effect_times_to_car",
                {
                    "car_id": car_id,
                    "double_effect_times": self.have_double_effect_times,
                },
                tx_buff=tx_buff,
            )

    def run(self):
        # 问题出在这里，阻塞导致效率很低
        self.working_flag = True
        self.logger.log("Messager start")
        main_rate = rospy.Rate(max(self.main_loop_hz, 1.0))
        
        # 可视化
        try:
            map_image = cv2.imread("/home/radar/Radar/code/Hust_Radar/Lidar/RM2026.png")
        except Exception as e:
            # self.logger.log(f"Read map image error: {e}")
            print(f"Read map image error: {e}")
            # 随便创建一个全白的map_image
            map_image = np.ones((480, 640, 3), np.uint8) * 255
            self.status_logger.log(f"image create error {e}")

        while not rospy.is_shutdown() and self.working_flag:
            # 主体代码在这里以下------------------------------------------------
            # interferance_level_index = self.interferance_level
            # try:
            #     self.sender.send_interferance_level_info(self.interferance_level_list[interferance_level_index-1])
            #     self.logger.log(f"Send interferance level: {self.interferance_level_list[interferance_level_index-1]}")
            # except Exception as e:
            #     self.logger.log(f"Send interferance level error: {e}")
            # Receiver 状态由 ROS 回调更新，这里只解析本地 flag。
            try:
                self.update_receiver_info()
                # 解析本地flag，更新flag
                self.update_flags()
            except Exception as e:
                self.logger.log(f"update error{e}")
            # 如果时间不为-1 ， 存剩余时间
            if self.time_left != -1:
                # self.logger.log(f"Time left: {self.time_left}")
                pass
            # 更新英雄预警
            # show_map_image = copy.deepcopy(map_image)
            # 发送自主决策信息（密钥从 ROS 订阅 /radar/enemy/jam_key 自动获取）
            is_skip, self.last_send_double_effect_time = Tools.frame_control_skip(self.double_effect_hz, self.last_send_double_effect_time)
            if not is_skip:
                self.send_double_effect_decision()
                self.logger.log(f"have_double_effect_times: {self.have_double_effect_times} , is_activating_double_effect: {self.is_activating_double_effect} , already_activate_double_effect_times: {self.already_activate_double_effect_times}")
            
            enemy_car_infos = self.enemy_car_infos # 敌方车辆信息
            our_car_infos = self.our_car_infos # 我方车辆信息
            # 优先从 ROS 订阅 /radar/enemy/health_array 获取血量，UDP 作为后备
            ros_health = self.get_enemy_health_from_ros()
            if ros_health is not None and self.is_health_data_fresh(timeout=2.0):
                # ROS 数据有效: [hero, engineer, infantry_3, infantry_4, reserved, sentry]
                # 转换为 Messager 内部格式 [hero, engineer, infantry_3, infantry_4, sentry] (5个元素)
                self.enemy_health_info = [
                    ros_health[0],  # hero
                    ros_health[1],  # engineer
                    ros_health[2],  # infantry_3
                    ros_health[3],  # infantry_4
                    ros_health[5]   # sentry
                ]
                # self.logger.log(f"[ROS] Enemy health from health_array: {self.enemy_health_info}")
            else:
                self.enemy_health_info = [100, 100, 100, 100, 100]  # 默认值，或保持之前的值
                # self.logger.log(f"[ROS] Enemy health data not fresh, using default or previous value: {self.enemy_health_info}")

            for i in range(12):
                self.send_map_info_is_latest[i] -= 1 if self.send_map_info_is_latest[i] > 0 else 0
            # self.logger.log(f"life : {self.send_map_info_is_latest}")

            for i, life_time in enumerate(self.send_map_info_is_latest[:6]):
                if life_time <= 0:
                    if i == 0:
                        result = self.hero_predictor.get_result()
                        self.send_map_infos[i] = result if (result and len(result) == 2) else [0.0, 0.0]
                        # TODO: 写英雄工程预测
                        # self.send_map_infos[i] = [4.6, 12.0] # 武工程
                    elif i == 1:
                        result = self.engine_predictor.get_result()
                        self.send_map_infos[i] = result if (result and len(result) == 2) else [0.0, 0.0]
                    elif i == 4:
                        # TODO：无人机y轴固定，其他按照雷达扫描数据发送
                        # 索引4=无人机，由ROS订阅实时更新，降级时使用最后已知坐标或[0,0]
                        drone_xyz = self.get_drone_field_xyz()
                        self.send_map_infos[i] = [drone_xyz[0], drone_xyz[1]]
                    else:
                        self.send_map_infos[i] = self.send_map_infos_bkp.copy()
            for i in range(6, 12):
                if self.send_map_info_is_latest[i] <= 0:
                    if i == 10:  # 我方6号无人机
                        # 我方无人机坐标使用停机坪坐标
                        if self.my_color == "Blue":
                            self.send_map_infos[i] = [27.5, 0.5]
                        else:
                            self.send_map_infos[i] = [0.5, 14.5]

            for enemy_car_info in enemy_car_infos:
                _, car_id, field_xyz, is_valid = enemy_car_info[0], enemy_car_info[1], enemy_car_info[4], enemy_car_info[6]
                if field_xyz == [] or is_valid == False:
                    continue
                x, y = field_xyz[0], field_xyz[1]
                x = max(0, min(x, 28))
                y = max(0, min(y, 15))
                for i, enemy_id in enumerate(self.enemy_id):
                    if car_id == enemy_id:
                        self.send_map_infos[i] = [x, y]
                        self.send_map_info_is_latest[i] = 5
                        break

            for our_car_info in our_car_infos:
                car_id, field_xyz, is_valid = our_car_info[1], our_car_info[4], our_car_info[6]
                if field_xyz == [] or is_valid == False:
                    continue
                x, y = field_xyz[0], field_xyz[1]
                x = max(0, min(x, 28))
                y = max(0, min(y, 15))
                for i, my_car_id in enumerate(self.my_cars_id):
                    if car_id == my_car_id:
                        self.send_map_infos[i + 6] = [x, y]
                        self.send_map_info_is_latest[i + 6] = 5
                        break

            if self.is_next_second():
                self.status_logger.log(
                    f"status record:is activating double effect flag{self.is_activating_double_effect} , enemy health info{self.enemy_health_info} , mark progress{self.mark_progress} , have double effect times{self.have_double_effect_times} , time left{self.time_left} , dart target{self.dart_target}")

            # 打印打包好后的信息
            # print("send_map_infos",self.send_map_infos)
            # 发送 , 采用skip的方式控制发送频率，不用sleep影响主循环频率
            is_skip, self.last_send_map_time = Tools.frame_control_skip(self.map_hz, self.last_send_map_time)
            if not is_skip:
                # p = [14, 7.5]
                # debug_data = [p,p,p,p,p,p]
                self.send_map(self.send_map_infos)
                self.logger.log(f"Sent map infos: {self.send_map_infos}")

            skip_sentry, self.last_send_sentry_time = Tools.frame_control_skip(self.sentry_hz, self.last_send_sentry_time)
            if not skip_sentry:
                self.send_sentry_perception(self.send_map_infos[:6])
                self.send_sentinel_enemy_HP(self.enemy_health_info)

            main_rate.sleep()

        print("messager stop")


def load_config(config_path):
    with open(config_path, encoding='Utf-8', mode='r') as config_file:
        return YAML().load(config_file)


def main():
    cfg = load_config(MAIN_CONFIG_PATH)
    if not rospy.core.is_initialized():
        rospy.init_node('radar_messager', anonymous=True, disable_signals=True)
    messager = Messager(cfg)
    try:
        messager.run()
    finally:
        messager.working_flag = False


if __name__ == "__main__":
    main()
