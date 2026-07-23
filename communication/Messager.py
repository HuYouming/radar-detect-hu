from .Sender import Sender

import copy
import json
import time
from ruamel.yaml import YAML
from Log.Log import RadarLog
from Tools.Tools import Tools

# ROS 导入
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

MAIN_CONFIG_PATH = "./configs/main_config.yaml"


class Messager:
    def __init__(self, cfg):
        messager_cfg = cfg.get('messager', {})
        self.state_topic = messager_cfg.get('state_topic', '/messager/state')
        communication_cfg = cfg.get('communication', {})
        self.receiver_state_topic = communication_cfg.get('receiver_state_topic', '/receiver/state')
        self.guess_topic = communication_cfg.get('guess_topic', '/guess/point')
        self.main_loop_hz = float(messager_cfg['main_loop_hz'])
        self.map_hz = float(messager_cfg['map_hz'])
        self.sentry_hz = float(messager_cfg['sentry_hz'])
        self.enemy_hp_hz = float(messager_cfg['enemy_hp_hz'])
        self.double_effect_hz = float(messager_cfg['double_effect_hz'])
        # ROS 订阅数据缓存
        self._drone_field_xyz = None  # [x, y, z] 赛场坐标系下的无人机坐标
        self._drone_field_timestamp = 0.0

        self.interference_level = 1  # 当前干扰等级，默认1级
        self.is_key_update = 0
        self._jam_key = None  # 干扰波密钥
        self._jam_key_timestamp = 0.0

        # 敌方血量订阅缓存（替代 UDP 获取）
        self._enemy_health_array = None  # [hero, engineer, infantry_3, infantry_4, reserved, sentry]
        self._health_timestamp = 0.0
        # log部分
        self.logger = RadarLog("Messager")
        self.status_logger = RadarLog("Messager_Status")

        # 发送部分
        sender_cfg = copy.deepcopy(cfg)
        self.sender = Sender(sender_cfg)

        # 特殊flag
        # self.super_flag = cfg['others']['is_vs_qd'] # 打青岛大学的特殊flag
        # self.is_vs_hg = cfg['others']['is_vs_hg']
        self.send_double_time_threshold = 240  # 发送双倍易伤的时间阈值，单位秒

        # 数据存储
        self.our_car_infos = []  # 我方车辆信息 , our_car_id , our_center_xy , our_camera_xyz , our_field_xyz , our_color
        self.our_drone_info = [0., 0.] # 我方无人机信息
        self.enemy_car_infos = []  # 敌方车辆信息，enemy_car_id , enemy_center_xy , enemy_camera_xyz , enemy_field_xyz , enemy_color = enemy_car_info
        self.enemy_drone_info = [0., 0.] # 敌方无人机信息
        self.car_life_infos = {}
        self.guess_points = {}
        self.time_left = -1  # 剩余时间
        self.last_time_left = -1  # 上次剩余时间 , 用于判断是否更新

        # 我方颜色
        self.my_color = self.sender.my_color

        # 敌我初始化
        if self.my_color == "Red":  # 红方是1-7 ， 蓝方是101-107
            self.enemy_id = [101, 102, 103, 104, 106, 107]
            self.my_cars_id = [1, 2, 3, 4, 6, 7]
        elif self.my_color == "Blue":
            self.enemy_id = [1, 2, 3, 4, 6, 7]
            self.my_cars_id = [101, 102, 103, 104, 106, 107]

        else:
            print("检查main_config里己方颜色是否大写！")
            exit(0)

        # 时间记录
        self.last_send_double_effect_time = time.time()
        self.last_send_map_time = time.time()
        self.last_send_sentry_time = time.time()
        self.last_send_enemy_hp_time = time.time()
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

        # 发送小地图历史记录
        self.send_map_infos = [[0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.], \
                               [0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.]]  # 哨兵全局感知
        self.send_map_info_is_latest = [0, 0, 0, 0, 0, 0, \
                                        0, 0, 0, 0, 0, 0]  # 是否是最新的小地图信息

        # 赛场状态
        # 双倍易伤相关
        self.have_double_effect_times = 0  # 拥有的双倍易伤次数
        self.is_activating_double_effect = False  # 正在激活双倍易伤
        self.double_effect_request_id = 0  # 0x0121 首字节，局内只能单调加1
        self.double_effect_request_pending = False
        # 标记进度
        self.mark_progress = [0, 0, 0, 0, 0, 0]  # 标记进度,对应对方1，2，3，4，无人机和哨兵
        self.my_health_info = [100, 100, 100, 100, 100, 0, 1500, 5000, 1500, 5000]
        self.enemy_health_info = [100, 100, 100, 100, 100]

        # 飞镖目标
        self.dart_target = 0

        # flag
        self.working_flag = False

        # 初始化 ROS 订阅（非阻塞回调方式）。必须放在状态字段初始化之后，
        # 否则订阅回调可能抢先访问尚未创建的属性。
        self._init_ros_subscribers()

    # 判断是否为下一秒
    def is_next_second(self):
        if self.time_left != self.last_time_left:
            self.last_time_left = self.time_left
            return True
        return False

    # 双倍易伤结束后允许在仍有机会时提交下一次请求。
    def update_receiver_is_activating_double_effect_flags(self, is_active=None):
        if is_active is None:
            return
        is_active = bool(is_active)
        if self.is_activating_double_effect and not is_active:
            self.double_effect_request_pending = False
        self.is_activating_double_effect = is_active

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

            rospy.Subscriber(self.guess_topic, String, self._guess_point_callback, queue_size=5)
            self.logger.log(f"[ROS] 已订阅 {self.guess_topic}")

        except Exception as e:
            self.logger.log(f"[ROS] 订阅初始化失败: {e}")

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
        self.update_car_life_infos(payload.get("car_life_infos", []))

    def _guess_point_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.logger.log(f"[ROS] {self.guess_topic} JSON解析错误: {exc}")
            return

        stamp = float(payload.get("stamp", time.time()))
        guess_points = {}
        for point in payload.get("points", []):
            try:
                car_id = int(point.get("car_id"))
                x = float(point.get("x", 0.0))
                y = float(point.get("y", 0.0))
            except (TypeError, ValueError):
                continue
            guess_points[car_id] = {
                "name": point.get("name", ""),
                "x": x,
                "y": y,
                "active": bool(point.get("active", False)),
                "stamp": stamp,
            }
        self.guess_points = guess_points

    def _receiver_state_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.logger.log(f"[ROS] {self.receiver_state_topic} JSON解析错误: {exc}")
            return

        state = payload.get("state", {})
        if "my_health" in state:
            health = state["my_health"]
            if isinstance(health, list) and len(health) >= 10:
                self.my_health_info = [int(v) for v in health[:10]]

        if "mark_progress" in state:
            mark_progress = state["mark_progress"]
            if isinstance(mark_progress, list) and len(mark_progress) >= 6:
                self.mark_progress = [int(v) for v in mark_progress[:6]]

        if "have_double_effect_times" in state:
            available_times = int(state["have_double_effect_times"])
            if available_times < self.have_double_effect_times:
                self.double_effect_request_pending = False
            self.have_double_effect_times = available_times

        if "is_activating_double_effect" in state:
            self.update_receiver_is_activating_double_effect_flags(state["is_activating_double_effect"])

        if "time_left" in state:
            time_left = int(state["time_left"])
            if time_left != self.time_left:
                self.time_left = time_left
                self.logger.log(f"update time left{self.time_left}")

        if "dart_target" in state:
            dart_target = int(state["dart_target"])
            if 0 <= dart_target <= 3:
                self.dart_target = dart_target

        if "interference_level" in state:
            self.interference_level = int(state["interference_level"])

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

    # 更新敌方车辆信息
    def update_enemy_car_infos(self, enemy_car_infos):
        # 如果为空，直接返回
        self.enemy_car_infos = enemy_car_infos
        # print(f"enemy car info{self.enemy_car_infos}")
        # self.logger.log(f"update enemy car infos{self.enemy_car_infos}")

    # 更新我方车辆信息
    def update_our_car_infos(self, our_car_infos):
        self.our_car_infos = our_car_infos

    def update_car_life_infos(self, car_life_infos):
        life_infos = {}
        for info in car_life_infos:
            try:
                car_id = int(info.get("car_id"))
            except (AttributeError, TypeError, ValueError):
                continue
            life_infos[car_id] = {
                "life_span": int(info.get("life_span", 0)),
                "trust": bool(info.get("trust", False)),
            }
        self.car_life_infos = life_infos

    def is_car_vision_valid(self, car_id):
        for car_info in self.enemy_car_infos + self.our_car_infos:
            if len(car_info) < 7:
                continue
            if car_info[1] == car_id:
                field_xyz = car_info[4]
                is_valid = car_info[6]
                return field_xyz != [] and bool(is_valid)
        return False

    def is_car_life_finished(self, car_id):
        life_info = self.car_life_infos.get(car_id)
        if life_info is not None:
            return life_info["life_span"] <= 0 or not life_info["trust"]

        for car_info in self.enemy_car_infos + self.our_car_infos:
            if len(car_info) >= 7 and car_info[1] == car_id:
                return not bool(car_info[6])
        return False

    def apply_guess_points(self):
        now = time.time()
        for car_id, point in self.guess_points.items():
            if not point.get("active", False):
                continue
            if now - float(point.get("stamp", 0.0)) > 1.0:
                continue
            if self.is_car_vision_valid(car_id):
                continue
            if point.get("name") != "drone" and not self.is_car_life_finished(car_id):
                continue

            x = max(0.0, min(float(point.get("x", 0.0)), 28.0))
            y = max(0.0, min(float(point.get("y", 0.0)), 15.0))
            for i, enemy_id in enumerate(self.enemy_id):
                if car_id == enemy_id:
                    self.send_map_infos[i] = [x, y]
                    self.send_map_info_is_latest[i] = 5
                    break

    # 更新我方无人机的坐标（已弃用，改为 ROS 订阅 /drone_field_xyz）
    def update_our_drone_info(self, our_drone_info):
        self.our_drone_info = our_drone_info

    # 更新敌方无人机的坐标（已弃用，改为 ROS 订阅 /drone_field_xyz）
    def update_enemy_drone_info(self, enemy_drone_info):
        self.enemy_drone_info = enemy_drone_info

    # 新版本发送车辆位置，一次性发送全部车辆，需要补全
    def send_map(self, infos):
        tx_buff = self.sender.generate_all_location_info(infos)
        self.sender.send_info(tx_buff)
        self.logger.log(f'Sent map infos: {infos}')

    def send_sentry_perception(self, map_infos):
        # 构造哨兵全局感知数据: 6个 [[x, y]] 格式
        # 顺序: 敌方1号, 敌方2号, 敌方3号, 敌方4号, 敌方6号, 敌方7号
        car_infos = []
        
        # 敌方6辆车 (map_infos 索引 0-5 对应 敌方1,2,3,4,6,7号)
        for info in map_infos:
            car_infos.append([float(info[0]), float(info[1])])
        
        tx_buff = self.sender.generate_sentinel_field_info(car_infos)
        self.sender.send_info(tx_buff)

    def send_sentinel_enemy_HP(self, enemy_health_info):
        # 构造哨兵敌方HP数据: 5个 [HP] 格式
        # 顺序: 敌方1号, 敌方2号, 敌方3号, 敌方4号, 敌方7号
        if enemy_health_info is None:
            enemy_health_info = [100, 100, 100, 100, 100]
        hp_infos = []
        for hp in enemy_health_info:
            hp_infos.append(int(hp))
        
        tx_buff = self.sender.generate_enemy_HP_info(hp_infos)
        self.sender.send_info(tx_buff)

    def should_request_double_effect(self):
        dart_condition = self.dart_target in (1, 2)
        health_condition = self.my_health_info[6] <= 1200 or self.my_health_info[7] <= 4800
        time_condition = 0 <= self.time_left <= self.send_double_time_threshold
        return dart_condition or health_condition or time_condition

    def send_double_effect_decision(self):
        jam_key = self.get_jam_key()
        if jam_key is None:
            jam_key = '123456'  # 默认密钥

        can_create_request = (
            self.should_request_double_effect()
            and self.have_double_effect_times > 0
            and not self.double_effect_request_pending
        )
        if can_create_request:
            if self.double_effect_request_id >= 0xFF:
                self.logger.log("Double effect request id exhausted")
            else:
                self.double_effect_request_id += 1
                self.double_effect_request_pending = True
                self.logger.log(
                    f"Request double effect id={self.double_effect_request_id}, "
                    f"available={self.have_double_effect_times}"
                )

        # 未产生新请求时也发送当前值，确保请求序号永不回退。
        self.sender.send_double_effect_analysis_result_info(self.double_effect_request_id, jam_key)

    def send_sentry_updates(self):
        skip_sentry, self.last_send_sentry_time = Tools.frame_control_skip(
            self.sentry_hz, self.last_send_sentry_time
        )
        if not skip_sentry:
            self.send_sentry_perception(self.send_map_infos[:6])

        skip_enemy_hp, self.last_send_enemy_hp_time = Tools.frame_control_skip(
            self.enemy_hp_hz, self.last_send_enemy_hp_time
        )
        if not skip_enemy_hp:
            self.send_sentinel_enemy_HP(self.enemy_health_info)

    def run(self):
        # 问题出在这里，阻塞导致效率很低
        self.working_flag = True
        self.logger.log("Messager start")
        main_rate = rospy.Rate(max(self.main_loop_hz, 1.0))
        
        while not rospy.is_shutdown() and self.working_flag:
            # 发送自主决策信息（密钥从 ROS 订阅 /radar/enemy/jam_key 自动获取）
            is_skip, self.last_send_double_effect_time = Tools.frame_control_skip(self.double_effect_hz, self.last_send_double_effect_time)
            if not is_skip:
                self.send_double_effect_decision()
                self.logger.log(
                    f"have_double_effect_times: {self.have_double_effect_times}, "
                    f"is_activating_double_effect: {self.is_activating_double_effect}, "
                    f"request_id: {self.double_effect_request_id}, "
                    f"request_pending: {self.double_effect_request_pending}"
                )
            
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
                self.send_map_info_is_latest[i] -= 1
            # self.logger.log(f"life : {self.send_map_info_is_latest}")

            for i, life_time in enumerate(self.send_map_info_is_latest[:6]):
                if life_time < 0:
                    self.send_map_infos[i] = [0.0, 0.0]
            for i in range(6, 12):
                if self.send_map_info_is_latest[i] < 0:
                    self.send_map_infos[i] = [0.0, 0.0]

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

            self.apply_guess_points()

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

            self.send_sentry_updates()

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
