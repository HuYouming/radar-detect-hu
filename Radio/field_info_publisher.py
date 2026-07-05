import rospy
from std_msgs.msg import Header, Float32MultiArray, MultiArrayDimension, String
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import sys
import os
import json
import logging
import datetime

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from Log.Log import RadarLog

# 添加 radar_udp_receiver.py 所在路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from radar_udp_receiver import (
    RadarUDPReceiver,
    CommandType,
    RobotPositions,
    RobotHealths,
    RobotAmmos,
    TeamStatus,
    TeamBuffs,
    JamKey,
    RobotBuffData,
)


class RadarRos1Publisher:
    """雷达数据 ROS1 发布节点"""

    def __init__(self):
        rospy.init_node('radar_ros_publisher', anonymous=True)

        # ========== 初始化文件日志 ==========
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_dir = os.path.join(script_dir, 'logs')
        os.makedirs(self.log_dir, exist_ok=True)

        self.data_logger = logging.getLogger('udp_data')
        self.data_logger.setLevel(logging.INFO)
        self._current_log_date = None
        self._setup_daily_log_handler()

        # ========== 创建发布者 ==========
        # 1. 位置相关话题
        self.pub_positions_array = rospy.Publisher(
            '/radar/enemy/positions_array', Float32MultiArray, queue_size=10)
        self.pub_positions_markers = rospy.Publisher(
            '/radar/enemy/positions_markers', MarkerArray, queue_size=10)

        # 2. 血量相关话题
        self.pub_health_array = rospy.Publisher(
            '/radar/enemy/health_array', Float32MultiArray, queue_size=10)
        self.pub_health_dict = rospy.Publisher(
            '/radar/enemy/health_json', String, queue_size=10)

        # 3. 弹药相关话题
        self.pub_ammo_array = rospy.Publisher(
            '/radar/enemy/ammo_array', Float32MultiArray, queue_size=10)

        # 4. 队伍状态话题
        self.pub_team_status = rospy.Publisher(
            '/radar/enemy/team_status', String, queue_size=10)
        self.pub_module_status = rospy.Publisher(
            '/radar/enemy/module_status', String, queue_size=10)

        # 5. Buff 增益话题
        self.pub_buffs = rospy.Publisher(
            '/radar/enemy/buffs', String, queue_size=10)

        # 6. 干扰密钥话题
        self.pub_jam_key = rospy.Publisher(
            '/radar/enemy/jam_key', String, queue_size=10)

        # 7. 综合信息话题（所有数据聚合）
        self.pub_all_info = rospy.Publisher(
            '/radar/enemy/all_info', String, queue_size=10)

        # 8. 原始帧数据话题
        self.pub_raw_frame = rospy.Publisher(
            '/radar/raw/frame_hex', String, queue_size=10)

        # ========== 初始化 UDP 接收器 ==========
        self.udp_receiver1 = RadarUDPReceiver(
            host="127.0.0.1",
            port=40001,
            shared_enemy_health_list=None,
            shared_enemy_position_list=None
        )

        self.udp_receiver2 = RadarUDPReceiver(
            host="127.0.0.1",
            port=40002,
            shared_enemy_health_list=None,
            shared_enemy_position_list=None,
        )
        # 注册回调
        self.udp_receiver1.register_callback(CommandType.POSITION, self._on_position)
        self.udp_receiver1.register_callback(CommandType.HEALTH, self._on_health)
        self.udp_receiver1.register_callback(CommandType.AMMO, self._on_ammo)
        self.udp_receiver1.register_callback(CommandType.TEAM_STATUS, self._on_team_status)
        self.udp_receiver1.register_callback(CommandType.BUFF, self._on_buff)
        self.udp_receiver2.register_callback(CommandType.JAM_KEY, self._on_jam_key)

        # 启动 UDP 接收
        if not (self.udp_receiver1.start() and self.udp_receiver2.start()):
            rospy.logerr("UDP 接收器启动失败!")
            raise RuntimeError("UDP 接收器启动失败")

        # 定时发布统计信息
        self.stats_timer = rospy.Timer(rospy.Duration(5.0), self._publish_stats)

        self.logger = RadarLog("udp_publisher")

        rospy.loginfo("雷达 ROS1 发布节点已启动")
        rospy.loginfo("UDP 监听: 127.0.0.1:40001 (位置/血量/弹药/状态/Buff),127.0.0.1:40002 (干扰密钥)")
        rospy.loginfo(f"UDP 数据文件日志目录: {self.log_dir}")

    def _setup_daily_log_handler(self):
        """按日期切换日志文件，跨天自动新建"""
        today = datetime.datetime.now().strftime('%Y-%m-%d')
        if self._current_log_date == today:
            return

        # 移除旧的 FileHandler
        for h in list(self.data_logger.handlers):
            if isinstance(h, logging.FileHandler):
                self.data_logger.removeHandler(h)
                h.close()

        log_file = os.path.join(self.log_dir, f'udp_data_{today}.log')
        fh = logging.FileHandler(log_file, mode='a', encoding='utf-8')
        fh.setLevel(logging.INFO)
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        fh.setFormatter(formatter)
        self.data_logger.addHandler(fh)
        self._current_log_date = today

    # ========== 回调函数 ==========

    def _on_position(self, data, addr, seq):
        """处理位置数据"""
        pos_dict = data.get_position_dict()

        # 发布 Float32MultiArray: [hero_x, hero_y, engineer_x, engineer_y, ...]
        msg = Float32MultiArray()
        msg.data = [
            float(pos_dict[1][0]), float(pos_dict[1][1]),   # hero
            float(pos_dict[2][0]), float(pos_dict[2][1]),   # engineer
            float(pos_dict[3][0]), float(pos_dict[3][1]),   # infantry_3
            float(pos_dict[4][0]), float(pos_dict[4][1]),   # infantry_4
            float(pos_dict[6][0]), float(pos_dict[6][1]),   # aerial
            float(pos_dict[7][0]), float(pos_dict[7][1]),   # sentry
        ]
        self.logger.log(f"Received positions: {msg.data}")
        dim = MultiArrayDimension()
        dim.label = "robot_positions"
        dim.size = 6
        dim.stride = 2
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        self.pub_positions_array.publish(msg)

        # 写入文件日志（完整数据）
        self._setup_daily_log_handler()
        self.data_logger.info(
            f"[POSITION] seq={seq} addr={addr} positions={msg.data}"
        )

        # 发布 MarkerArray 用于 RViz 可视化
        marker_array = MarkerArray()
        robot_names = {1: 'hero', 2: 'engineer', 3: 'infantry_3',
                       4: 'infantry_4', 6: 'aerial', 7: 'sentry'}
        colors = {
            1: (1.0, 0.0, 0.0),   # hero - red
            2: (0.0, 1.0, 0.0),   # engineer - green
            3: (1.0, 1.0, 0.0),   # infantry_3 - yellow
            4: (1.0, 0.5, 0.0),   # infantry_4 - orange
            6: (0.0, 0.0, 1.0),   # aerial - blue
            7: (1.0, 0.0, 1.0),   # sentry - purple
        }

        for rid, (x, y) in pos_dict.items():
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"
            marker.ns = "enemy_robots"
            marker.id = rid
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(x) / 1000.0  # mm -> m
            marker.pose.position.y = float(y) / 1000.0
            marker.pose.position.z = 0.5
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 1.0
            r, g, b = colors.get(rid, (0.5, 0.5, 0.5))
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(2.0)
            marker_array.markers.append(marker)

            # 文字标签
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "enemy_labels"
            text_marker.id = rid + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = marker.pose.position.x
            text_marker.pose.position.y = marker.pose.position.y
            text_marker.pose.position.z = 1.2
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = "{} ({},{})".format(robot_names.get(rid, 'unknown'), x, y)
            text_marker.lifetime = rospy.Duration(2.0)
            marker_array.markers.append(text_marker)

        self.pub_positions_markers.publish(marker_array)
        rospy.logdebug("[POS] seq={} positions={}".format(seq, pos_dict))

    def _on_health(self, data, addr, seq):
        """处理血量数据"""
        health_dict = data.get_health_dict()

        # 发布 Float32MultiArray: [hero, engineer, infantry_3, infantry_4, reserved, sentry]
        msg = Float32MultiArray()
        msg.data = [
            float(health_dict[1]),   # hero
            float(health_dict[2]),   # engineer
            float(health_dict[3]),   # infantry_3
            float(health_dict[4]),   # infantry_4
            0.0,                      # reserved/infantry_5
            float(health_dict[7]),   # sentry
        ]
        dim = MultiArrayDimension()
        dim.label = "robot_healths"
        dim.size = 6
        dim.stride = 1
        msg.layout.dim = [dim]
        self.pub_health_array.publish(msg)
        self.logger.log(f"Received health: {msg.data}")

        # 写入文件日志（完整数据）
        self._setup_daily_log_handler()
        self.data_logger.info(
            f"[HEALTH] seq={seq} addr={addr} health={msg.data} "
            f"dict={health_dict}"
        )

        # 发布 JSON 格式字符串
        health_json = {
            'seq': seq,
            'timestamp': rospy.Time.now().to_sec(),
            'health': {
                'hero': health_dict[1],
                'engineer': health_dict[2],
                'infantry_3': health_dict[3],
                'infantry_4': health_dict[4],
                'sentry': health_dict[7],
            }
        }
        json_msg = String()
        json_msg.data = json.dumps(health_json)
        self.pub_health_dict.publish(json_msg)

        rospy.logdebug("[HP] seq={} health={}".format(seq, health_dict))

    def _on_ammo(self, data, addr, seq):
        """处理弹药数据"""
        ammo_dict = data.get_ammo_dict()

        msg = Float32MultiArray()
        msg.data = [
            float(ammo_dict[1]),   # hero
            float(ammo_dict[3]),   # infantry_3
            float(ammo_dict[4]),   # infantry_4
            float(ammo_dict[6]),   # aerial
            float(ammo_dict[7]),   # sentry
        ]
        dim = MultiArrayDimension()
        dim.label = "robot_ammos"
        dim.size = 5
        dim.stride = 1
        msg.layout.dim = [dim]
        self.pub_ammo_array.publish(msg)
        self.logger.log(f"Received ammo: {msg.data}")

        # 写入文件日志（完整数据）
        self._setup_daily_log_handler()
        self.data_logger.info(
            f"[AMMO] seq={seq} addr={addr} ammo={msg.data} "
            f"dict={ammo_dict}"
        )

        rospy.logdebug("[AMMO] seq={} ammo={}".format(seq, ammo_dict))

    def _on_team_status(self, data, addr, seq):
        """处理队伍状态数据"""
        ms = data.module_status

        status_json = {
            'seq': seq,
            'timestamp': rospy.Time.now().to_sec(),
            'remaining_coins': data.remaining_coins,
            'destroy_count': data.destroy_count,
            'module_status': {
                'base_shield': ms.base_shield,
                'outpost_shield': ms.outpost_shield,
                'hero_shield': ms.hero_shield,
                'engineer_shield': ms.engineer_shield,
                'infantry_3_shield': ms.infantry_3_shield,
                'infantry_4_shield': ms.infantry_4_shield,
                'sentry_shield': ms.sentry_shield,
                'base_occupied': ms.base_occupied,
                'outpost_occupied': ms.outpost_occupied,
                'power_rune': ms.power_rune,
                'flyover_buff': ms.flyover_buff,
                'flyover_cooldown': ms.flyover_cooldown,
                'center_buff': ms.center_buff,
                'resource_island_buff': ms.resource_island_buff,
                'power_rune_point': ms.power_rune_point,
                'trapezoid_highland': ms.trapezoid_highland,
                'ring_highland': ms.ring_highland,
            }
        }

        msg = String()
        msg.data = json.dumps(status_json, ensure_ascii=False)
        self.pub_team_status.publish(msg)

        # 单独发布模块状态
        module_msg = String()
        module_msg.data = json.dumps(status_json['module_status'], ensure_ascii=False)
        self.pub_module_status.publish(module_msg)
        self.logger.log(f"Received team status: {msg.data}")

        # 写入文件日志（完整数据）
        self._setup_daily_log_handler()
        self.data_logger.info(
            f"[TEAM_STATUS] seq={seq} addr={addr} status={msg.data}"
        )

        rospy.logdebug("[STATUS] seq={} coins={} destroy={}".format(
            seq, data.remaining_coins, data.destroy_count))

    def _on_buff(self, data, addr, seq):
        """处理 Buff 数据"""
        buff_dict = data.get_buff_dict()

        def buff_to_dict(b):
            return {
                'hp_percent': b.hp_percent,
                'cooling_percent': b.cooling_percent,
                'defense_percent': b.defense_percent,
                'attack_percent': b.attack_percent,
                'hp_value': b.hp_value,
            }

        buff_json = {
            'seq': seq,
            'timestamp': rospy.Time.now().to_sec(),
            'buffs': {
                'hero': buff_to_dict(buff_dict[1]),
                'engineer': buff_to_dict(buff_dict[2]),
                'infantry_3': buff_to_dict(buff_dict[3]),
                'infantry_4': buff_to_dict(buff_dict[4]),
                'sentry': buff_to_dict(buff_dict[7]),
                'aerial': buff_to_dict(buff_dict[6]),
            }
        }

        msg = String()
        msg.data = json.dumps(buff_json, ensure_ascii=False)
        self.pub_buffs.publish(msg)
        self.logger.log(f"Received buffs: {msg.data}")

        # 写入文件日志（完整数据）
        self._setup_daily_log_handler()
        self.data_logger.info(
            f"[BUFF] seq={seq} addr={addr} buffs={msg.data}"
        )

        rospy.logdebug("[BUFF] seq={}".format(seq))

    def _on_jam_key(self, data, addr, seq):
        """处理干扰密钥数据"""
        msg = String()
        msg.data = data.key
        self.pub_jam_key.publish(msg)
        rospy.logdebug("[JAM] seq={} key={}".format(seq, data.key))
        self.logger.log(f"Received jam key: {data.key}")

        # 写入文件日志（完整数据）
        self._setup_daily_log_handler()
        self.data_logger.info(
            f"[JAM_KEY] seq={seq} addr={addr} key={data.key}"
        )

    def _publish_stats(self, event):
        """定时发布统计信息"""
        stats1 = self.udp_receiver1.get_stats()
        stats2 = self.udp_receiver2.get_stats()
        rospy.loginfo("[统计] total_packets={} data={} valid_frames={} error_frames={} success_rate={}".format(
            stats1['total_packets'], stats1['data'], stats1['valid_frames'], stats1['error_frames'], stats1['success_rate']))
        rospy.loginfo("[统计] total_packets={} data={} valid_frames={} error_frames={} success_rate={}".format(
            stats2['total_packets'], stats1['data'], stats2['valid_frames'], stats2['error_frames'], stats2['success_rate']))

        # 写入文件日志
        self._setup_daily_log_handler()
        self.data_logger.info(f"[STATS] port=40001 stats={stats1}")
        self.data_logger.info(f"[STATS] port=40002 stats={stats2}")

    def run(self):
        """主循环"""
        rospy.spin()

    def shutdown(self):
        """清理资源"""
        rospy.loginfo("正在关闭UDP接收器...")
        self.udp_receiver1.stop()
        self.udp_receiver2.stop()


def main():
    node = None
    try:
        node = RadarRos1Publisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("收到中断信号，正在关闭...")
    finally:
        if node:
            node.shutdown()


if __name__ == '__main__':
    main()