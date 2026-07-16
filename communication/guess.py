import json
import os
import time

import rospy
from ruamel.yaml import YAML
from std_msgs.msg import String
'''
guess文件作为一个独立的节点，用来猜点
主要任务是猜英雄和工程的点，对应着1号和2号的点

这个节点一直读取着vision/detect节点查看相对应车是否被检测到以及他们的生命周期
读取receiver/state中的易伤标记位

读取yaml文件中的点

按照以下逻辑进行踩点，如果detect没有检测到，且相对应的生命周期也变成了0
则相对应车猜点传入/guess/point中，供其他节点使用，猜的点是根据yaml文件中的点进行的，猜点的顺序是按照yaml文件中的顺序进行的
同时开始计时，如果一个点猜了4s之后标志位没有变成1，则认为这个点猜错了，继续猜下一个点，如此循环
途中如果detect检测到了相对应的车，则停止猜点，如果满足上述的条件，就重新开始猜点，循环这个过程

其中Messenger也要进行适配，只有当vision/detect没有检测到相对应的车且生命周期结束，才会使用guess/point的点

guess/point的数据格式包括车的名字和在小地图坐标系下的xy坐标，如果说yaml没有相对应的点，guess文件就传入（0，0）

'''


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_YAML = os.path.join(REPO_ROOT, 'configs', 'main_config.yaml')
GUESS_YAML = os.path.join(REPO_ROOT, 'communication', 'guess.yaml')


class Guess:
    def __init__(self):
        self.yaml = YAML(typ='safe')
        self.guess_yaml = self._load_yaml(GUESS_YAML)
        self.main_yaml = self._load_yaml(MAIN_YAML)

        self.color = self.main_yaml['global']['my_color']
        self.points = self._load_points(self.guess_yaml.get(self.color, {}))
        self.seq = 0
        self.timeout_sec = 4.0

        if self.color == "Blue":
            self.targets = {
                "hero": {"car_id": 1, "label": "R1", "mark_index": 0},
                "engineer": {"car_id": 2, "label": "R2", "mark_index": 1},
                "drone": {"car_id": 6, "label": None, "mark_index": 4, "use_life": False},
            }
        else:
            self.targets = {
                "hero": {"car_id": 101, "label": "B1", "mark_index": 0},
                "engineer": {"car_id": 102, "label": "B2", "mark_index": 1},
                "drone": {"car_id": 106, "label": None, "mark_index": 4, "use_life": False},
            }

        self.detected_labels = set()
        self.life_infos = {}
        self.mark_progress = [0, 0, 0, 0, 0, 0]
        self.state = {
            name: {
                "active": False,
                "confirmed": False,
                "point_index": 0,
                "point_start": 0.0,
            }
            for name in self.targets
        }

        detector_cfg = self._load_yaml(os.path.join(REPO_ROOT, 'configs', 'detector_config.yaml'))
        detector_ros_cfg = detector_cfg.get('ros', {})
        communication_cfg = self.main_yaml.get('communication', {})
        self.detect_topic = detector_ros_cfg.get('detect_topic', '/vision/detect')
        self.result_topic = detector_ros_cfg.get('result_topic', '/vision/result')
        self.receiver_state_topic = communication_cfg.get('receiver_state_topic', '/receiver/state')
        self.guess_topic = communication_cfg.get('guess_topic', '/guess/point')

        self.pub = rospy.Publisher(self.guess_topic, String, queue_size=1)
        rospy.Subscriber(self.detect_topic, String, self.detect_callback, queue_size=1)
        rospy.Subscriber(self.result_topic, String, self.result_callback, queue_size=1)
        rospy.Subscriber(self.receiver_state_topic, String, self.receiver_state_callback, queue_size=20)

    def _load_yaml(self, path):
        with open(path, 'r', encoding='utf-8') as file:
            return self.yaml.load(file) or {}

    def _load_points(self, raw_points):
        if isinstance(raw_points, list):
            merged = {}
            for item in raw_points:
                if isinstance(item, dict):
                    merged.update(item)
            raw_points = merged
        if not isinstance(raw_points, dict):
            raw_points = {}

        points = {}
        for name in ("hero", "engineer", "drone"):
            point_list = raw_points.get(name, [])
            if not isinstance(point_list, list):
                point_list = []
            points[name] = [
                [float(point.get("x", 0.0)), float(point.get("y", 0.0))]
                for point in point_list
                if isinstance(point, dict)
            ]
        return points

    def detect_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        labels = set()
        for detection in payload.get("detections", []):
            label = detection.get("label")
            if label and label != "NULL":
                labels.add(str(label))
        self.detected_labels = labels

    def result_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        life_infos = {}
        for info in payload.get("car_life_infos", []):
            try:
                car_id = int(info.get("car_id"))
            except (TypeError, ValueError):
                continue
            life_infos[car_id] = {
                "life_span": int(info.get("life_span", 0)),
                "trust": bool(info.get("trust", False)),
            }
        if life_infos:
            self.life_infos = life_infos

    def receiver_state_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        mark_progress = payload.get("state", {}).get("mark_progress")
        if isinstance(mark_progress, list) and len(mark_progress) >= 2:
            self.mark_progress = [int(v) for v in mark_progress[:6]]

    def _target_detected(self, name):
        label = self.targets[name].get("label")
        return label is not None and label in self.detected_labels

    def _target_life_finished(self, name):
        if not self.targets[name].get("use_life", True):
            return True
        car_id = self.targets[name]["car_id"]
        life_info = self.life_infos.get(car_id)
        if life_info is None:
            return False
        return life_info["life_span"] <= 0 or not life_info["trust"]

    def _target_marked(self, name):
        mark_index = self.targets[name]["mark_index"]
        if mark_index >= len(self.mark_progress):
            return False
        return bool(self.mark_progress[mark_index])

    def _current_point(self, name):
        points = self.points.get(name, [])
        if not points:
            return [0.0, 0.0]
        point_index = self.state[name]["point_index"] % len(points)
        return points[point_index]

    def _step_target(self, name, now):
        target_state = self.state[name]
        target_detected = self._target_detected(name)
        life_finished = self._target_life_finished(name)
        target_marked = self._target_marked(name)

        if target_detected or not life_finished:
            target_state["active"] = False
            target_state["confirmed"] = False
            target_state["point_start"] = 0.0
            return

        if target_state["active"] and target_marked:
            target_state["confirmed"] = True

        if target_state["confirmed"]:
            target_state["active"] = True
            return

        if not target_state["active"]:
            target_state["active"] = True
            target_state["point_start"] = now
            return

        if now - target_state["point_start"] >= self.timeout_sec:
            point_count = max(len(self.points.get(name, [])), 1)
            target_state["point_index"] = (target_state["point_index"] + 1) % point_count
            target_state["point_start"] = now

    def build_payload(self):
        now = time.time()
        points = []
        for name, target in self.targets.items():
            self._step_target(name, now)
            x, y = self._current_point(name)
            points.append({
                "name": name,
                "car_id": target["car_id"],
                "x": x,
                "y": y,
                "active": self.state[name]["active"],
                "confirmed": self.state[name]["confirmed"],
                "point_index": self.state[name]["point_index"],
            })

        payload = {
            "seq": self.seq,
            "stamp": now,
            "points": points,
        }
        self.seq += 1
        return payload

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            payload = self.build_payload()
            self.pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
            rate.sleep()


def main():
    if not rospy.core.is_initialized():
        rospy.init_node('radar_guess', anonymous=True, disable_signals=True)
    Guess().run()


if __name__ == '__main__':
    main()
