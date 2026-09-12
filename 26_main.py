from detect.Video import Video
from detect.Capture import Capture
from Lidar.Converter import Converter 
from Log.Log import RadarLog
from Car.Car import *
import numpy as np
import cv2
import time
from collections import deque
from ruamel.yaml import YAML
import os
import json
import rospy
from std_msgs.msg import String
from Tools.Paths import project_path

mode = "video" # "video" or "camera" , 如果纯视频模式选用video,需要播放录制livox mid-70的rosbag获得点云信息
save_video = False # 是否保存视频
ready_topic = "/radar/main_ready"

def get_new_box(xyxy,xywh):
    '''

    Args:
        box: XYXY

    Returns:NEW BOX [x1,y1,x2,y2]

    '''
    x1,y1,x2,y2 = xyxy
    x ,y ,w1,h1 = xywh
    w = x2-x1
    h = y2-y1
    # 如果（x，y）位于图像下半部分
    if y > 2064 / 2:
        # 计算新的x1和y1
        new_x1 = x1+w/2
        new_y1 = y1+h/2+h/9# 这里是为了更准确的定位车底盘位置

    else:
        new_x1 = x1+w/2
        new_y1 = y1+h/2+h/9
    return [new_x1,new_y1,new_x1,new_y1]


class VisionRosBuffer:
    def __init__(self, detect_topic="/vision/detect", result_topic="/vision/result"):
        self.latest_detection = None
        self.last_consumed_seq = -1
        self.source_size = None
        self.detect_sub = rospy.Subscriber(detect_topic, String, self.detect_callback, queue_size=1)
        self.result_pub = rospy.Publisher(result_topic, String, queue_size=1)

    def detect_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            rospy.logwarn(f"invalid /vision/detect payload: {exc}")
            return
        self.latest_detection = payload
        width = payload.get("source_width")
        height = payload.get("source_height")
        if width and height:
            self.source_size = (int(width), int(height))

    def get_results(self):
        payload = self.latest_detection
        if payload is None:
            return None
        seq = int(payload.get("seq", -1))
        if seq <= self.last_consumed_seq:
            return None
        self.last_consumed_seq = seq
        results = []
        for item in payload.get("detections", []):
            results.append([
                item.get("xyxy", []),
                item.get("xywh", []),
                int(item.get("track_id", -1)),
                item.get("label", "NULL"),
                float(item.get("stamp", payload.get("stamp", time.time()))),
            ])
        return None, results

    def get_seq(self):
        if self.latest_detection is None:
            return -1
        return int(self.latest_detection.get("seq", -1))

    def get_stamp(self):
        if self.latest_detection is None:
            return None
        return float(self.latest_detection.get("stamp", 0.0))

    def publish_result(self, payload):
        self.result_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))


class MessagerStatePublisher:
    def __init__(self, state_topic):
        self.state_pub = rospy.Publisher(state_topic, String, queue_size=1)
        self.seq = 0

    def publish(self, enemy_car_infos, our_car_infos, car_life_infos, vision_seq, vision_stamp):
        payload = {
            "seq": self.seq,
            "stamp": time.time(),
            "vision_seq": vision_seq,
            "vision_stamp": vision_stamp,
            "enemy_car_infos": to_builtin(enemy_car_infos),
            "our_car_infos": to_builtin(our_car_infos),
            "car_life_infos": to_builtin(car_life_infos),
        }
        self.state_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
        self.seq += 1


def to_builtin(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (list, tuple)):
        return [to_builtin(item) for item in value]
    if isinstance(value, dict):
        return {key: to_builtin(item) for key, item in value.items()}
    return value


def add_text(draw_payload, text, point, scale=1.0, color=(0, 255, 122), thickness=1):
    draw_payload["texts"].append({
        "text": text,
        "point": [int(point[0]), int(point[1])],
        "scale": scale,
        "color": list(color),
        "thickness": thickness,
    })


def add_circle(draw_payload, center, radius=5, color=(0, 0, 255), thickness=-1):
    draw_payload["circles"].append({
        "center": [int(center[0]), int(center[1])],
        "radius": radius,
        "color": list(color),
        "thickness": thickness,
    })


if __name__ == '__main__':
    video_path = project_path("data", "shifan.mp4")
    detector_config_path = project_path("configs", "detector_config.yaml")
    binocular_camera_cfg_path = project_path("configs", "bin_cam_config.yaml")
    main_config_path = project_path("configs", "main_config.yaml")
    converter_config_path = project_path("configs", "converter_config.yaml")
    camera_name = "new_cam"
    main_cfg = YAML().load(open(main_config_path, encoding='Utf-8', mode='r'))
    mode = main_cfg.get('ctrl', {}).get('MODE', mode)
    detector_cfg = YAML().load(open(detector_config_path, encoding='Utf-8', mode='r'))
    detector_ros_cfg = detector_cfg.get('ros', {})
    detect_topic = detector_ros_cfg.get('detect_topic', '/vision/detect')
    result_topic = detector_ros_cfg.get('result_topic', '/vision/result')
    detector_process_hz = float(detector_ros_cfg['process_hz'])
    messager_cfg = main_cfg.get('messager', {})
    messager_state_topic = messager_cfg.get('state_topic', '/messager/state')
    messager_enabled = bool(messager_cfg.get('enabled', True))
    if not rospy.core.is_initialized():
        rospy.init_node('radar_vision_main', anonymous=True, disable_signals=True)
    ready_pub = rospy.Publisher(ready_topic, String, queue_size=1, latch=True)
    # 全局变量
    global_my_color = main_cfg['global']['my_color']
    is_debug = main_cfg['global']['is_debug']

    logger = RadarLog("main")

    # 类初始化
    vision_buffer = VisionRosBuffer(detect_topic, result_topic)
    logger.log("vision_buffer init")
    messager_state_pub = MessagerStatePublisher(messager_state_topic)
    logger.log("messager_state_pub init")
    converter = Converter(global_my_color,converter_config_path)  # 传入的是path
    logger.log("converter init")
    carList = CarList(main_cfg)
    logger.log("carList init")

    if mode == "video":
        capture = Video(video_path)
    elif mode == "camera":
        from detect.Capture import Capture
        capture = Capture(binocular_camera_cfg_path,camera_name)
    else:
        print("mode error")
        exit(1)


    # 场地解算初始化
    converter.camera_to_field_init(capture)
    capture.release()
    ready_pub.publish(String(data="ready"))
    logger.log(f"main ready published: {ready_topic}")

    start_time = time.time()
    # fps计算
    N = 10
    fps_queue = deque(maxlen=10)

    # 创建一个空列表来存储所有检测的结果
    all_detections = []

    # 当前帧ID
    frame_id = 1
    counter = 0

    # 可视化小地图绘制queue
    main_rate = rospy.Rate(max(detector_process_hz, 1.0))
    print("enter main loop")
    logger.log('satrt main loop')
    try:
        while not rospy.is_shutdown():
            # 计算fps
            now = time.time()
            fps = 1 / (now - start_time)
            start_time = now
            # 将FPS值添加到队列中
            fps_queue.append(fps)
            # 计算平均FPS
            avg_fps = sum(fps_queue) / len(fps_queue)

            print("fps:",avg_fps)
            draw_payload = {
                "seq": vision_buffer.get_seq(),
                "detect_stamp": vision_buffer.get_stamp(),
                "stamp": time.time(),
                "car_life_infos": [],
                "texts": [],
                "circles": [],
                "lines": [],
            }

            # 获得推理结果
            infer_result = vision_buffer.get_results()
            if infer_result is None:
                main_rate.sleep()
                continue

            # 需要打包一份给carList
            carList_results = []
            debug_results = []  # 用于小地图可视化

            # 确保推理结果不为空且可以解包
            if infer_result is not None:
                _, results = infer_result

                if results is not None:
                    print("results is not none")
                    # 对每个结果进行分析 , 进行目标定位
                    for result in results:

                    # 结果：[xyxy_box, xywh_box , track_id , label ]
                        xyxy_box, xywh_box ,  track_id , label, stamp = result # xywh的xy是中心点的xy

                    # 如果没有分类出是什么车直接跳过
                        if label == "NULL":
                            continue

                    # 获取新xyxy_box , 原来是左上角和右下角，现在想要中心点保持不变，宽高设为原来的一半，再计算一个新的xyxy_box,可封装
                        new_xywh_box = get_new_box(xyxy_box, xywh_box)
                        if is_debug:
                            add_circle(draw_payload, (new_xywh_box[0], new_xywh_box[1]), radius=8, color=(255, 0, 255))
                        center = converter.detection_main(new_xywh_box,t=stamp)
                        center = converter.vision_locator.post_process(center, global_my_color)

                    # 将点转到赛场坐标系下
                        field_xyz = center

                        if is_debug:
                            add_text(
                                draw_payload,
                                "x: {:.2f}y:{:.2f}z:{:.2f}".format(field_xyz[0], field_xyz[1], field_xyz[2]),
                                (xyxy_box[2], xyxy_box[3] + 10),
                            )

                    # 将结果打包
                        carList_results.append([track_id , carList.get_car_id(label) , xywh_box , 1 , center , field_xyz])
                        debug_results.append([center, carList.get_car_id(label)])

                    if is_debug and len(debug_results) > 0:
                        converter.vision_locator.visualize(debug_results)

                    # 将结果传入carList
            carList.update_car_info(carList_results)
            draw_payload["car_life_infos"] = carList.get_life_info()
            all_infos = carList.get_all_info() # 此步不做trust的筛选，留给messager做
            my_car_infos = []
            enemy_car_infos = []
            # result in results:[car_id , center_xy , camera_xyz , field_xyz]
            for all_info in all_infos:
                track_id , car_id , center_xy , camera_xyz , field_xyz , color , is_valid = all_info
                # 将信息分两个列表存储
                if color == global_my_color:
                    if track_id == -1:
                        continue
                    my_car_infos.append(all_info)
                else:
                    enemy_car_infos.append(all_info)
                    if track_id != -1:
                        # 将每个检测结果添加到列表中，增加frame_id作为每一帧的ID
                        all_detections.append([frame_id] + list(all_info))

            if messager_enabled:
                messager_state_pub.publish(
                    enemy_car_infos,
                    my_car_infos,
                    draw_payload["car_life_infos"],
                    vision_buffer.get_seq(),
                    vision_buffer.get_stamp(),
                )

            if is_debug:
                add_text(draw_payload, "fps: {:.2f}".format(avg_fps), (10, 500), scale=0.75)
            vision_buffer.publish_result(to_builtin(draw_payload))
            frame_id += 1
            main_rate.sleep()
    finally:
        print("finally")

        cv2.destroyAllWindows()
