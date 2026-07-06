from communication.Messager import Messager
from detect.Detector import Detector
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

mode = "camera" # "video" or "camera" , 如果纯视频模式选用video,需要播放录制livox mid-70的rosbag获得点云信息
save_video = False # 是否保存视频


if __name__ == "__main__":
    video_path = '/home'  #视频模式下的视频路径
    main_config_path = '/home/rosall/config/main_config.yaml'
    converter_config_path = '/home/rosall/config/converter_config.yaml'
    main_cfg = YAML().load(open(main_config_path, 'r', encoding='utf-8'))

    # 全局变量
    global_my_color = main_cfg['global']['my_color']
    is_debuig = main_cfg['global']['debug']

    # 保存路径
    save_video_path = main_cfg['save_video_path']
    today = time.strftime("%Y%m%d", time.localtime())
    today_video_path = save_video_path + today
    if not os.path.exists(today_video_path):
        os.makedirs(today_video_path)
    video_name = time.strftime("%H%M%S", time.localtime()) + '.mp4'
    video_save_path = today_video_path + '/' + video_name

    logger = RadarLog('main')

    if save_video:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(video_save_path, fourcc, 30, (1920, 1080))
    else:
        video_writer = None


    



















