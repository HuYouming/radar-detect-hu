#!/bin/bash
set -e

PROJECT_ROOT="/home/radar/Radar/code/Radar_ros_2026/26radar/26radar-main"
VISION_DIR="$PROJECT_ROOT/vision/bbox"
ROS1_SETUP="/opt/ros/noetic/setup.bash"

# 1) roscore
# gnome-terminal -- bash -lc "
#     source '$ROS1_SETUP';
#     echo 'Starting roscore...';
#     roscore;
#     exec bash
# "

# sleep 2

# 2) 串口桥: 串口收发 <-> ROS1 话题
gnome-terminal -- bash -lc "
    source '$ROS1_SETUP';
    cd '$VISION_DIR';
    echo 'Starting serial bridge node...';
    python3 serial_bridge_ros1.py --port /dev/ttyUSB1 --baud 115200 --feedback-topic /gimbal/angle_feedback --cmd-topic /gimbal/abs_cmd;
    exec bash
"

sleep 1

# 3) 主控: 订阅反馈角度 -> 计算 -> 发布绝对角命令
# 记得上场删除--debug --view
gnome-terminal -- bash -lc "
    source '$ROS1_SETUP';
    cd '$VISION_DIR';
    echo 'Starting bbox main node...';
    python3 main_radar.py --view --debug --save  --imgsz 640 --max-det 1 --log-interval 1 --process-every-n 1 --max-process-fps 0 --camera-color bgr --feedback-topic /gimbal/angle_feedback --cmd-topic /gimbal/abs_cmd;
    exec bash
"

echo "ROS1 串口桥和主控已启动。"
