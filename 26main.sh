#!/bin/bash
# filename: 26main_gnome.sh

MAIN_NAME="radar_main"
SDK_NAME="SDK"
COUNTER_NAME="drone"
RADIO_NAME="radio"
RADIO_ENV="base"
MAIN_ENV="Radar"
ROS_DISTRO="noetic"
LIDAR_PATH="/root/rm/ws_livox/"
WORKSPACE_PATH="/home/radar/Radar/code/Hust_Radar_2026"
UDP_PATH="/home/radar/Radar/code/All_In_2/All_In"

# 关闭已有的同名终端（可选，通过进程管理）

# 1. 启动 roscore（独立终端）
gnome-terminal --title="roscore" -- bash -c "
    roscore
    exec bash
" &


sleep 2

# 2. 启动 SDK（Livox 雷达驱动）
gnome-terminal --title="Livox SDK" -- bash -c "
    source ${LIDAR_PATH}/devel/setup.bash
    roslaunch livox_ros_driver livox_lidar_rviz.launch
    exec bash
" &

sleep 3

# 3. 启动无人机搜索程序
gnome-terminal --title="Drone Search" -- bash -c '
    eval "$('/home/radar/radioconda/bin/conda' 'shell.bash' 'hook')"
    conda activate '"${MAIN_ENV}"'
    source /opt/ros/'"${ROS_DISTRO}"'/setup.bash
    cd '"${WORKSPACE_PATH}"'
    python3 Counter/init_angle_sender.py --no-record-raw --no-record-world
    exec bash
' &

sleep 2

# # 4. 启动 UDP 接收器
# gnome-terminal --title="UDP Receiver" -- bash -c '
#     eval "$('/home/radar/radioconda/bin/conda' 'shell.bash' 'hook')"
#     conda activate '"${RADIO_ENV}"'
#     cd '"${UDP_PATH}"'
#     ./RX/start.sh
#     exec bash
# ' &
# sleep 3
# gnome-terminal --title="Radio UDP" -- bash -c '
#     eval "$('/home/radar/radioconda/bin/conda' 'shell.bash' 'hook')"
#     conda activate '"${RADIO_ENV}"'
#     source /opt/ros/'"${ROS_DISTRO}"'/setup.bash
#     cd '"${WORKSPACE_PATH}"'
#     python3 Radio/field_info_publisher.py
#     exec bash
# ' &

sleep 2

# 5. 启动主程序
gnome-terminal --title="Main Program" -- bash -c '
    eval "$('/home/radar/radioconda/bin/conda' 'shell.bash' 'hook')"
    conda activate '"${MAIN_ENV}"'
    source /opt/ros/'"${ROS_DISTRO}"'/setup.bash
    cd '"${WORKSPACE_PATH}"'
    python3 26_main.py
    exec bash
' &