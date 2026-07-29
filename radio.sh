#!/bin/bash
# filename: radio.sh

IS_TEST=0  # 表示非测试模式
if [ "$1" == "--test" ]; then
    IS_TEST=1  # 表示测试模式
fi

MAIN_ENV="Radar"
ROS_DISTRO="noetic"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDR_PATH="${SDR_PATH:-${REPO_DIR}/Radio/All_In}"
UDP_PATH="${UDP_PATH:-${REPO_DIR}/Radio}"
CONDA_EXE="${CONDA_EXE:-conda}"

# 1. 启动 roscore（独立终端）
if pgrep -x "roscore" > /dev/null; then
    echo "roscore already running, skip."
else
    echo "starting roscore..."
    roscore &
    sleep 1
fi

# 测试用：启动自发程序
if [ $IS_TEST -eq 1 ]; then
    gnome-terminal --title="Test program" -- bash -c '
        cd '"${SDR_PATH}/TX/HackRF"'
        ./Launch_tx.sh
        exec bash
    ' &
    sleep 5
fi

# 2. 启动SDR接收程序
gnome-terminal --title="SDR receiver" -- bash -c '
    eval "$('"${CONDA_EXE}"' shell.bash hook)"
    cd '"${SDR_PATH}/RX"'
    ./Pluto_RX.sh
    exec bash
' &
sleep 5

# 3. 启动主程序
gnome-terminal --title="UDP receriver" -- bash -c '
    eval "$('"${CONDA_EXE}"' shell.bash hook)"
    conda activate '"${MAIN_ENV}"'
    source /opt/ros/'"${ROS_DISTRO}"'/setup.bash
    cd '"${UDP_PATH}"'
    python3 field_info_publisher.py
    exec bash
' &
