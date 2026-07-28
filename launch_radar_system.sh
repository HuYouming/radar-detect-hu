#!/usr/bin/env bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
LIVOX_SETUP="/home/radar/Radar/sdk/devel/setup.bash"
PIDS=()

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "${LIVOX_SETUP}" ]; then
    source "${LIVOX_SETUP}"
fi

cd "${REPO_DIR}"

cleanup() {
    for pid in "${PIDS[@]}"; do
        if kill -0 "${pid}" >/dev/null 2>&1; then
            kill "${pid}" >/dev/null 2>&1 || true
        fi
    done
    wait >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

start_bg() {
    "$@" &
    PIDS+=("$!")
}

roscore >/tmp/radar_roscore.log 2>&1 &
PIDS+=("$!")

until rostopic list >/dev/null 2>&1; do
    sleep 0.2
done

start_bg roslaunch livox_ros_driver livox_lidar_rviz.launch
start_bg python3 Counter/init_angle_sender.py
start_bg python3 -m communication.Receiver
start_bg python3 -m communication.guess
start_bg python3 -m communication.Messager
start_bg python3 -m detect.Detector
start_bg python3 26_main.py
# start_bg ./bbox_job.sh
# start_bg ./radio.sh

wait
