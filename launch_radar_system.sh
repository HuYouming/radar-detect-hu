#!/usr/bin/env bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
PIDS=()

is_livox_setup() {
    local candidate="$1"
    local workspace

    if [ ! -f "${candidate}" ]; then
        return 1
    fi
    workspace="$(cd "$(dirname "${candidate}")/.." && pwd)"
    [ -d "${workspace}/src/livox_ros_driver" ] || \
        [ -d "${workspace}/devel/share/livox_ros_driver" ]
}

find_livox_setup() {
    local candidate search_root
    local -a candidates=()

    if [ -n "${LIVOX_SETUP:-}" ]; then
        candidates+=("${LIVOX_SETUP}")
    fi
    if [ -n "${LIVOX_WORKSPACE:-}" ]; then
        candidates+=("${LIVOX_WORKSPACE}/devel/setup.bash")
    fi

    candidates+=(
        "${REPO_DIR}/../ws_livox/devel/setup.bash"
        "${REPO_DIR}/ws_livox/devel/setup.bash"
        "${HOME}/ws_livox/devel/setup.bash"
        "${HOME}/catkin_ws/ws_livox/devel/setup.bash"
    )

    for candidate in "${candidates[@]}"; do
        if is_livox_setup "${candidate}"; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done

    for search_root in "${REPO_DIR}/.." "${HOME}"; do
        while IFS= read -r candidate; do
            if is_livox_setup "${candidate}"; then
                printf '%s\n' "${candidate}"
                return 0
            fi
        done < <(
            find "${search_root}" -maxdepth 5 -type f \
                -path '*/devel/setup.bash' -print 2>/dev/null
        )
    done

    return 1
}

if [ ! -f "${ROS_SETUP}" ]; then
    echo "ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
fi
source "${ROS_SETUP}"

LIVOX_SETUP="$(find_livox_setup || true)"
if [ -z "${LIVOX_SETUP}" ]; then
    echo "Livox workspace not found. Set LIVOX_SETUP or LIVOX_WORKSPACE." >&2
    exit 1
fi
source "${LIVOX_SETUP}"

if ! rospack find livox_ros_driver >/dev/null 2>&1; then
    echo "livox_ros_driver is unavailable after sourcing ${LIVOX_SETUP}" >&2
    exit 1
fi
if ! roslaunch --files livox_ros_driver livox_lidar_rviz.launch >/dev/null 2>&1; then
    echo "livox_lidar_rviz.launch was not found in livox_ros_driver" >&2
    exit 1
fi

if [ "${1:-}" = "--check-paths" ]; then
    echo "ROS setup: ${ROS_SETUP}"
    echo "Livox setup: ${LIVOX_SETUP}"
    echo "Livox package: $(rospack find livox_ros_driver)"
    exit 0
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
