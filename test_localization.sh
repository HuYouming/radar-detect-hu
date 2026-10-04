#!/usr/bin/env bash
# filename: test_localization.sh
#
# 纯视觉定位链路测试启动器: video / camera 两种模式, 不需要 Livox 雷达。
#
# 启动顺序(与 26_main.py + detect.Detector 的既有约定一致):
#   roscore -> 26_main.py(交互式五点标定, 完成后发 /radar/main_ready)
#           -> python3 -m detect.Detector(收到 ready 后开始取图 + YOLO 推理)
#
# 用法:
#   bash test_localization.sh video                 # 用 data/shifan.mp4 测试
#   bash test_localization.sh camera                # 用实体工业相机测试
#   bash test_localization.sh video -v /path/a.mp4  # 换一个视频文件
#   bash test_localization.sh camera -c Blue        # 覆盖己方颜色
#   bash test_localization.sh video --check         # 只检查环境, 不启动
#
# 选项:
#   -v, --video-path PATH   视频模式使用的视频文件(默认 data/shifan.mp4)
#   -c, --color Red|Blue    覆盖 main_config.yaml 的 global.my_color
#       --keep-config       退出后保留被修改的 main_config.yaml(不还原)
#       --no-detector       只跑 roscore + 26_main.py(只验证标定/解算)
#       --duration SEC      运行 SEC 秒后自动退出(默认一直运行到 Ctrl-C)
#       --ready-timeout SEC 等待 /radar/main_ready 的超时(默认 900 秒, 标定要手点)
#       --system-python     不激活 conda 环境, 直接用当前 python3
#       --check             只做环境自检后退出
#   -h, --help              显示本帮助
#
# Ctrl-C 退出时会自动: 杀掉 Detector/26_main、还原 main_config.yaml、
# 并在需要时关掉本脚本启动的 roscore。
#
# 说明: 模式与颜色写在 configs/main_config.yaml(26_main.py 与 Detector 都读它),
# 所以本脚本会在运行期间改这两个字段, 并在退出时还原(用 --keep-config 保留)。
# 视频源通过环境变量 RADAR_VIDEO_PATH 传给两个进程, 不改文件、不动 data/。

set -o pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
CONDA_ENV="${CONDA_ENV:-Radar}"
CONDA_EXE="${CONDA_EXE:-conda}"
READY_TOPIC="/radar/main_ready"
CONFIG_FILE="${REPO_DIR}/configs/main_config.yaml"
DEFAULT_VIDEO="${REPO_DIR}/data/shifan.mp4"
CONFIG_BAK="${REPO_DIR}/configs/.main_config.yaml.testbak"

MODE=""
VIDEO_PATH_OPT=""
COLOR_OPT=""
KEEP_CONFIG=0
WITH_DETECTOR=1
DURATION=0
READY_TIMEOUT=900
USE_CONDA=1
CHECK_ONLY=0

MAIN_PID=""
DETECTOR_PID=""
ROSCORE_PID=""
CONFIG_PATCHED=0
CLEANING=0

log()  { printf '[test-loc] %s\n' "$*" >&2; }
warn() { printf '[test-loc][warn] %s\n' "$*" >&2; }
die()  { printf '[test-loc][error] %s\n' "$*" >&2; exit 1; }

usage() { sed -n '2,33p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

# ---------------------------------------------------------------- 参数解析
while [[ $# -gt 0 ]]; do
    case "$1" in
        video|camera)      MODE="$1"; shift ;;
        -v|--video-path)   VIDEO_PATH_OPT="${2:-}"; shift 2 ;;
        -c|--color)        COLOR_OPT="${2:-}"; shift 2 ;;
        --keep-config)     KEEP_CONFIG=1; shift ;;
        --no-detector)     WITH_DETECTOR=0; shift ;;
        --duration)        DURATION="${2:-0}"; shift 2 ;;
        --ready-timeout)   READY_TIMEOUT="${2:-900}"; shift 2 ;;
        --system-python)   USE_CONDA=0; shift ;;
        --check)           CHECK_ONLY=1; shift ;;
        -h|--help)         usage; exit 0 ;;
        *)                 usage; die "无法识别的参数: $1" ;;
    esac
done

[[ -n "$MODE" ]] || { usage; die "缺少模式参数: 请传入 video 或 camera"; }
if [[ -n "$COLOR_OPT" && "$COLOR_OPT" != "Red" && "$COLOR_OPT" != "Blue" ]]; then
    die "--color 只能是 Red 或 Blue"
fi
if [[ -n "$VIDEO_PATH_OPT" && "$MODE" != "video" ]]; then
    warn "--video-path 在 camera 模式下不生效, 已忽略"
    VIDEO_PATH_OPT=""
fi
if ! [[ "$DURATION" =~ ^[0-9]+$ && "$READY_TIMEOUT" =~ ^[0-9]+$ ]]; then
    die "--duration / --ready-timeout 需要是非负整数"
fi

cd "$REPO_DIR" || die "无法进入仓库目录: $REPO_DIR"
LOG_DIR="${REPO_DIR}/logs/localization_test/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR" || die "无法创建日志目录: $LOG_DIR"

# ---------------------------------------------------------------- 环境准备
[[ -f "$ROS_SETUP" ]] || die "找不到 ROS 环境: $ROS_SETUP (可用 ROS_SETUP=... 覆盖)"
# shellcheck disable=SC1090
source "$ROS_SETUP" || die "source ROS 环境失败: $ROS_SETUP"

if [[ "$USE_CONDA" -eq 1 ]] && command -v "$CONDA_EXE" >/dev/null 2>&1; then
    eval "$("$CONDA_EXE" shell.bash hook)" || warn "conda hook 失败, 继续用当前 python3"
    if "$CONDA_EXE" env list | awk '{print $1}' | grep -qx "$CONDA_ENV"; then
        conda activate "$CONDA_ENV" || warn "conda activate $CONDA_ENV 失败"
        log "python3 = $(command -v python3)  (conda env: $CONDA_ENV)"
    else
        warn "找不到 conda 环境 '$CONDA_ENV', 继续使用当前 python3"
    fi
else
    log "python3 = $(command -v python3)  (未使用 conda)"
fi

# ---------------------------------------------------------------- 环境自检
PY_DEPS="cv2 numpy torch ultralytics open3d ruamel.yaml rospy shapely"
MISSING=""
for dep in $PY_DEPS; do
    python3 -c "import $dep" >/dev/null 2>&1 || MISSING="$MISSING $dep"
done

check_summary() {
    local vp="$DEFAULT_VIDEO"
    log "---------- 环境自检 ----------"
    log "仓库目录   : $REPO_DIR"
    log "模式       : $MODE"
    log "ROS 环境   : $ROS_SETUP"
    if [[ -f /.dockerenv ]] || grep -qa docker /proc/1/cgroup 2>/dev/null; then
        log "运行环境   : Docker 容器"
    fi
    log "python3    : $(command -v python3 || echo '<缺失>')"
    if [[ -n "$MISSING" ]]; then
        warn "python 依赖缺失:$MISSING"
    else
        log "python 依赖: 全部就绪"
    fi
    if [[ "$MODE" == "video" ]]; then
        [[ -n "$VIDEO_PATH_OPT" ]] && vp="$VIDEO_PATH_OPT"
        if [[ -f "$vp" ]]; then
            log "视频文件   : $vp ($(du -h "$vp" 2>/dev/null | cut -f1))"
        else
            warn "视频文件不存在: $vp"
        fi
    else
        log "相机配置   : $REPO_DIR/configs/bin_cam_config.yaml"
        warn "相机模式需要 Hikrobot USB3 相机已插好且当前用户有 USB 权限"
        warn "容器内跑相机: 需要 --device 透传 USB 设备, 例如 --device=/dev/bus/usb"
    fi
    if [[ -z "${DISPLAY:-}" ]]; then
        warn "DISPLAY 为空: 标定点选窗口 / Detector 显示窗口打不开"
        warn "  Docker 里转发 X11: docker run -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ..."
        warn "  宿主机先执行: xhost +local:docker   (或 xhost +local:)"
    else
        log "DISPLAY    : $DISPLAY"
        if [[ -n "${XAUTHORITY:-}" ]]; then
            log "XAUTHORITY : $XAUTHORITY ($([[ -r "$XAUTHORITY" ]] && echo 可读 || echo 不可读))"
        fi
    fi
    log "ROS_MASTER : ${ROS_MASTER_URI:-<默认 http://localhost:11311>}"
    log "ROS_IP/HOST: ${ROS_IP:-${ROS_HOSTNAME:-<未设置, 单机容器内运行无需设置>}}"
    command -v roscore  >/dev/null 2>&1 || warn "PATH 里没有 roscore"
    command -v rostopic >/dev/null 2>&1 || warn "PATH 里没有 rostopic"
    log "日志目录   : $LOG_DIR"
    log "------------------------------"
}
check_summary

if [[ "$CHECK_ONLY" -eq 1 ]]; then
    [[ -n "$MISSING" ]] && exit 1
    exit 0
fi

[[ -z "$MISSING" ]] || die "python 依赖缺失:$MISSING (先修好依赖, 或看 --check 输出)"
if [[ -z "${DISPLAY:-}" ]]; then
    die "没有 DISPLAY 无法进行五点标定交互。Docker 里的做法: 宿主机 xhost +local:docker, 容器用 -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix 启动"
fi

# ---------------------------------------------------------------- 清理钩子
# 先停进程再 wait: 若本脚本自己起了 roscore, 提前 wait 会一直阻塞到它死。
stop_pid() {  # $1=pid $2=名字; 先 TERM, 最多等 5s, 再 KILL
    local pid="$1" name="$2" i
    [[ -n "$pid" ]] || return 0
    kill -0 "$pid" 2>/dev/null || return 0
    log "停止 $name (pid $pid)"
    kill "$pid" 2>/dev/null
    for i in $(seq 1 50); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.1
    done
    warn "$name 未响应 TERM, 强制 KILL"
    kill -9 "$pid" 2>/dev/null
}

cleanup() {
    [[ "$CLEANING" -eq 1 ]] && return
    CLEANING=1
    trap - EXIT INT TERM

    stop_pid "$DETECTOR_PID" "Detector"
    stop_pid "$MAIN_PID" "26_main.py"
    stop_pid "$ROSCORE_PID" "roscore(本脚本启动的)"
    wait >/dev/null 2>&1

    if [[ "$CONFIG_PATCHED" -eq 1 ]]; then
        if [[ "$KEEP_CONFIG" -eq 1 ]]; then
            rm -f "$CONFIG_BAK"
            log "按 --keep-config 保留修改后的 configs/main_config.yaml"
        elif [[ -e "$CONFIG_BAK" ]]; then
            cp -- "$CONFIG_BAK" "$CONFIG_FILE" && log "已还原 configs/main_config.yaml"
            rm -f "$CONFIG_BAK"
        fi
    fi
    log "日志: $LOG_DIR"
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------- 视频源(环境变量传递)
if [[ -n "$VIDEO_PATH_OPT" ]]; then
    [[ -f "$VIDEO_PATH_OPT" ]] || die "视频文件不存在: $VIDEO_PATH_OPT"
    export RADAR_VIDEO_PATH="$(realpath -- "$VIDEO_PATH_OPT")"
    log "视频源 -> $RADAR_VIDEO_PATH (RADAR_VIDEO_PATH)"
fi

# ------------------------------------------------------- 写入/备份 运行配置
if [[ -e "$CONFIG_BAK" ]]; then
    die "发现残留备份 $CONFIG_BAK (上次异常退出?), 请先手动处理"
fi
cp -- "$CONFIG_FILE" "$CONFIG_BAK" || die "备份 main_config.yaml 失败"
CONFIG_PATCHED=1
python3 - "$CONFIG_FILE" "$MODE" "$COLOR_OPT" <<'PY' || die "写入 main_config.yaml 失败"
import re
import sys

path, mode, color = sys.argv[1], sys.argv[2], sys.argv[3]
text = open(path, encoding='utf-8').read()


def replace_key(src, key, value):
    pattern = re.compile(r"(?m)^(\s*%s:\s*).*$" % re.escape(key))
    new_src, count = pattern.subn(lambda m: m.group(1) + value, src, count=1)
    if count != 1:
        raise SystemExit("未找到唯一的配置项: %s" % key)
    return new_src


if mode:
    text = replace_key(text, "MODE", "'%s'" % mode)
if color:
    text = replace_key(text, "my_color", '"%s"' % color)
open(path, 'w', encoding='utf-8').write(text)
print("config patched: MODE=%s my_color=%s" % (mode, color or "<unchanged>"))
PY
log "已设置 configs/main_config.yaml: ctrl.MODE='$MODE'${COLOR_OPT:+ global.my_color='$COLOR_OPT'}"

# ---------------------------------------------------------------- roscore
# 用 ROS 自己判断 master 是否在跑(不用 pgrep: 精简镜像常没装 procps,
# 且跨容器/跨主机时进程不在本容器里, pgrep 看不到)。
if timeout 3 rostopic list >/dev/null 2>&1; then
    log "检测到已有 ROS master (${ROS_MASTER_URI:-http://localhost:11311}), 复用它(退出时不会关闭)"
else
    log "启动 roscore (日志: $LOG_DIR/roscore.log)"
    roscore >"$LOG_DIR/roscore.log" 2>&1 &
    ROSCORE_PID=$!
    for _ in $(seq 1 150); do
        kill -0 "$ROSCORE_PID" 2>/dev/null || die "roscore 启动失败, 见 $LOG_DIR/roscore.log"
        rostopic list >/dev/null 2>&1 && break
        sleep 0.1
    done
    rostopic list >/dev/null 2>&1 || die "roscore 未就绪, 见 $LOG_DIR/roscore.log"
fi

# ---------------------------------------------------------------- 主程序
log "启动 26_main.py (日志: $LOG_DIR/main.log)"
log ">>> 标定: 先弹出 'clear press y else n' 窗口, 按 y 后按终端提示顺序点 5 个点"
python3 26_main.py >"$LOG_DIR/main.log" 2>&1 &
MAIN_PID=$!
log "26_main.py pid=$MAIN_PID"

log "等待 $READY_TOPIC (最多 ${READY_TIMEOUT}s, 标定完成前一直等待)..."
READY_OK=0
deadline=$((SECONDS + READY_TIMEOUT))
while ((SECONDS < deadline)); do
    if ! kill -0 "$MAIN_PID" 2>/dev/null; then
        warn "26_main.py 已退出(标定失败或报错?)"
        break
    fi
    if timeout 2 rostopic echo -n1 "$READY_TOPIC" >/dev/null 2>&1; then
        READY_OK=1
        break
    fi
done

if [[ "$READY_OK" -eq 0 ]]; then
    kill -0 "$MAIN_PID" 2>/dev/null && warn "等待 $READY_TOPIC 超时"
    log "----- main.log 末尾 40 行 -----"
    tail -n 40 "$LOG_DIR/main.log" >&2 || true
    exit 1
fi
log "收到 $READY_TOPIC: 标定完成"

# ---------------------------------------------------------------- 检测进程
if [[ "$WITH_DETECTOR" -eq 1 ]]; then
    log "启动 detect.Detector (日志: $LOG_DIR/detector.log)"
    python3 -m detect.Detector >"$LOG_DIR/detector.log" 2>&1 &
    DETECTOR_PID=$!
    log "Detector pid=$DETECTOR_PID; 'frame' 检测窗口与定位小地图窗口应随后弹出"
else
    log "按 --no-detector 跳过检测进程"
fi

# ---------------------------------------------------------------- 运行等待
log "定位测试运行中, Ctrl-C 退出(会自动清进程并还原配置)。"
main_alive() { kill -0 "$MAIN_PID" 2>/dev/null; }
det_alive()  { [[ -n "$DETECTOR_PID" ]] && kill -0 "$DETECTOR_PID" 2>/dev/null; }

if [[ "$DURATION" -gt 0 ]]; then
    log "--duration ${DURATION}s, 到点自动停止"
    end=$((SECONDS + DURATION))
    while ((SECONDS < end)); do
        main_alive || { warn "26_main.py 已退出"; break; }
        [[ -n "$DETECTOR_PID" ]] && ! det_alive && { warn "Detector 已退出"; break; }
        sleep 1
    done
else
    while main_alive; do
        [[ -n "$DETECTOR_PID" ]] && ! det_alive && { warn "Detector 已退出"; break; }
        sleep 1
    done
    main_alive || warn "26_main.py 已退出"
fi

log "----- main.log 末尾 20 行 -----"
tail -n 20 "$LOG_DIR/main.log" >&2 || true
if [[ -f "$LOG_DIR/detector.log" ]]; then
    log "----- detector.log 末尾 20 行 -----"
    tail -n 20 "$LOG_DIR/detector.log" >&2 || true
fi
exit 0
