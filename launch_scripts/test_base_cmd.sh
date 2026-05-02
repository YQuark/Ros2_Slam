#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${ROS_WS:-/home/robot/ros2_ws}"

# shellcheck source=/home/robot/ros2_ws/launch_scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

DETECT_BASE_PORT_SCRIPT="${SCRIPT_DIR}/detect_base_port.sh"
STOP_ALL_SCRIPT="${SCRIPT_DIR}/stop_all.sh"

BASE_PORT="${BASE_PORT:-/dev/ttyUSB1}"
BASE_TEST_MODE="${BASE_TEST_MODE:-rotate}"
BASE_TEST_LINEAR="${BASE_TEST_LINEAR:-0.08}"
BASE_TEST_ANGULAR="${BASE_TEST_ANGULAR:-0.35}"
BASE_TEST_ANGULAR_SERIES="${BASE_TEST_ANGULAR_SERIES:-0.35,0.7,1.0,1.5,2.0,3.0}"
BASE_TEST_DURATION="${BASE_TEST_DURATION:-1.5}"
BASE_TEST_PUBLISH_HZ="${BASE_TEST_PUBLISH_HZ:-10}"
BASE_TEST_CMD_TIMEOUT="${BASE_TEST_CMD_TIMEOUT:-1.0}"
BASE_TEST_KEEPALIVE="${BASE_TEST_KEEPALIVE:-0.10}"
BASE_TEST_STOP_FIRST="${BASE_TEST_STOP_FIRST:-true}"
BASE_TEST_COUNTDOWN="${BASE_TEST_COUNTDOWN:-5}"
BASE_TEST_LOG_DIR="${BASE_TEST_LOG_DIR:-/tmp/robot_base_test}"

usage() {
    cat <<'EOF'
Usage: ./test_base_cmd.sh [--base-port PORT] [--rotate-only|--with-linear|--calibrate-angular] [--no-stop-first]

Purpose:
  Isolate upper-computer base control only:
    ROS2 /cmd_vel -> stm32_bridge -> STM32 -> motors -> /odom

Defaults:
  rotate-only, angular=0.35rad/s, duration=1.5s, cmd_timeout=1.0s

Environment overrides:
  BASE_TEST_LINEAR=0.08
  BASE_TEST_ANGULAR=0.35
  BASE_TEST_ANGULAR_SERIES=0.35,0.7,1.0,1.5,2.0,3.0
  BASE_TEST_DURATION=1.5
  BASE_TEST_CMD_TIMEOUT=1.0
  BASE_TEST_KEEPALIVE=0.10
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --base-port|--port)
            if [ $# -lt 2 ]; then
                log_error "missing value for $1"
                exit 1
            fi
            BASE_PORT="$2"
            shift 2
            ;;
        --rotate-only)
            BASE_TEST_MODE="rotate"
            shift
            ;;
        --with-linear)
            BASE_TEST_MODE="linear"
            shift
            ;;
        --calibrate-angular)
            BASE_TEST_MODE="angular_sweep"
            shift
            ;;
        --no-stop-first)
            BASE_TEST_STOP_FIRST="false"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            log_error "unknown argument: $1"
            usage
            exit 1
            ;;
    esac
done

ensure_workspace_built
setup_ros_env

mkdir -p "$BASE_TEST_LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
BRIDGE_LOG="${BASE_TEST_LOG_DIR}/bridge_${STAMP}.log"
ODOM_LOG="${BASE_TEST_LOG_DIR}/odom_${STAMP}.log"
CMD_LOG="${BASE_TEST_LOG_DIR}/cmd_${STAMP}.log"

BRIDGE_PID=""
ODOM_PID=""

send_stop() {
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
}

cleanup() {
    send_stop
    if [ -n "$ODOM_PID" ] && kill -0 "$ODOM_PID" 2>/dev/null; then
        kill "$ODOM_PID" 2>/dev/null || true
        wait "$ODOM_PID" 2>/dev/null || true
    fi
    if [ -n "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
        kill "$BRIDGE_PID" 2>/dev/null || true
        wait "$BRIDGE_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

if [ "$BASE_PORT" = "auto" ] && [ -x "$DETECT_BASE_PORT_SCRIPT" ]; then
    detected="$("$DETECT_BASE_PORT_SCRIPT" --probe 2>/dev/null || true)"
    if [ -n "$detected" ]; then
        BASE_PORT="$detected"
    fi
fi

if [ "$BASE_PORT" != "auto" ] && [ ! -e "$BASE_PORT" ]; then
    log_error "base port does not exist: $BASE_PORT"
    exit 1
fi

print_header "Base Command Test"
log_info "This test starts only stm32_bridge and publishes low-speed /cmd_vel."
log_info "Keep the robot lifted or in a clear open area. Press Ctrl-C to abort."
log_info "base_port=${BASE_PORT}"
log_info "mode=${BASE_TEST_MODE} duration=${BASE_TEST_DURATION}s publish_hz=${BASE_TEST_PUBLISH_HZ}"
if [ "$BASE_TEST_MODE" = "angular_sweep" ]; then
    log_info "angular_series=${BASE_TEST_ANGULAR_SERIES}"
fi
log_info "cmd_timeout=${BASE_TEST_CMD_TIMEOUT}s keepalive=${BASE_TEST_KEEPALIVE}s"
log_info "logs: ${BASE_TEST_LOG_DIR}"
echo

if [ "$BASE_TEST_STOP_FIRST" = "true" ] && [ -x "$STOP_ALL_SCRIPT" ]; then
    "$STOP_ALL_SCRIPT" >/dev/null 2>&1 || true
fi

for ((i=BASE_TEST_COUNTDOWN; i>0; i--)); do
    printf 'Starting motion test in %s seconds...\r' "$i"
    sleep 1
done
printf '\n'

ros2 launch stm32_robot_bridge stm32_bridge.launch.py \
    port:="${BASE_PORT}" \
    baudrate:=115200 \
    use_status_yaw:=true \
    status_yaw_mode:=relative \
    odom_feedback_source:=status_twist \
    cmd_timeout:="${BASE_TEST_CMD_TIMEOUT}" \
    drive_keepalive_sec:="${BASE_TEST_KEEPALIVE}" \
    status_log_interval_sec:=0.5 \
    cmd_log_interval_sec:=0.2 >"$BRIDGE_LOG" 2>&1 &
BRIDGE_PID="$!"

log_info "Waiting for stm32_bridge..."
for _ in $(seq 1 20); do
    if ros2 node list 2>/dev/null | grep -qx "/stm32_bridge"; then
        break
    fi
    sleep 0.3
done

if ! ros2 node list 2>/dev/null | grep -qx "/stm32_bridge"; then
    log_error "stm32_bridge did not start. See: $BRIDGE_LOG"
    exit 1
fi

timeout 20s ros2 topic echo /odom >"$ODOM_LOG" 2>&1 &
ODOM_PID="$!"

log_info "Publishing deterministic /cmd_vel sequence..."
python3 "${SCRIPT_DIR}/base_cmd_test_node.py" \
    --mode "${BASE_TEST_MODE}" \
    --linear "${BASE_TEST_LINEAR}" \
    --angular "${BASE_TEST_ANGULAR}" \
    --angular-series "${BASE_TEST_ANGULAR_SERIES}" \
    --duration "${BASE_TEST_DURATION}" \
    --hz "${BASE_TEST_PUBLISH_HZ}" >"$CMD_LOG" 2>&1

send_stop
sleep 1

if kill -0 "$ODOM_PID" 2>/dev/null; then
    kill "$ODOM_PID" 2>/dev/null || true
    wait "$ODOM_PID" 2>/dev/null || true
fi

log_success "Base command test finished."
echo "Bridge log: $BRIDGE_LOG"
echo "Odom log:   $ODOM_LOG"
echo "Cmd log:    $CMD_LOG"
echo
echo "Quantitative summary:"
grep '^SUMMARY ' "$CMD_LOG" || echo "  (no summary lines found)"
if grep -q '^CALIBRATION ' "$CMD_LOG"; then
    echo
    echo "Angular calibration:"
    grep '^CALIBRATION ' "$CMD_LOG"
fi
echo
if grep -q 'status=NO_ODOM' "$CMD_LOG"; then
    log_warn "⚠ 至少一个测试阶段没有收到 /odom，当前测试不能证明底盘已正确执行。"
    log_warn "  优先检查 stm32_bridge 是否真正启动、底盘是否上电、串口是否正确。"
fi
echo "Useful checks:"
echo "  rg 'cmd_vel_rx|drive_tx|status_summary|NACK|mode=' $BRIDGE_LOG"
echo "  tail -80 $BRIDGE_LOG"
