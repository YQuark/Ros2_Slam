#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="/home/robot/ros2_ws"

# shellcheck source=/home/robot/ros2_ws/launch_scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
    cat <<'EOF'
用法: ./record_mapping_test.sh <test-label> [--notes "说明"] [--output-dir DIR]

示例:
  ./record_mapping_test.sh baseline
  ./record_mapping_test.sh status-yaw --notes "mapping lidar --real-base --base-use-status-yaw"
EOF
}

escape_json() {
    local value="${1:-}"
    value="${value//\\/\\\\}"
    value="${value//\"/\\\"}"
    value="${value//$'\n'/\\n}"
    printf '%s' "$value"
}

main() {
    local test_label=""
    local notes=""
    local output_root="${HOME}/ros2_test_records"
    local timestamp=""
    local test_dir=""
    local bag_dir=""
    local bag_name="bag"
    local meta_file=""
    local note_file=""
    local branch=""
    local commit=""
    local dirty=""
    local now_iso=""
    local notes_json=""

    if [ $# -eq 0 ]; then
        usage
        exit 1
    fi

    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
    esac

    test_label="$1"
    shift || true

    while [ $# -gt 0 ]; do
        case "$1" in
            --notes)
                if [ $# -lt 2 ]; then
                    log_error "✗ --notes 需要文本内容"
                    exit 1
                fi
                notes="$2"
                shift 2
                ;;
            --output-dir)
                if [ $# -lt 2 ]; then
                    log_error "✗ --output-dir 需要目录路径"
                    exit 1
                fi
                output_root="$2"
                shift 2
                ;;
            *)
                log_error "✗ 未知参数: $1"
                usage
                exit 1
                ;;
        esac
    done

    ensure_workspace_built
    setup_ros_env

    timestamp="$(date +%Y%m%d_%H%M%S)"
    test_label="$(printf '%s' "$test_label" | tr ' /' '__')"
    test_dir="${output_root}/${timestamp}_${test_label}"
    bag_dir="${test_dir}/${bag_name}"
    meta_file="${test_dir}/metadata.json"
    note_file="${test_dir}/TEST_RECORD.md"

    mkdir -p "$test_dir"

    branch="$(git -C "${ROS_WS}" rev-parse --abbrev-ref HEAD 2>/dev/null || printf 'unknown')"
    commit="$(git -C "${ROS_WS}" rev-parse --short HEAD 2>/dev/null || printf 'unknown')"
    if git -C "${ROS_WS}" diff --quiet --ignore-submodules HEAD -- 2>/dev/null; then
        dirty="false"
    else
        dirty="true"
    fi
    now_iso="$(date --iso-8601=seconds)"
    notes_json="$(escape_json "$notes")"

    cat >"${meta_file}" <<EOF
{
  "test_label": "$(escape_json "$test_label")",
  "started_at": "${now_iso}",
  "workspace": "${ROS_WS}",
  "git_branch": "${branch}",
  "git_commit": "${commit}",
  "git_dirty": ${dirty},
  "notes": "${notes_json}",
  "bag_dir": "${bag_dir}",
  "topics": [
    "/scan",
    "/odom",
    "/imu/data",
    "/tf",
    "/tf_static",
    "/odometry/filtered"
  ]
}
EOF

    cat >"${note_file}" <<EOF
# 建图测试记录

## 基本信息

- 标签: \`${test_label}\`
- 开始时间: \`${now_iso}\`
- 工作区: \`${ROS_WS}\`
- 分支: \`${branch}\`
- 提交: \`${commit}\`
- 工作区脏状态: \`${dirty}\`
- 说明: ${notes:-待补充}

## 本次启动命令

\`\`\`bash
# 在这里填本次实际启动命令
\`\`\`

## 测试路线

1. 静止 10 秒
2. 直线前进 2 米
3. 直线后退 2 米
4. 原地左转 90 度，停 2 秒
5. 再左转 90 度
6. 贴墙走矩形一圈
7. 回到起点静止 5 秒

## 观察结果

- 静止漂移:
- 直线横漂:
- 90 度转角:
- 矩形边缘平直度:
- 回到起点闭环误差:
- 是否出现地图抖动或不更新:

## 结论

- 本组是否优于基线:
- 推测问题归属:
- 下一组准备改动:

## 文件位置

- Bag 目录: \`${bag_dir}\`
- 元数据: \`${meta_file}\`
EOF

    print_header "建图测试记录启动"
    log_info "测试目录: ${test_dir}"
    log_info "记录模板: ${note_file}"
    log_info "Bag 输出: ${bag_dir}"
    if [ -n "$notes" ]; then
        log_info "说明: ${notes}"
    fi
    echo
    log_info "开始录制话题: /scan /odom /imu/data /tf /tf_static /odometry/filtered"
    log_warn "停止录制请按 Ctrl+C"
    echo

    cd "${test_dir}"
    exec ros2 bag record \
        -o "${bag_name}" \
        /scan \
        /odom \
        /imu/data \
        /tf \
        /tf_static \
        /odometry/filtered
}

main "$@"
