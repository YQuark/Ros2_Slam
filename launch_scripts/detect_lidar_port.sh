#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DETECT_BASE_PORT_SCRIPT="$SCRIPT_DIR/detect_base_port.sh"
CONFIG_PORT="${1:-}"
BASE_PORT_HINT="${ROBOT_BASE_PORT_HINT:-${BASE_PORT_HINT:-}}"
LIDAR_PORT_HINT="${ROBOT_LIDAR_PORT_HINT:-${LIDAR_PORT_HINT:-}}"

resolve_port() {
    local candidate="$1"
    readlink -f "$candidate" 2>/dev/null || printf '%s\n' "$candidate"
}

declare -A SEEN=()
CANDIDATES=()

add_candidate() {
    local candidate="$1"
    local resolved

    [ -e "$candidate" ] || return 0
    resolved="$(resolve_port "$candidate")"
    [ -e "$resolved" ] || return 0

    if [ -n "${SEEN[$resolved]:-}" ]; then
        return 0
    fi

    SEEN[$resolved]=1
    CANDIDATES+=("$resolved")
}

add_candidate /dev/ydlidar

for port in /dev/ttyUSB*; do
    [ -e "$port" ] || continue
    add_candidate "$port"
done

if [ "${#CANDIDATES[@]}" -eq 0 ]; then
    exit 0
fi

BASE_PORT="$BASE_PORT_HINT"
if [ -z "$BASE_PORT" ] && [ -x "$DETECT_BASE_PORT_SCRIPT" ]; then
    BASE_PORT="$($DETECT_BASE_PORT_SCRIPT 2>/dev/null || true)"
fi

if [ -n "$BASE_PORT" ]; then
    RESOLVED_BASE="$(resolve_port "$BASE_PORT")"
    FILTERED=()
    for port in "${CANDIDATES[@]}"; do
        if [ "$port" != "$RESOLVED_BASE" ]; then
            FILTERED+=("$port")
        fi
    done
    if [ "${#FILTERED[@]}" -gt 0 ]; then
        CANDIDATES=("${FILTERED[@]}")
    fi
fi

if [ -n "$CONFIG_PORT" ]; then
    RESOLVED_CONFIG="$(resolve_port "$CONFIG_PORT")"
    for port in "${CANDIDATES[@]}"; do
        if [ "$port" = "$RESOLVED_CONFIG" ]; then
            printf '%s\n' "$port"
            exit 0
        fi
    done
fi

if [ -n "$LIDAR_PORT_HINT" ]; then
    RESOLVED_HINT="$(resolve_port "$LIDAR_PORT_HINT")"
    for port in "${CANDIDATES[@]}"; do
        if [ "$port" = "$RESOLVED_HINT" ]; then
            printf '%s\n' "$port"
            exit 0
        fi
    done
fi

printf '%s\n' "${CANDIDATES[0]}"
