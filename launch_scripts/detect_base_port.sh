#!/bin/bash

set -euo pipefail

emit_first=1
passive_probe=0

while [ $# -gt 0 ]; do
    case "$1" in
        --all)
            emit_first=0
            ;;
        --probe)
            passive_probe=1
            ;;
    esac
    shift
done

BASE_PORT_HINT="${ROBOT_BASE_PORT_HINT:-${BASE_PORT_HINT:-/dev/serial0}}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROTOCOL_PROBE="${SCRIPT_DIR}/chassis_protocol_probe.py"

resolve_port() {
    local candidate="$1"
    readlink -f "$candidate" 2>/dev/null || printf '%s\n' "$candidate"
}

add_if_exists() {
    local candidate="$1"
    [ -e "$candidate" ] || return 0
    resolve_port "$candidate"
}

probe_v2_status() {
    local port="$1"
    python3 "$PROTOCOL_PROBE" probe "$port"
}

RESULTS=()
if [ -n "$BASE_PORT_HINT" ]; then
    while IFS= read -r port; do
        [ -n "$port" ] && RESULTS+=("$port")
    done < <(add_if_exists "$BASE_PORT_HINT")
fi

if [ "$BASE_PORT_HINT" != "/dev/serial0" ]; then
    while IFS= read -r port; do
        [ -n "$port" ] && RESULTS+=("$port")
    done < <(add_if_exists "/dev/serial0")
fi

if [ -e "/dev/ttyAMA0" ]; then
    resolved_ama="$(resolve_port "/dev/ttyAMA0")"
    duplicate=0
    for port in "${RESULTS[@]:-}"; do
        if [ "$port" = "$resolved_ama" ]; then
            duplicate=1
            break
        fi
    done
    if [ "$duplicate" -eq 0 ]; then
        RESULTS+=("$resolved_ama")
    fi
fi

if [ "${#RESULTS[@]}" -eq 0 ]; then
    exit 0
fi

if [ "$passive_probe" -eq 1 ]; then
    MATCHED=()
    for port in "${RESULTS[@]}"; do
        matched="$(probe_v2_status "$port" 2>/dev/null || true)"
        if [ -n "$matched" ]; then
            MATCHED+=("$matched")
        fi
    done
    RESULTS=("${MATCHED[@]}")
    if [ "${#RESULTS[@]}" -eq 0 ]; then
        exit 0
    fi
fi

if [ "$emit_first" -eq 1 ]; then
    printf '%s\n' "${RESULTS[0]}"
    exit 0
fi

printf '%s\n' "${RESULTS[@]}"
