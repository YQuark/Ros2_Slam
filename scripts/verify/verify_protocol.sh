#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

python3 -m pytest \
  src/stm32_robot_bridge/test/test_protocol_v2.py \
  tests/protocol/test_firmware_golden_vectors.py \
  -q
