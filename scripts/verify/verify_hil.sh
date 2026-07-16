#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then
    source install/setup.bash
fi
set -u

python3 -m pytest tests/integration/test_pty_fake_stm32.py -v

if ! grep -Fqx 'Overall status: PASS' reports/hil/v0.3.0.md; then
    echo 'Physical HIL report is not PASS: reports/hil/v0.3.0.md' >&2
    exit 1
fi
