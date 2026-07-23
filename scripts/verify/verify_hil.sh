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

python3 - <<'PY'
from pathlib import Path
import yaml

report = yaml.safe_load(Path('verification/reports/hil/v0.5.0-rc1.yaml').read_text(encoding='utf-8'))
if report.get('result') != 'PASS':
    raise SystemExit('Physical HIL report is not PASS: verification/reports/hil/v0.5.0-rc1.yaml')
for key in ('upper_commit', 'firmware_commit', 'hardware_revision', 'config_sha256'):
    if report.get(key) in (None, ''):
        raise SystemExit(f'HIL report is missing {key}')
PY
