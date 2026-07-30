#!/usr/bin/env python3
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
FORBIDDEN = (
    "logs/",
    "log/",
    "colcon.meta",
    "src/YDLidar-SDK/",
    "src/ydlidar_ros2_driver/",
    "src/third_party/robot_localization/",
)
LEGACY_READ_ONLY = ("experiments/legacy/", "launch_scripts/", "reports/")
LEGACY_ALLOWLIST = ROOT / "scripts" / "verify" / "legacy-read-only-files.txt"


def main() -> int:
    result = subprocess.run(
        ["git", "ls-files"], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE
    )
    offenders = [
        path
        for path in result.stdout.splitlines()
        if path == "colcon.meta" or path.startswith(FORBIDDEN)
    ]
    tracked = set(result.stdout.splitlines())
    allowed_legacy = {
        line.strip()
        for line in LEGACY_ALLOWLIST.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("#")
    }
    unexpected_legacy = sorted(
        path for path in tracked if path.startswith(LEGACY_READ_ONLY) and path not in allowed_legacy
    )
    offenders.extend(unexpected_legacy)
    if offenders:
        print("Forbidden tracked paths:", file=sys.stderr)
        for path in offenders:
            print(f"  {path}", file=sys.stderr)
        return 1
    print("Repository boundaries OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
