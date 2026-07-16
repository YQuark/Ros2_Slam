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


def main() -> int:
    result = subprocess.run(
        ["git", "ls-files"], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE
    )
    offenders = [
        path
        for path in result.stdout.splitlines()
        if path == "colcon.meta" or path.startswith(FORBIDDEN)
    ]
    if offenders:
        print("Forbidden tracked paths:", file=sys.stderr)
        for path in offenders:
            print(f"  {path}", file=sys.stderr)
        return 1
    print("Repository boundaries OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
