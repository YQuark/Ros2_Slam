#!/usr/bin/env python3

import shutil
import subprocess
import time


def run(cmd):
    if shutil.which(cmd[0]) is None:
        return f"{cmd[0]}: unavailable"
    try:
        return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT, timeout=3).strip()
    except subprocess.SubprocessError as exc:
        return f"{cmd[0]}: {exc}"


def main():
    while True:
        print("=== Raspberry Pi monitor ===", flush=True)
        print(run(["vcgencmd", "measure_temp"]), flush=True)
        print(run(["vcgencmd", "get_throttled"]), flush=True)
        print(run(["free", "-h"]), flush=True)
        print(run(["df", "-h", "/"]), flush=True)
        time.sleep(5)


if __name__ == "__main__":
    main()
