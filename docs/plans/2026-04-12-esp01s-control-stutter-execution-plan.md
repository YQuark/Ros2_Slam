# ESP01S Control Stutter Execution Plan

## Grade

M: focused ESP01S control timing and browser command-contract fix.

## Plan

1. Inspect ESP01S HTTP command handlers, idle-stop logic, UART refresh logic, and browser long-press code.
2. Increase timing separation so held commands do not get zeroed by normal Wi-Fi jitter.
3. Ensure duplicate accepted HTTP commands still refresh the manual-control watchdog.
4. Keep UART refresh period below STM32 `CMD_TIMEOUT_MS`.
5. Split browser closed-loop turn commands from open-loop raw wheel commands.
6. Provide remote firmware verification guidance without claiming local hardware success.

## Verification Boundary

The ROS workspace is mounted and cannot run hardware or build commands locally. ESP01S firmware must be flashed and tested on the robot from the proper firmware toolchain environment.

## Rollback

If ESP stop latency is unacceptable after this change, reduce `CMD_IDLE_STOP_MS` only after confirming browser keepalive interval and observed `/health.loop_max_gap_ms`.
