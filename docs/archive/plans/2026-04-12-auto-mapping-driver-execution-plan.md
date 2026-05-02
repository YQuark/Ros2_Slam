# Nav2 Frontier Auto Mapping Execution Plan

## Internal Grade

L: single native implementation lane. The task is bounded to `robot_bringup`,
the shell entrypoint, and documentation.

## Steps

1. Keep manual mapping unchanged when `--auto-drive` is omitted.
2. Add `frontier_explorer.py` under `robot_bringup/scripts`.
3. Install the script from `robot_bringup/CMakeLists.txt`.
4. Add mapping-specific `nav2_mapping_params.yaml`.
5. Pass this package's BT XML path through `robot_bringup/launch/nav2.launch.py`
   because Nav2 Humble rewrites `default_bt_xml_filename` at launch time.
6. Keep mapping BT XML and `plugin_lib_names` in the same contract by using the
   Humble-compatible `PipelineSequence + RateController + ComputePathToPose +
   FollowPath` tree and only listing those plugin libraries.
7. In mapping mode with `auto_mapping_drive:=true`, start Nav2 navigation
   servers with the mapping params file.
8. Start `frontier_explorer.py` instead of the direct `/cmd_vel` driver.
9. Have the explorer subscribe to `/map`, use `map -> base_link` TF, cluster
   frontiers, filter noisy fragments, convert frontier cells into known-free
   approach goals, and send `NavigateToPose` goals.
10. Add low-cadence explorer state logs and enable bridge status logs during
   auto-drive runs.
11. Make the STM32 bridge re-request closed-loop mode while active nonzero
   `/cmd_vel` is present and the reported mode is not closed-loop.
12. Add bridge `cmd_vel_rx`, `drive_tx`, and decoded NACK logs for no-motion
   diagnosis.
13. Fail early when real-base auto-drive has no valid dedicated STM32 serial
   port.
14. Align bridge velocity normalization defaults with the STM32 physical
    normalization contract so `/cmd_vel` values are not amplified by the MCU.
15. Blacklist rejected, failed, or timed-out frontier goals temporarily.
16. Handle Ctrl+C cleanly without a `KeyboardInterrupt` traceback.
17. Run local static checks only.

## Safety Defaults

- Nav2 mapping max linear speed: `0.16 m/s`
- Nav2 mapping max angular speed: `1.70 rad/s`
- Nav2 mapping minimum sampled angular speed: `1.10 rad/s`
- Frontier min distance: `0.45 m`
- Frontier max distance: `3.0 m`
- Frontier approach offset: `0.25 m`
- Frontier goal timeout: `25 s`
- Frontier blacklist TTL: `30 s`
- Frontier goal clearance radius: `0.24 m`
- Startup delay: `5 s`
- Max duration: `180 s`

## Verification Policy

Local verification is limited to static checks because this workspace is a
mounted robot workspace.

当前目录是挂载环境，不能在本地执行，请在远程机器人通过 SSH 执行。

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup stm32_robot_bridge
source install/setup.bash
./launch_scripts/robot.sh mapping lidar --real-base --ekf-base --auto-drive
```

## Rollback

Disable by omitting `--auto-drive`. If needed, revert only the frontier explorer,
mapping Nav2 params, launch auto-drive branch, CMake install entry, and shell
flag changes.
