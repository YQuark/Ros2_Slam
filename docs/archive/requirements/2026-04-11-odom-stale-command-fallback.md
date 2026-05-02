# Odom Stale Status Command Fallback

## Goal

Ensure upper-computer odometry reflects STM32 feedback only, not commanded velocity, when the STM32 status stream is stale or unavailable.

## Confirmed Defect

`bridge_node.py` previously initialized odometry velocity from `current_vx/current_wz`, which are derived from the active `/cmd_vel`. It then replaced those values with STM32 feedback only when the latest status packet was within `status_timeout`.

When serial feedback was stale, blocked, or absent, `/odom` could continue integrating command velocity. This creates false pose motion and can corrupt `odom`, EKF, SLAM, `map->odom`, and navigation localization.

## Acceptance Criteria

- `/odom` must not integrate `/cmd_vel` when STM32 feedback is stale.
- When STM32 status is fresh, `/odom` continues to use `feedback_vx/feedback_wz`.
- When STM32 status is stale, `/odom.twist` reports zero commanded measurement and pose is held.
- Stale odometry publishes high covariance so downstream fusion treats it as unreliable.
- No topic names, frame IDs, or protocol payloads are changed.

## Non-Goals

- Do not change STM32 firmware.
- Do not change `/cmd_vel` command handling.
- Do not run ROS, hardware, or build commands in the mounted workspace.
