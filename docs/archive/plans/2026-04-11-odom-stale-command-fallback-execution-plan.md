# Odom Stale Status Command Fallback Execution Plan

## Grade

M: single-file upper-computer safety fix with static verification only in the mounted workspace.

## Plan

1. Inspect `bridge_node.py` odometry integration path.
2. Confirm whether stale STM32 status falls back to command velocity.
3. Change `publish_odom()` so odometry velocity is feedback-only.
4. Hold pose and publish zero twist with high covariance when status is stale.
5. Provide remote robot build and runtime verification commands.

## Verification Boundary

Local build, launch, topic, node, and hardware checks are forbidden by `AGENTS.md` because this workspace is mounted from the robot. Verification must be performed on the remote robot.

## Rollback

Revert the `publish_odom()` change in `src/stm32_robot_bridge/stm32_robot_bridge/bridge_node.py` if downstream behavior requires command-dead-reckoning, but that mode is intentionally unsafe for SLAM/navigation and should only be reintroduced behind an explicit opt-in parameter.
