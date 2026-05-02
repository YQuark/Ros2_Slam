# Manual Yaw Hold Isolation Execution Plan

## Grade

M: focused lower-computer control-source gating change.

## Plan

1. Inspect the yaw-hold and straight-balance activation conditions and command-source availability.
2. Add explicit configuration switches for PC, PS2, and ESP01S yaw-hold eligibility.
3. Gate yaw-hold activation by command source while leaving closed-loop wheel speed control unchanged.
4. Enable gentle source-gated straight balance for PS2/ESP01S manual straight commands, preferring IMU gyro yaw rate over encoder yaw rate.
5. Apply a conservative static side trim for observed fixed left drift before increasing dynamic correction gains.
6. Keep PS2 polling, ESP01S HTTP timing, and protocol fields untouched.
7. Record static verification and remote hardware validation commands.

## Verification Boundary

The current workspace is mounted from the robot. Do not build, flash, launch ROS, inspect topics, or run hardware locally. Validation must happen on the robot after firmware flashing.

## Manual Spot Checks

- PS2 forward/back should no longer show repeated absolute-heading tugging, but should receive gentle IMU yaw-rate straight-line assistance.
- ESP01S forward/back should no longer show repeated absolute-heading tugging, but should receive gentle IMU yaw-rate straight-line assistance.
- If the robot still drifts left after flashing, increase `STRAIGHT_SIDE_TRIM` gradually; if it drifts right, reduce it.
- PC/navigation straight motion should still be allowed to use yaw hold.
- Turning commands should still pass through as closed-loop velocity commands.

## Rollback

If autonomous straight tracking regresses, keep manual yaw hold disabled and only revisit `YAW_HOLD_ALLOW_PC` or navigation-side velocity limits. Do not re-enable yaw hold for manual sources without measured evidence.
