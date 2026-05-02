# Manual Yaw Hold Isolation

## Goal

Stop PS2 and ESP01S manual driving from feeling like the chassis is repeatedly correcting heading, while still helping manual straight-line driving stay straight.

## Confirmed Static Finding

`robot_control.c` previously applied `CTRL_USE_YAW_HOLD` to every command source. When PS2 or ESP01S sends a straight manual command, `w_cmd` is near zero and `v_ref` is non-zero, so yaw hold becomes active and injects additional `w_ref` correction.

That correction is appropriate for autonomous PC/navigation straight motion, but it can fight human manual input because the operator is already controlling heading.

However, fully removing all straight-line correction from manual sources is too absolute: the chassis has mechanical asymmetry and still needs a gentle straight assist. Encoder-only straight balance is also insufficient if equal encoder counts do not produce equal real-world wheel travel. The correct separation is:

- PC/navigation: may use absolute yaw hold.
- PS2/ESP01S: should not use absolute yaw hold, but should use actual yaw-rate straight balance, preferring IMU gyro yaw rate and falling back to encoder yaw rate only when IMU is unavailable.
- Fixed single-direction drift should be handled by `STRAIGHT_SIDE_TRIM` before increasing dynamic correction gains.

## Acceptance Criteria

- Closed-loop wheel speed control remains available for PS2 and ESP01S.
- Automatic yaw hold is enabled by default for PC/navigation commands.
- Automatic yaw hold is disabled by default for PS2 and ESP01S manual commands.
- Yaw-rate straight balance is enabled by default for PS2 and ESP01S manual straight commands, preferring IMU gyro rate.
- A conservative positive `STRAIGHT_SIDE_TRIM` is applied for the observed fixed left drift.
- The change is source-gated and explicit in configuration.
- No PS2 polling/protocol changes are included in this fix.
- No local hardware verification is claimed from the mounted workspace.

## Non-Goals

- Do not disable `MODE_CLOSED_LOOP`.
- Do not tune PS2 analog mode or ESP01S HTTP timing in this fix.
- Do not change ROS topic names or STM32 serial protocol payloads.
