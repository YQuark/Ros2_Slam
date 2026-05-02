# 2026-04-12 odom geometry contract execution plan

## Implemented Upper-Layer Changes

- Decode STM32 wheel encoder cps and compute `/odom` from explicit wheel geometry.
- Keep `odom_feedback_source:=status_twist` as a rollback/comparison path.
- Expose odom geometry parameters:
  - `wheel_radius`
  - `wheel_track_width`
  - `encoder_cpr`
  - `odom_linear_scale`
  - `odom_angular_scale`
  - `odom_angular_sign`
- Expose `status_log_interval_sec` so the bridge can print wheel/status odom comparison lines without raw serial tools.
- Keep stale STM32 status behavior safe: hold pose rather than integrating command velocity.

## Required Robot-Side Calibration

1. Start the base bridge with status summary logs.
2. Rotate the robot by a known physical angle, preferably 90 degrees.
3. Read the reported `/odom` yaw delta.
4. If odom yaw is too large, increase `BASE_WHEEL_TRACK_WIDTH` or reduce `BASE_ODOM_ANGULAR_SCALE`.
5. If odom yaw direction is reversed, set `BASE_ODOM_ANGULAR_SIGN=-1.0`.

Track-width correction formula:

```text
new_track_width = current_track_width * odom_yaw_delta_deg / physical_yaw_delta_deg
```

Example:

```text
current_track_width = 0.125
physical_yaw_delta_deg = 90
odom_yaw_delta_deg = 130
new_track_width = 0.125 * 130 / 90 = 0.1806
```

Observed PS2 turn sample from 2026-04-12:

```text
physical_yaw_delta_deg ~= 90
logged_odom_yaw_delta_deg ~= 358
recommended BASE_ODOM_ANGULAR_SCALE ~= 90 / 358 = 0.25
equivalent effective track width ~= 0.125 * 358 / 90 = 0.497
```

The equivalent track width is physically suspicious for the current small base, so treat `BASE_ODOM_ANGULAR_SCALE=0.25` as a measured upper-layer compensation while auditing lower-layer wheel pairing, encoder polarity, and same-side wheel speed mismatch.

The follow-up mapping screenshot is substantially closer than the unscaled wheel-cps path and PS2 control no longer feels discontinuous. However, the user clarified that the better reference map is the recent committed repository behavior, which preserves the middle tunnel.

The 2026-04-13 auto-drive logs overturned that temporary conclusion. During
PC-source closed-loop control, `status_twist` and direct wheel-cps geometry
diverged by the exact scale implied by the upper/lower Q15 contract mismatch.

Therefore the corrected default is:

```text
odom_feedback_source = wheel_cps
max_linear = 1.20
max_angular = 19.27
odom_angular_scale = 0.25
```

This aligns `/cmd_vel` encoding and status decoding with the STM32 physical
normalization contract derived from `MAX_CPS`, wheel radius, and track width.
The angular odom scale remains at the measured `0.25` compensation because the
raw wheel-cps geometry still makes RViz/map yaw rotate faster than the real
chassis.
The old status-twist path remains available only for A/B comparison through:

```text
BASE_ODOM_FEEDBACK_SOURCE=status_twist
```

A later static regression check found that the shell entrypoint was still overriding
`./robot.sh mapping lidar --real-base --ekf-base` back to `base_fusion_mode=none`
inside the lidar+real-base branch. That made the runtime odom/TF chain different
from the recent committed reference behavior. The mapping entrypoint now keeps
the default as `none`, but respects an explicit `--ekf-base`.

## Lower-Layer Follow-Up

The firmware should be audited before re-enabling IMU-based straight correction:

- Confirm whether `MPU6050_Data_t.ax_g/ay_g/az_g` are already body-mapped values.
- Compare the static body acceleration axes with the configured `IMU_BODY_*` mapping.
- Disable PS2/ESP dynamic straight balance until the IMU yaw-rate axis and encoder yaw sign are verified.
- Prevent ESP zero commands from repeatedly winning arbitration while PS2 is actively controlling.
- Keep only static side trim for manual straight-line correction until the closed-loop correction is stable.
