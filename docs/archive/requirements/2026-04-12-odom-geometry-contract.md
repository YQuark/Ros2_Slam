# 2026-04-12 odom geometry contract

## Problem

During lidar mapping, the map rotates or drifts after a straight segment followed by an approximately 90 degree turn. The generated map does not preserve right angles, and RViz shows a large mismatch between the apparent map axes and the odom/map pose.

## Relevant Facts

- Lidar is fixed to `/dev/ttyUSB0`.
- STM32 base is fixed to `/dev/ttyUSB1`.
- The STM32 status payload includes four wheel encoder speeds in counts/s.
- The previous upper-layer bridge integrated odom from STM32 `v_est_q15/w_est_q15`, which hides the physical wheel radius, encoder CPR, track width, and angular sign behind a normalized Q15 contract.
- The STM32 firmware computes yaw rate as `(right_mps - left_mps) / DIFF_TRACK_WIDTH_M`.
- The firmware currently uses `DIFF_TRACK_WIDTH_M=0.1250`.
- Static IMU data reported by ROS showed acceleration dominated by the published Z axis, while the current firmware comment/config says body Z comes from sensor X. This makes firmware yaw/straight correction suspect until verified on the robot.

## Requirement

The upper-layer `/odom` shall be computed from explicit physical geometry by default:

- left/right wheel velocity from four encoder cps values,
- wheel radius,
- encoder counts per wheel revolution,
- effective wheel track width,
- explicit angular sign,
- optional linear/angular scale factors.

The old normalized `status_twist` odom path shall remain available for A/B comparison.

## Acceptance Criteria

- For a measured physical 90 degree rotation, `/odom` yaw delta should be close to 90 degrees after setting the effective wheel track width or angular scale.
- During lidar mapping, `odom->base_link` shall not keep moving when STM32 status is stale.
- Lidar and base serial ports shall not resolve to the same device.
- Runtime validation must be performed on the remote robot, not in this mounted workspace.

## 2026-04-12 Runtime Observation

User-provided bridge logs for an approximately 90 degree PS2 turn showed the following:

- `source=wheel_cps` and STM32 `status` angular velocity matched nearly exactly, so upper-layer Q15 decoding is not the active mismatch.
- Integrating the logged angular velocity samples gives approximately `-358 deg` for a physical turn reported as about `90 deg`.
- The required first-order angular correction is therefore approximately `90 / 358 = 0.25`.
- A follow-up user screenshot and report confirmed the `0.25` correction makes the map much closer than the unscaled wheel-cps path and removes the PS2 stutter feeling during this test.
- The user then clarified that the reference `image2` is the recent committed repository behavior and is still better than the `0.25` wheel-cps map because it preserves the middle tunnel.
- The recent committed behavior used `status_twist` interpretation with `max_angular=1.50`, while the STM32 normalizes against an estimated physical maximum near `19.27 rad/s`. At that time it looked closer to one reference map, but later auto-drive telemetry showed this was an accidental scale mismatch rather than a correct contract.
- A static regression check found that the script path for `mapping lidar --real-base --ekf-base` was accidentally forcing `base_fusion_mode=none`, so the user-requested EKF chain was not actually used during lidar mapping.
- The mapping script must keep its default fusion mode as `none`, but it must not override an explicit `--ekf-base`.
- The log alternates between `src=1` and `src=2`; firmware defines `src=1` as PS2 and `src=2` as ESP. This means ESP is still participating in arbitration during PS2 operation and can contribute to stop/start discontinuities.
- Same-side wheel cps values differ significantly during rotation, for example `L1=2156,L2=1921,R1=-820,R2=-2317`. This makes skid-steer effective geometry different from the nominal `DIFF_TRACK_WIDTH_M=0.1250`.

## 2026-04-13 Auto-Drive Observation

User-provided auto-drive logs showed:

- `auto_mapping_state` was publishing intended commands such as `cmd=(0.000,0.350)`.
- STM32 reported `mode=2 src=3`, proving the PC source was active in closed-loop mode.
- The bridge decoded `status_twist` angular velocity around `30 deg/s`, while direct wheel-cps geometry reported around `385 deg/s` for the same status sample.

This proves the old `status_twist` path is not physically trustworthy with
`max_angular=1.50`: the firmware normalizes `w_est` against the physical chassis
maximum, while the bridge denormalized it against a smaller software limit.

The upper-layer default is therefore changed to:

```text
max_linear = 1.20
max_angular = 19.27
odom_feedback_source = wheel_cps
odom_angular_scale = 0.25
```

These values match the firmware constants approximately:

```text
wheel_radius = 0.0325 m
encoder_cpr = 2340
max_cps = 13800
max_linear ~= 1.20 m/s
max_angular ~= 19.27 rad/s
```

The old `status_twist + 0.50/1.50` behavior remains useful only as a historical
comparison point, not as the corrected contract.

The `odom_angular_scale=0.25` default is kept as the current measured
skid-steer/effective-geometry compensation. Without it, the user observed that
RViz/map yaw rotated much faster than the real chassis.
