# Manual Control Arbitration And Yaw Hold Execution Plan

## Grade

M: lower-computer control-path audit and targeted firmware adjustment, with local static analysis only.

## Plan

1. Inspect PS2, ESP01S, PC link, and `robot_control.c` command-source arbitration.
2. Separate query/link activity from real motion-control activity.
3. Check whether zero keepalive commands can suppress non-zero manual input.
4. Check whether yaw-hold correction is appropriate for manual operation.
5. Keep closed-loop wheel speed control as the preferred control mode unless a specific failure proves otherwise.
6. Record remote flashing and hardware validation steps for the robot.

## Verification Boundary

The current workspace is mounted from the robot. Local build, flashing, ROS launch, topic checks, and hardware tests are not allowed here. Firmware behavior must be verified on the actual robot after flashing from the proper toolchain environment.

## Rollback

If manual input regresses, revert the specific lower-computer manual-control changes and return to the last known PS2/ESP01S behavior before further tuning.
