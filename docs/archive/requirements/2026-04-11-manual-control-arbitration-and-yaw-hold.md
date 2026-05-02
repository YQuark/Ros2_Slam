# Manual Control Arbitration And Yaw Hold

## Goal

Audit the lower-computer manual control path for PC, PS2, and ESP01S so manual inputs are not unexpectedly masked by host keepalive traffic or by overly aggressive heading correction.

## Observed Problem

During manual-control discussion, the likely risk was that multiple command sources shared the same normalized chassis command path while the PC host could keep sending zero or near-zero commands. If arbitration gives those commands too much authority, PS2 or ESP01S manual commands may appear connected but fail to move the chassis.

Another risk was yaw-hold behavior during human control. Closed-loop wheel speed control is still desirable, but heading hold must not create hard, repeated corrections that fight the operator.

## Acceptance Criteria

- PC query traffic must not be treated as active chassis control.
- PC zero keepalive must not mask non-zero manual input.
- PS2 and ESP01S command paths must remain inspectable as separate sources.
- Heading hold behavior must be evaluated separately for navigation control and human manual control.
- Any firmware change must be validated on real hardware before it is considered accepted.

## Non-Goals

- Do not change ROS topic names or serial protocol fields without a separate contract update.
- Do not claim local hardware verification from the mounted workspace.
- Do not replace scientific root-cause analysis with blind parameter trial.
