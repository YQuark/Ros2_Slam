# ESP01S Control Stutter

## Goal

Fix ESP01S long-press manual control stutter without disabling closed-loop chassis control.

## Confirmed Static Findings

The ESP01S control path has three timing and command-contract risks:

- Browser long-press keepalive is `160ms`, ESP idle-stop is `250ms`, and STM32 command timeout is `300ms`; Wi-Fi or HTTP jitter can exceed the idle-stop window and inject zero commands during a held button.
- Duplicate HTTP commands can return early without refreshing `g_lastHttpCmdMs`, so repeated identical long-press requests may still let the idle-stop watchdog expire.
- Closed-loop left/right buttons reuse raw-wheel values, sending mixed `v/w` commands instead of pure angular velocity commands.

## Acceptance Criteria

- Repeated identical held commands refresh the ESP manual-control watchdog.
- ESP refreshes active UART drive/raw commands comfortably inside the STM32 command timeout.
- Browser keepalive, ESP idle-stop, and STM32 timeout are separated with explicit timing margins.
- Closed-loop left/right commands use `v=0,w=+/-W`; open-loop left/right commands still use opposite raw wheel outputs.
- No ROS or hardware verification is claimed from the mounted workspace.

## Non-Goals

- Do not change STM32 command protocol.
- Do not disable closed-loop wheel speed control.
- Do not change PS2 code in this fix.
