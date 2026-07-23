# Chassis operation result contract

`/chassis/operation` is the sole public operation endpoint. Bridge-private
`/stm32_bridge/wire_*` services report queue admission only.

| Result | Required evidence | ROS Action terminal |
| --- | --- | --- |
| `QUEUED` | wire queue accepted; protocol exposes no state evidence | succeed |
| `APPLIED` | a new same-session firmware fact confirms the effect | succeed |
| `REJECTED` | local validation, link/session or wire adapter rejected it | abort |
| `CONDITION_NOT_CLEARED` | clear request queued but a new STATUS still has the condition | abort |
| `TIMEOUT` | monotonic deadline expired before required evidence | abort |
| `CANCELLED` | caller stopped waiting | canceled |

Cancellation cannot retract a frame already admitted to the serial queue.
