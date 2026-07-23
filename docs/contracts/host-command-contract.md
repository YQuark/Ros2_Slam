# Host command contract

Nav2, teleoperation, test and explicitly configured research sources publish candidate
Twists. `robot_control` validates, leases, limits and selects those candidates, then
publishes one `HostMotionCommand` on `chassis/host_motion_command`.

`command_epoch` identifies a Host activation generation. A new epoch is legal only
after Host disable/release and rearm. Within an epoch, `sequence` must advance modulo
2^32. Duplicate or out-of-order ROS commands do not refresh a lease. Header time is
recording metadata; safety leases use the host monotonic clock.

Bridge always maps every Host subsource to Upper-v3 mode 2/`COMMAND_SOURCE_HOST`.
Only the float32 wire target or disabled-to-enabled edge advances wire sequence;
unchanged target frames reuse sequence as duplicate keepalives.
