# Vehicle command authority

`robot_control` arbitrates only ROS Host subsources and is the sole publisher of the
Host candidate. It never decides the vehicle's final physical source.

Firmware `CommandManagement` is the final Host/PS2/ESP/Line/Debug source arbiter.
Firmware `SafetyManagement` is the final physical motion-permission authority.
Firmware `MotionControl` is the sole owner of PID, PWM and motor output.

An upper disable withdraws Host only. Physical stop and active-source facts come from
fresh firmware STATUS; queue admission, ROS publication and a requested zero velocity
are not evidence that the vehicle has stopped.
