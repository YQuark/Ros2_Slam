# End-to-end ownership

Platform API 4 separates Host intent from physical authority. `robot_control` is the sole ROS publisher of `HostMotionCommand`; it selects only among Nav2, teleop, test and research inputs. Firmware `CommandManagement` is the final Host/PS2/ESP/Line/Debug source arbiter, `SafetyManagement` owns physical motion permission, and `MotionControl` alone writes motor output.

The upper computer owns goals, paths, formal odometry, fusion and ROS TF. Firmware owns encoder/IMU measurement facts, direction normalization, local fault stops and motor control. Upper supervision may scale or release only the Host candidate.
