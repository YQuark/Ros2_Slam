# Firmware control contract

Upper Protocol v3 is the private adapter boundary between Bridge and firmware.
Firmware owns command-session replay protection, source arbitration, safety permission,
control parameters and motor output. Upper code records firmware commit, hardware
revision and parameter CRC only as compatibility identity; it must not maintain a
second authoritative copy of firmware parameters.

STATUS is the final fact for selected source, ESTOP, fault-stop, line-enabled state,
wheel observations and command ACK. A clear-fault request is applied only after a new
same-session STATUS reports no fault.
