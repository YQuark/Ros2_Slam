# Timing budget

All Host leases, supervision ages, rearm quiet windows, limiter `dt`, bridge watchdogs, STATUS and ACK timeouts use monotonic time. ROS time is used only for headers and observation/bag alignment. Configured layers are mux lease 250 ms, bridge command age 150 ms, wire keepalive 50 ms and firmware Host watchdog 200 ms; these are separate defenses and must be measured independently in UART HIL.
