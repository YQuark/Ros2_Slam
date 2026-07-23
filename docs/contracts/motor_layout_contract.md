# Motor layout contract

Upper-v3 slot order is `LF/LR/RF/RR`. The frozen firmware default is mask `0b0110`: M2 is left and M3 is right; M1/M4 are disabled. Eligibility is `enabled & speed_valid & ~anomaly`. Each side requires at least one eligible wheel. Disabled wheels are absent, not valid zero-speed wheels. Pair consistency is evaluated only with at least two eligible wheels on one side. Session, enabled-mask or eligible-mask changes reset odometry baselines. Wheel direction normalization belongs to firmware.
