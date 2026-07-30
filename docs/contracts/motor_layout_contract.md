# Motor layout contract

Upper-v3 slot order is `LF/LR/RF/RR`. The frozen firmware default is mask
`0b0110`: M2 is left and M3 is right; M1/M4 are disabled.

Count eligibility is `enabled & ~encoder_anomaly`; speed eligibility is
`enabled & speed_valid & ~encoder_anomaly`. A disabled wheel is absent, never a
valid zero. Count integration requires at least one accepted wheel on each side.
In 4WD a single failed wheel uses its same-side partner with increased covariance;
a missing whole side does not integrate pose. Session, reset generation or enabled
topology changes reset count baselines. Wheel direction normalization belongs to
firmware.
