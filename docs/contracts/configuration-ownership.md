# Configuration ownership

Canonical upper configuration lives under `src/robot_config/config` and is compiled into
one effective configuration plus SHA-256. Geometry, frames, upper safety timing and
upper estimator parameters are upper-owned. Firmware control/PID/motor parameters are
firmware-owned; the upper calibration bundle stores only the expected firmware identity
and parameter CRC.

Code defaults are development fallbacks, not release facts. A release requires a final
calibration bundle, clean upper commit, non-null real hardware identity and matching
effective-config hash. Runtime nodes may reject configuration but may not rewrite the
canonical bundle.
