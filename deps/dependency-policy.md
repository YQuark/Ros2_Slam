# Dependency Policy

This repository tracks first-party ROS packages, launch files, deployment templates and patches only.

- ROS Humble packages are installed through apt and rosdep.
- `robot_localization` must come from `ros-humble-robot-localization`.
- `ydlidar_ros2_driver` is imported with `vcs` into `src/vendor/ydlidar_ros2_driver` and is ignored by Git.
- `YDLidar-SDK` is imported with `vcs` into `vendor/ydlidar-sdk`, patched, and installed into `/opt/slamrobot/vendor/ydlidar-sdk`.
- Third-party commits in `.repos` files must be full commit SHA values, not branch names.
- Runtime maps, bags, logs, reports and calibration files stay outside Git.

Current locked sources:

| Component | Source | Commit | Local target | Patch |
| --- | --- | --- | --- | --- |
| YDLidar SDK | https://github.com/YDLIDAR/YDLidar-SDK.git | `01cdda4f2b36dff2a706d0535c64228d863c7411` | `vendor/ydlidar-sdk` | `deps/patches/ydlidar-sdk-modern-cmake.patch` |
| YDLidar ROS2 driver | https://github.com/YDLIDAR/ydlidar_ros2_driver.git | `4ef70d3f32a85704ade0be54b214f3763b1ab3e8` | `src/vendor/ydlidar_ros2_driver` | `deps/patches/ydlidar-ros2-driver-local-safety.patch` |
