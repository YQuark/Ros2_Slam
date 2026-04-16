# Learnings

Corrections, insights, and knowledge gaps captured during development.

**Categories**: correction | insight | knowledge_gap | best_practice

---

## [LRN-20260413-002] best_practice

**Logged**: 2026-04-13T15:10:00+08:00
**Priority**: high
**Status**: pending
**Area**: robotics

### Summary
For ROS2/Nav2 no-motion faults, first prove the launch/BT/plugin contract from logs and upstream Foxy source before asking for repeated robot trials.

### Details
The robot log showed Nav2 lifecycle active but every `NavigateToPose` goal aborted inside `bt_navigator` before any `/cmd_vel` reached the bridge. The deterministic cause was that `nav2_bringup` Foxy `navigation_launch.py` rewrites `default_bt_xml_filename`; a wrapper that does not pass this launch argument can run the default BT XML while a custom params file lists only a small plugin set.

### Suggested Action
When debugging Nav2 motion, audit `navigation_launch.py` launch arguments, `default_bt_xml_filename`, BT XML node tags, and `plugin_lib_names` as one contract before changing robot motion parameters.

### Metadata
- Source: user_feedback
- Related Files: src/robot_bringup/launch/nav2.launch.py, src/robot_bringup/config/nav2_mapping_params.yaml
- Tags: ros2,nav2,foxy,behavior-tree,launch

---

## [LRN-20260413-001] correction

**Logged**: 2026-04-13T14:47:30+08:00
**Priority**: medium
**Status**: pending
**Area**: robotics

### Summary
ROS 2 Foxy on this robot does not support `ros2 topic echo --once` or `ros2 topic echo -n 1`.

### Details
The remote robot rejected both forms as unrecognized arguments. For one-shot inspection on this environment, use a timeout-based command or manually stop `ros2 topic echo`.

### Suggested Action
Prefer `timeout 2 ros2 topic echo /topic` when asking the user to inspect a sample message on the robot.

### Metadata
- Source: user_feedback
- Related Files: AGENTS.md
- Tags: ros2,foxy,cli

---
