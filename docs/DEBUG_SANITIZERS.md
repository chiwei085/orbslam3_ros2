# Debug Sanitizers (ASan / LSan)

This document keeps sanitizer-specific launch details out of the main README.

## Scope

These options are mainly used with the WSL RealSense launch flow:

- `launch/realsense_d435i_rgbd_wsl_save.launch.py`

Relevant launch args:

- `debug_mode`
- `asan_options`
- `lsan_options`
- `check_outputs`
- `orb_shutdown_after_sec`

## Example (opt-in ASan run)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py \
  start_realsense:=false viewer_backend:=auto \
  check_outputs:=true orb_shutdown_after_sec:=30.0 \
  asan_options:="detect_odr_violation=1:new_delete_type_mismatch=0:abort_on_error=1:halt_on_error=1" \
  debug_mode:=true
```

## Notes

- `new_delete_type_mismatch=0` is intentionally disabled here because it is common ROS 2 + ASan noise (`rcutils` / `rclcpp`) and usually not useful for this workflow.
- `debug_mode:=true` defaults `LSAN_OPTIONS=detect_leaks=0` if `lsan_options` is not explicitly set.
- On WSL/Wayland, keep `viewer_backend:=auto` unless you are specifically testing backend behavior.

## Logs

Typical log outputs used by this launch flow:

- `/tmp/orbslam3_orb.log`
- `/tmp/orbslam3_realsense.log`
- `/tmp/orbslam3_first_asan.txt` (first extracted ASan report)

## Quick output-save verification (without ASan)

Use this for short runs that auto-stop and validate save outputs (`mtime` + non-empty file checks):

```bash
ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py \
  start_realsense:=false viewer_backend:=auto \
  check_outputs:=true orb_shutdown_after_sec:=30.0
```
