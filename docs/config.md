# Configuration Reference

This document is the full parameter reference for `orbslam3_ros2`.

Source of truth in code:

- Core params: `src/core/core_config.cpp`
- RGB-D frontend params: `src/rgbd/rgbd_config.cpp`
- Split launch presets: `launch/orbslam3_rgbd_split.launch.py`

## Runtime Config Files

- `config/rgbd.yaml`: base runtime configuration used by split launch.
- `config/rgbd_split.yaml`: preset catalog (`high_fps`, `low_cpu`).
- `config/orbslam3_rgbd_split.yaml`: deprecated alias kept for compatibility.

## Core Parameters (`orbslam3_core_node`)

### Lifecycle / Frames / IO

- `autostart_lifecycle` (`bool`, default: `true`)
  - Declared in `core_node_main.cpp`.
  - `true`: auto configure + activate in `main`.
  - `false`: stays unconfigured until external lifecycle transition.
- `map_frame` (`string`, default: `map`)
  - Parent frame for pose/odom/TF outputs.
- `camera_frame` (`string`, default: empty)
  - If non-empty, overrides observation frame id for published outputs.
- `publish_odom` (`bool`, default: `false`)
  - Enables `/orbslam3/odom` publisher.
- `publish_tf` (`bool`, default: `true`)
  - Enables TF broadcaster (`map -> camera_frame`).
- `observation_topic` (`string`, default: `/orbslam3/observations`)
- `camera_pose_topic` (`string`, default: `/orbslam3/camera_pose`)
- `odom_topic` (`string`, default: `/orbslam3/odom`)

### ORB-SLAM3 Assets / Mode

- `voc_file` (`string`, default: empty)
- `voc_uri` (`string`, default: empty)
- `settings_file` (`string`, default: empty)
- `settings_uri` (`string`, default: empty)
- `use_viewer` (`bool`, default: `false`)
- `test_mode_skip_orb_init` (`bool`, default: `false`)
- `test_mode_track_delay_ms` (`int64`, default: `0`, min: `0`)

### Queue / Backpressure / Worker

- `qos_keep_last` (`int64`, default: `10`, min: `1`)
- `pending_queue_size` (`int64`, default: `10`, min: `1`)
- `latest_keep_last` (`int64`, default: `3`, min: `1`)
- `latest_take` (`int64`, default: `1`, min: `1`)
- `worker_time_budget_us` (`int64`, default: `8000`, min: `0`)
- `worker_idle_sleep_us` (`int64`, default: `0`, min: `0`)
- `no_work_wait_us` (`int64`, default: `2000`, min: `0`)
- `max_frame_age_ms` (`int64`, default: `0`, min: `0`)

### Recovery / Diagnostics / Point Cloud

- `lost_auto_reset_enable` (`bool`, default: `false`)
- `lost_auto_reset_frames` (`int64`, default: `30`, min: `1`)
- `pointcloud_max_points` (`int64`, default: `5000`, min: `1`)
- `sync_stats_period_ms` (`int64`, default: `1000`, min: `1`)
- `diagnostics_period_ms` (`int64`, default: `1000`, min: `200`)

### Deprecated Compatibility Parameters

- `worker_time_budget_ms` (deprecated)
  - If set (`>=0`), converted to `worker_time_budget_us = value * 1000`.
- `backlog_drop_keep_last` (deprecated)
  - If set (`>=0`), mapped to `latest_keep_last`.

## RGB-D Frontend Parameters (`rgbd_frontend_node`)

### Topics / QoS

- `rgb_topic` (`string`, default: `/camera/color/image_raw`)
- `depth_topic` (`string`, default: `/camera/aligned_depth_to_color/image_raw`)
- `observation_topic` (`string`, default: `/orbslam3/observations`)
- `qos_keep_last` (`int64`, default: `10`, min: `1`)

### Sync

- `sync_queue_size` (`int64`, default: `30`, min: `1`)
- `sync_max_dt_ms` (`int64`, default: `10`, min: `1`)
- `sync_enable_span_guard` (`bool`, default: `true`)
- `sync_max_span_ms` (`int64`, default: `500`, min: `1`)
- `sync_enable_huge_jump_reset` (`bool`, default: `true`)
- `sync_huge_jump_ms` (`int64`, default: `2000`, min: `1`)
- `sync_reset_on_time_backwards` (`bool`, default: `true`)

### Conversion

- `depth_scale` (`double`, default: `0.001`)
- `depth_max_m` (`double`, default: `0.0`, disabled when `<=0`)
- `depth_encoding_override` (`string`, default: empty)
- `rgb_clone` (`bool`, default: `false`)
- `depth_clone` (`bool`, default: `false`)

### Deprecated Compatibility Parameter

- `observation_rgbd_topic` (deprecated)
  - If non-empty, overrides `observation_topic`.

## Split Launch Presets

Defined in `launch/orbslam3_rgbd_split.launch.py`, applied on top of `config/rgbd.yaml`.

- `preset:=high_fps`
  - frontend: `qos_keep_last=20`, `sync_max_dt_ms=8`, `sync_queue_size=60`
  - core: `pending_queue_size=20`, `latest_keep_last=6`, `latest_take=2`, `worker_time_budget_us=12000`, `pointcloud_max_points=8000`
  - tracking cloud: `publish_hz=20`, `max_points=8000`
  - path: `path_max_length=4000`
- `preset:=low_cpu`
  - frontend: `qos_keep_last=8`, `sync_max_dt_ms=15`, `sync_queue_size=20`
  - core: `pending_queue_size=8`, `latest_keep_last=2`, `latest_take=1`, `worker_time_budget_us=6000`, `pointcloud_max_points=3000`
  - tracking cloud: `publish_hz=8`, `max_points=3000`
  - path: `path_max_length=1200`

## Quick Validation Commands

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 param list /orbslam3_core_node
ros2 param get /orbslam3_core_node pending_queue_size
ros2 param get /rgbd_frontend_node sync_max_dt_ms
```
