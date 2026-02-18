# Interface Reference

This document is the full interface contract for `orbslam3_ros2`.

## Topics

### Core IO

- `/orbslam3/observations` (`orbslam3_ros2/msg/Observations`)
  - Producer: `rgbd_frontend_node` (or bag playback/custom frontend)
  - Consumer: `orbslam3_core_node`
  - QoS: `SensorDataQoS` with `keep_last(qos_keep_last)`
  - Rate: event-driven / bag-paced
  - Semantics: canonical observation stream; `header.stamp` is tracking timestamp source
- `/orbslam3/camera_pose` (`geometry_msgs/msg/PoseStamped`)
  - Producer: `orbslam3_core_node`
  - QoS: `KeepLast(10), reliable, volatile`
  - Rate: event-driven on successful tracking results
  - Semantics: `T_map_camera`
- `/orbslam3/odom` (`nav_msgs/msg/Odometry`)
  - Producer: `orbslam3_core_node` when `publish_odom=true`
  - QoS: `KeepLast(10), reliable, volatile`
  - Rate: event-driven
  - Semantics: same pose source as `/orbslam3/camera_pose`
- `/tf` (`tf2_msgs/msg/TFMessage`)
  - Producer: `orbslam3_core_node` when `publish_tf=true`
  - Semantics: includes dynamic `map -> camera_frame`
- `/orbslam3/map/points_snapshot` (`sensor_msgs/msg/PointCloud2`)
  - Producer: `orbslam3_core_node`
  - QoS: `QoS(1), transient_local, reliable`
  - Rate: action-triggered snapshot

### Debug / Utility Topics

- `/orbslam3/tracking/points` (`sensor_msgs/msg/PointCloud2`)
  - Producer: `tracking_cloud_node`
  - QoS: `SensorDataQoS`
  - Source: polled from `/orbslam3/get_tracked_points`
- `/orbslam3/path` (`nav_msgs/msg/Path`)
  - Producer: `path_node`
  - QoS: `SystemDefaultsQoS`
  - Source: accumulated from `/orbslam3/camera_pose`

## Services

- `/orbslam3/get_status` (`orbslam3_ros2/srv/GetStatus`)
  - Purpose: runtime and gate counters
  - Response fields include:
    - lifecycle/worker state: `running`, `worker_alive`, `shutdown_requested`, `tracking_state_name`
    - queue/counter stats: `pending_size`, `pending_drop_count`, `rx_obs_count`, `processed_obs_count`, `pose_published_count`, `ignored_obs_count`
    - timing/resource stats: `last_observation_stamp`, `track_time_us_last`, `memory_rss_kb`
- `/orbslam3/get_tracked_points` (`orbslam3_ros2/srv/GetTrackedPoints`)
  - Request: `max_points`
  - Response: `success`, `message`, `cloud`
- `/orbslam3/reset` (`std_srvs/srv/Empty`)
- `/orbslam3/shutdown` (`std_srvs/srv/Empty`)
- `/orbslam3/save_trajectory` (`orbslam3_ros2/srv/SaveTrajectory`)
  - Request: `format`, `path`
- `/orbslam3/save_map` (`orbslam3_ros2/srv/SaveMap`)
  - Request: `path`
- `/orbslam3/load_map` (`orbslam3_ros2/srv/LoadMap`)
  - Request: `path`
- `/orbslam3/set_localization_mode` (`orbslam3_ros2/srv/SetLocalizationMode`)
  - Request: `localization_mode`
- `/orbslam3/path/reset` (`std_srvs/srv/Empty`)

## Actions

- `/orbslam3/map/export_pointcloud` (`orbslam3_ros2/action/ExportPointCloud`)
  - Goal:
    - `scope` (`tracking` supported; `map` currently not supported)
    - `max_points`
    - `output_path`
    - `publish_snapshot`
    - `include_cloud_in_result`
  - Result:
    - `success`, `message`, `output_path`, `cloud`
  - Feedback:
    - `progress`

## Message Contract Notes

- `Observations.mode`
  - `MODE_RGBD=1`, `MODE_STEREO=2`, `MODE_MONO_IMU=3`
- `Observations.header.stamp`
  - used as tracking timestamp source
- `Observations.header.frame_id` / `camera_frame`
  - camera frame contract; core may override via configured `camera_frame` param

## Quick Checks

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic list | rg "orbslam3|^/tf$"
ros2 service list | rg "orbslam3"
ros2 action list | rg "orbslam3"

ros2 interface show orbslam3_ros2/msg/Observations
ros2 interface show orbslam3_ros2/srv/GetStatus
ros2 interface show orbslam3_ros2/action/ExportPointCloud
```
