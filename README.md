# orbslam3_ros2

ROS 2 wrapper for ORB-SLAM3 with reproducible local gates and bag-based validation.

Search keywords: ros2 humble orb-slam3 slam conan

## What You Get

- Lifecycle SLAM core (`orbslam3_core_node`) and RGB-D frontend (`rgbd_frontend_node`).
- Deterministic local release gates (`smoke` / `full`) with machine-checkable artifacts.
- Tools for bag generation and native runtime verification.

## Compatibility

- ROS 2 Humble
- Ubuntu 22.04
- x86_64
- `rmw_fastrtps_cpp` (tested)

## Quickstart

Run from workspace root (`$COLCON_WS`).

```bash
export COLCON_WS=$HOME/my_ros2_ws
cd "$COLCON_WS"

conan install src/orbslam3_ros2 \
  -pr:h src/orbslam3_ros2/conan/profiles/myprofile \
  -pr:b src/orbslam3_ros2/conan/profiles/myprofile \
  -of build/conan \
  -b missing

source /opt/ros/humble/setup.bash
colcon build --packages-select orbslam3_ros2 \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=$PWD/build/conan/conan_toolchain.cmake \
    -DCMAKE_PREFIX_PATH=$PWD/build/conan \
    -DCMAKE_BUILD_TYPE=Release
```

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash

python3 src/orbslam3_ros2/scripts/run_e2e.py \
  --ws "$COLCON_WS" \
  --dataset fr1_xyz \
  --smoke \
  --artifact-dir /tmp/orbslam3_e2e_artifacts_smoke
```

Pass contract:

- process exit code is `0`
- `/tmp/orbslam3_e2e_artifacts_smoke/manifest.json` contains `"result": "pass"`

## Interfaces

External contract only.
Full reference: `docs/interfaces.md`.

### Topics

- `/orbslam3/observations` (`orbslam3_ros2/msg/Observations`): frontend->core input, `SensorDataQoS`, bag-paced/event-driven.
- `/orbslam3/camera_pose` (`geometry_msgs/msg/PoseStamped`): `T_map_camera`, reliable/volatile.
- `/tf` (`tf2_msgs/msg/TFMessage`): dynamic TF, includes `map -> camera_link` when `publish_tf=true`.
- `/orbslam3/map/points_snapshot` (`sensor_msgs/msg/PointCloud2`): action-triggered snapshot, transient local + reliable.

### Services

- `/orbslam3/get_status` (`orbslam3_ros2/srv/GetStatus`): runtime counters used by gates.
- `/orbslam3/reset` (`std_srvs/srv/Empty`): reset SLAM runtime.
- `/orbslam3/shutdown` (`std_srvs/srv/Empty`): graceful process shutdown.

### Actions

- `/orbslam3/map/export_pointcloud` (`orbslam3_ros2/action/ExportPointCloud`): point-cloud snapshot export.

Interface quick check:

- `ros2 topic list | rg "orbslam3|^/tf$" && ros2 service list | rg "orbslam3" && ros2 action list | rg "orbslam3"`

## Frames / TF Contract

- Core publishes: `map -> camera_link` (dynamic, stamped from observation header stamp).
- External must provide: `base_link -> camera_link` (usually static).
- Consumers derive: `map -> base_link` via TF tree.

Checks:

- `ros2 run tf2_tools view_frames`
- `ros2 run tf2_ros tf2_echo map camera_link`

## Configuration

README keeps only top-5 high-impact params. Full detail: `docs/config.md`.

- `autostart_lifecycle` (`bool`, default `true`): auto configure/activate.
- `observation_topic` (`string`, default `/orbslam3/observations`): input stream.
- `camera_pose_topic` (`string`, default `/orbslam3/camera_pose`): pose output stream.
- `pending_queue_size` (`int`, default `10`): queue backpressure bound.
- `publish_tf` (`bool`, default `true`): enable/disable TF publishing.

Runtime config files:

- `config/rgbd.yaml`
- `config/rgbd_split.yaml`

## Local Release Gates

Gate command:

- `python3 src/orbslam3_ros2/scripts/run_e2e.py --ws "$COLCON_WS" --dataset fr1_xyz --smoke|--full --artifact-dir /tmp/orbslam3_e2e_artifacts`

Pass/fail spec:

- pass requires: `pose_published_count > 0`, TF lookup `map -> camera_link` success, and status counters (`rx/enqueued/processed > 0`, `ignored_obs_count == 0`)
- fail on invalid config: `play_seconds < min_play_seconds_required` -> `invalid_config=true`

Artifacts:

- `<artifact-dir>/manifest.json`
- `<artifact-dir>/test_summary.json`

## Dataset / Rosbag

- Reference dataset: TUM RGB-D `fr1_xyz`.
- Build observations bag with `orbslam3_make_e2e_bag`, then verify with `ros2 bag info`.

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 run orbslam3_ros2 orbslam3_make_e2e_bag \
  --tum-dir "$COLCON_WS/src/orbslam3_ros2/data/tum/raw/rgbd_dataset_freiburg1_xyz" \
  --output-bag "$COLCON_WS/src/orbslam3_ros2/data/tum/bags/fr1_xyz_observations" \
  --overwrite \
  --topic /orbslam3/observations \
  --frame-id camera_link

ros2 bag info "$COLCON_WS/src/orbslam3_ros2/data/tum/bags/fr1_xyz_observations"
```

`run_e2e.py --dataset fr1_xyz` auto-resolves bag/voc/settings paths.

## Troubleshooting

- `pose not ready within 5s`: check `manifest.json` counters and `test_summary.json`; increase `--max-frames` or playback seconds.
- `invalid_config=true`: `play_seconds` is below `min_play_seconds_required`; rerun with longer playback.
- `libg2o.so` not from Conan cache: inspect `ldd_orbslam3_core_node.txt` in artifact dir; rerun Conan install with `myprofile` and rebuild.

## Development

Full developer workflow: `docs/dev.md`.

Minimal developer path:

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select orbslam3_ros2
colcon test-result --all --verbose
```

## License / Acknowledgements

- Package license: Apache-2.0 (`package.xml`)
- Third-party code under `third_party/` keeps upstream licenses
