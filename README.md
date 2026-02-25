# orbslam3_ros2

![ROS 2-Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Build-colcon](https://img.shields.io/badge/Build-colcon-2A5CAA)
![Runtime-RGB--D%20(non--inertial)](https://img.shields.io/badge/Runtime-RGB--D%20(non--inertial)-1F6FEB)

## Preamble

ROS 2 Humble wrapper for ORB-SLAM3 (RGB-D, non-inertial).

This repo uses a vendor split:

- `src/orbslam3_ros2_vendor` builds/install Pangolin + ORB_SLAM3 in a separate package prefix.
- `orbslam3_ros2` links against installed vendor artifacts, not `third_party/**/lib/*.so` in the wrapper source tree.

## Vendor setup

Initialize the sibling vendor package:

```bash
./src/orbslam3_ros2/scripts/bootstrap_vendor.py
```

Full vendor/submodule/bootstrap details:

- [`docs/VENDOR_SETUP.md`](docs/VENDOR_SETUP.md)

## Build

Assume Ubuntu 22.04 + ROS 2 Humble is already installed.

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./src/orbslam3_ros2/scripts/bootstrap_vendor.py
colcon build
source install/setup.bash   # or setup.zsh
```

If you need detailed vendor/system dependency notes, see [`docs/VENDOR_SETUP.md`](docs/VENDOR_SETUP.md).

## Run

### Generic RGB-D quickstart

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch orbslam3_ros2 tum_rgbd.launch.py
```

Common overrides: `settings_file`, `rgb_topic`, `depth_topic`, `enable_pangolin`, `viewer_backend`, `use_sim_time`.

Atlas save/load note:

- Default `TUM1.yaml` does not save atlas.
- Use `config/RGB-D/TUM1_save.yaml` (or another `*_save.yaml`) to write `.osa` on shutdown.

### WSL + RealSense quickstart

WSL setup and runtime details are documented here:

- [`docs/WSL_REALSENSE_SETUP.md`](docs/WSL_REALSENSE_SETUP.md)

Minimal ORB wrapper launch (camera started separately):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py \
  start_realsense:=false viewer_backend:=auto
```

`viewer_backend:=auto` is recommended on WSL/Wayland.

Sanitizer/debug launch options (`debug_mode`, `asan_options`, `lsan_options`) are documented in [`docs/DEBUG_SANITIZERS.md`](docs/DEBUG_SANITIZERS.md).

## Runtime TF contract

- Publishes dynamic TF each frame: `map_frame_id -> cam_frame_id` (default `map -> camera_link`).
- Publishes pose/path in `map_frame_id`.
- Does not require external TF (`odom -> base_link`, `base_link -> camera_link`) to run.

This matches the runtime expectation used in `thien94/orb_slam3_ros` style deployments.

## Offline export

`orbslam3_export_osa` is the offline exporter CLI for `.osa` atlas files (no ROS topics/TF required).

`--atlas_file` supports direct relative/absolute paths (the exporter normalizes the path and loads from the atlas parent directory internally).

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p src/orbslam3_ros2/examples/open3d_viewer_server/out

install/orbslam3_ros2/lib/orbslam3_ros2/orbslam3_export_osa \
  --voc_file install/orbslam3_ros2/share/orbslam3_ros2/Vocabulary/ORBvoc.txt \
  --settings_file src/orbslam3_ros2/config/RGB-D/RealSense_D435i_save.yaml \
  --atlas_file src/orbslam3_ros2/maps/d455_map.osa \
  --out_ply src/orbslam3_ros2/examples/open3d_viewer_server/out/map.ply \
  --out_traj_xyz src/orbslam3_ros2/examples/open3d_viewer_server/out/traj.xyz
```

For exporter behavior notes (multi-map atlas aggregation, older-binary compatibility notes), see [`docs/OFFLINE_EXPORT_TROUBLESHOOTING.md`](docs/OFFLINE_EXPORT_TROUBLESHOOTING.md).

## Links

- `Pangolin`: [`stevenlovegrove/Pangolin`](https://github.com/stevenlovegrove/Pangolin)
- `ORB_SLAM3`: [`UZ-SLAMLab/ORB_SLAM3`](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- ROS1 reference semantics: [`thien94/orb_slam3_ros`](https://github.com/thien94/orb_slam3_ros) (launch defaults / parameter naming / topic-frame behavior)
