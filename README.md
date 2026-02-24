# orbslam3_ros2

![ROS 2-Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Build-colcon](https://img.shields.io/badge/Build-colcon-2A5CAA)
![Runtime-RGB--D%20(non--inertial)](https://img.shields.io/badge/Runtime-RGB--D%20(non--inertial)-1F6FEB)

## Contents

- [Preamble](#preamble).
- [Prerequisites](#prerequisites).
- [Build](#build).
- [Run](#run).
- [Third-party sources](#third-party-sources).

## Preamble

ROS 2 Humble wrapper for ORB-SLAM3 with a vendored superbuild:

- `third_party/Pangolin` is built and installed into the package install prefix.
- `third_party/ORB_SLAM3` is built against that Pangolin install.
- The ROS 2 node is linked against the staged ORB-SLAM3/Pangolin artifacts.

This package expects the `third_party` sources/submodules to be present.

## Prerequisites

Platform assumed in this README:

- Ubuntu 22.04.
- ROS 2 Humble.

### Quick summary

| Item              | Value                    |
| ----------------- | ------------------------ |
| ROS distro        | Humble                   |
| Language standard | C++17                    |
| Build tool        | `colcon` + `ament_cmake` |
| SLAM core         | vendored `ORB_SLAM3`     |
| Viewer            | vendored `Pangolin`      |

### System packages

Install common build tools first. ROS package dependencies are resolved via `rosdep` in the next steps.

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  ninja-build \
  pkg-config \
  python3 \
  python3-colcon-common-extensions \
  python3-rosdep \
  libeigen3-dev \
  libopencv-dev \
  libglew-dev \
  libepoxy-dev
```

Notes:

- `ninja-build` is preferred. The ORB-SLAM3 vendor build script falls back to `Unix Makefiles` if Ninja is unavailable.
- `libepoxy-dev` is important for Pangolin on Ubuntu 22.04.

### ROS environment

```bash
source /opt/ros/humble/setup.bash
```

### rosdep

From the workspace root (`colcon_ws`):

```bash
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Ensure vendored sources exist

This package expects:

- `third_party/Pangolin`.
- `third_party/ORB_SLAM3`.

If you cloned without submodules, initialize/update them before building.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select orbslam3_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If you changed the CMake generator previously and hit generator mismatch errors, remove only this package's build cache (for example `build/orbslam3_ros2`) and rebuild.

> [!Tip] 
> Avoid reusing the same build directory across different generators (`Ninja` vs `Unix Makefiles`).

## Run

Choose one of the following paths:

- **Generic RGB-D launch**: for standard ROS 2 RGB-D camera topics (simulation or hardware).
- **WSL + RealSense launch**: for the WSL-specific RealSense workaround flow.

### Generic RGB-D (custom topics / datasets)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch orbslam3_ros2 tum_rgbd.launch.py
```

Common launch arguments:

- `voc_file`.
- `settings_file` (defaults to `config/RGB-D/TUM1.yaml`).
- `rgb_topic` (default `/camera/rgb/image_raw`).
- `depth_topic` (default `/camera/depth_registered/image_raw`).
- `enable_pangolin`.
- `viewer_backend` (`auto|x11|wayland`, default `auto`).
- `use_sim_time`.

Viewer backend note:

- In Wayland sessions, `viewer_backend:=auto` forces Pangolin to use X11/XWayland (`WAYLAND_DISPLAY=0`) to avoid a known Wayland teardown crash.
- You can force Wayland with `viewer_backend:=wayland` (not recommended; may crash on shutdown in some environments).

Atlas save/load:

- Default `TUM1.yaml` does **not** save an atlas.
- Use `config/RGB-D/TUM1_save.yaml` to save on shutdown (`Ctrl-C`).
- The provided save-ready configs write atlas files under `src/orbslam3_ros2/maps/`.

Example:

```bash
ros2 launch orbslam3_ros2 tum_rgbd.launch.py \
  settings_file:=/path/to/TUM2.yaml \
  rgb_topic:=/camera/rgb/image_color \
  depth_topic:=/camera/depth/image
```

Example (generic RGB-D with atlas save):

```bash
ros2 launch orbslam3_ros2 tum_rgbd.launch.py \
  settings_file:=src/orbslam3_ros2/config/RGB-D/TUM1_save.yaml
```

### WSL + RealSense (WSL-specific entry point)

Read the WSL setup guide first:

- [`docs/WSL_REALSENSE_SETUP.md`](docs/WSL_REALSENSE_SETUP.md).

That guide covers Windows `usbipd`, custom `librealsense`, `realsense-ros` source build, and the WSL-specific ROS 2 runtime workarounds.

This launch is intended for WSL RealSense setups and uses `config/RGB-D/RealSense_D435i_save.yaml` (atlas save/load ready).

To run the same WSL workflow without atlas save, override the settings file:

```bash
ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py \
  settings_file:=<path-to>/config/RGB-D/RealSense_D435i.yaml
```

#### Quickstart (default)

Terminal A (start the camera yourself):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera \
  enable_sync:=true align_depth.enable:=true
```

Terminal B (start ORB wrapper only):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py \
  start_realsense:=false viewer_backend:=auto
```

Notes:

- `start_realsense:=false` avoids interfering with a camera process you already launched.
- On WSL/Wayland, keep `viewer_backend:=auto` (it auto-forces X11/XWayland for Pangolin).

#### Optional: quick output-save verification (one-shot)

Use this when you want a short run that auto-shuts down and validates save outputs (`mtime` + non-empty file checks):

```bash
ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py \
  start_realsense:=false viewer_backend:=auto \
  check_outputs:=true orb_shutdown_after_sec:=30.0
```

#### Debug (opt-in)

ASAN is for memory-bug triage. Keep it out of the default quickstart.

```bash
ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py \
  start_realsense:=false viewer_backend:=auto \
  check_outputs:=true orb_shutdown_after_sec:=30.0 \
  asan_options:="detect_odr_violation=1:new_delete_type_mismatch=0:abort_on_error=1:halt_on_error=1" \
  debug_mode:=true
```

Notes:

- `new_delete_type_mismatch=0` is disabled because it is known ROS 2 + ASAN noise (`rcutils` / `rclcpp`), not a workflow-specific signal.
- `debug_mode:=true` defaults `LSAN_OPTIONS=detect_leaks=0` if `lsan_options` is not explicitly set.
- Logs: `orb.log` -> `/tmp/orbslam3_orb.log`, `realsense.log` -> `/tmp/orbslam3_realsense.log`, first ASAN extract -> `/tmp/orbslam3_first_asan.txt`.

#### Troubleshooting (topics)

```bash
ros2 topic list | grep /camera
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

If topics are delayed, the launch gate waits for them before starting ORB-SLAM3.

## Third-party sources

This repository follows a vendor-based integration strategy and keeps explicit provenance for both upstream SLAM dependencies and ROS interface semantics.

### Upstream dependencies

- `Pangolin`: [`stevenlovegrove/Pangolin`](https://github.com/stevenlovegrove/Pangolin).
- `ORB_SLAM3`: [`UZ-SLAMLab/ORB_SLAM3`](https://github.com/UZ-SLAMLab/ORB_SLAM3).

### Reference ROS1 implementation

- [`thien94/orb_slam3_ros`](https://github.com/thien94/orb_slam3_ros).

This ROS 2 package follows the thien94 ROS1 implementation as the practical reference for launch defaults, parameter naming style, and topic/frame runtime semantics, while adapting the implementation to ROS 2 APIs (parameters, QoS, message_filters, tf2).
