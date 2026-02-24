# orbslam3_ros2

## Contents

- [Preamble](#preamble)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Run](#run)
- [Third-party sources](#third-party-sources)

## Preamble

ROS 2 Humble wrapper for ORB-SLAM3 with a vendored superbuild:

- `third_party/Pangolin` is built and installed into the package install prefix.
- `third_party/ORB_SLAM3` is then built against that Pangolin install.
- The ROS 2 node links against the staged ORB-SLAM3/Pangolin artifacts.

This package expects the `third_party` sources/submodules to be present.

> Status: current runtime support is focused on RGB-D (non-inertial).

## Prerequisites

Platform assumed in this README:

- Ubuntu 22.04
- ROS 2 Humble

### Quick summary

| Item | Value |
| --- | --- |
| ROS distro | Humble |
| Language standard | C++17 |
| Build tool | `colcon` + `ament_cmake` |
| SLAM core | vendored `ORB_SLAM3` |
| Viewer | vendored `Pangolin` |

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

- `ninja-build` is recommended. The ORB-SLAM3 vendor build script will fall back to `Unix Makefiles` if Ninja is unavailable.
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

- `third_party/Pangolin`
- `third_party/ORB_SLAM3`

If you cloned without submodules, initialize/update them before building.

## Build

From workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select orbslam3_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If you changed the CMake generator previously and hit generator mismatch errors, remove only this package's build cache (for example `build/orbslam3_ros2`) and rebuild.

> Tip: avoid reusing the same build directory across different generators (`Ninja` vs `Unix Makefiles`).

## Run

### RGB-D

```bash
source install/setup.bash
ros2 launch orbslam3_ros2 tum_rgbd.launch.py
```

Useful launch args:

- `voc_file`
- `settings_file` (defaults to `config/RGB-D/TUM1.yaml`)
- `rgb_topic` (default `/camera/rgb/image_raw`)
- `depth_topic` (default `/camera/depth_registered/image_raw`)
- `enable_pangolin`
- `use_sim_time`

Example:

```bash
ros2 launch orbslam3_ros2 tum_rgbd.launch.py \
  settings_file:=/path/to/TUM2.yaml \
  rgb_topic:=/camera/rgb/image_color \
  depth_topic:=/camera/depth/image
```

## Third-party sources

This repository follows a vendor-based integration strategy and keeps explicit provenance for both upstream SLAM dependencies and ROS interface semantics.

### Upstream dependencies

- `Pangolin`: [`stevenlovegrove/Pangolin`](https://github.com/stevenlovegrove/Pangolin)
- `ORB_SLAM3`: [`UZ-SLAMLab/ORB_SLAM3`](https://github.com/UZ-SLAMLab/ORB_SLAM3)

### Reference ROS1 implementation

- [`thien94/orb_slam3_ros`](https://github.com/thien94/orb_slam3_ros)

This ROS 2 package follows the thien94 ROS1 implementation as the practical reference for launch defaults, parameter naming style, and topic/frame runtime semantics, while adapting the implementation to ROS 2 APIs (parameters, QoS, message_filters, tf2).
