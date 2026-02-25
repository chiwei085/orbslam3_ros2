# Vendor Package Setup

`orbslam3_ros2` now expects a separate sibling package at `src/orbslam3_ros2_vendor`.

This keeps Pangolin / ORB_SLAM3 builds isolated from the wrapper package and avoids linking against `third_party/**/lib/*.so` from a source tree.

## Quick Start

From the workspace root (`colcon_ws`):

```bash
./src/orbslam3_ros2/scripts/bootstrap_vendor.py
```

The script will:

1. Exit successfully if `src/orbslam3_ros2_vendor` already exists.
2. Prefer `git submodule update --init --recursive` when a `.gitmodules` entry for `orbslam3_ros2_vendor` is present.
3. Fall back to `git clone --recursive` using `ORBROS_VENDOR_REPO_URL`.

## Fallback Clone Mode (No Submodule)

```bash
export ORBROS_VENDOR_REPO_URL='https://github.com/chiwei085/orbslam3_ros2_vendor.git'
export ORBROS_VENDOR_REPO_REF='main'   # optional
./src/orbslam3_ros2/scripts/bootstrap_vendor.py
```

## System Packages for Vendor Build

`orbslam3_ros2_vendor/package.xml` intentionally does not list OpenGL/X11 runtime/build system packages. Install them with apt before building:

```bash
sudo apt update
sudo apt install -y \
  libeigen3-dev \
  libopencv-dev \
  libepoxy-dev \
  libglew-dev \
  libgl1-mesa-dev \
  libx11-dev \
  libxext-dev \
  libxrandr-dev \
  libxi-dev \
  libxinerama-dev \
  libxcursor-dev
```

## Clean Build / Validation

```bash
rm -rf build install log
colcon build
source install/setup.bash   # or: source install/setup.zsh
ldd install/orbslam3_ros2/lib/orbslam3_ros2/ros_rgbd | grep -i asan || echo "no asan"
ldd install/orbslam3_ros2/lib/orbslam3_ros2/orbslam3_export_osa | grep -i asan || echo "no asan"
```
