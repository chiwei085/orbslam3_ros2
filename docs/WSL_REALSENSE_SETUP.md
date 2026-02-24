# WSL RealSense Setup for `orbslam3_ros2`

WSL2 + RealSense + ROS 2 commonly hits a few issues in practice (mixed SDK versions, `sudo`, ROS 2 transport, unstable depth streaming). This guide captures a reproducible workflow that worked for us.

## Scope

- Windows + WSL2 (Ubuntu 22.04)
- ROS 2 Humble
- `realsense-ros` in the same colcon workspace (source build)
- ORB-SLAM3 RGB-D + Pangolin

> [!IMPORTANT]
> This guide assumes a **custom-built `librealsense` installed to `/usr/local`**.
> Avoid mixing extra apt/ROS `librealsense` runtimes (for example `librealsense2`, `librealsense2-gl`, `ros-humble-librealsense2`) unless you intentionally want to use them.

## 1. Windows side (PowerShell as Administrator)

> [!IMPORTANT]
> Open **PowerShell as Administrator** before using `usbipd`.

Install `usbipd-win`:

```powershell
winget install --id dorssel.usbipd-win -e
```

Attach the camera to WSL:

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

> [!TIP]
> After reboot, USB replug, or device re-enumeration, you usually need to `attach` again.

## 2. WSL prerequisites (no `librealsense` runtime packages)

Install build tools and USB/udev headers:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  ninja-build \
  pkg-config \
  libusb-1.0-0-dev \
  libudev-dev
```

Confirm the device is visible in WSL (do not hardcode a vendor ID filter):

```bash
lsusb
```

## 3. Build `librealsense` inside WSL (`/usr/local`)

```bash
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir -p build && cd build
```

Use these flags (tested in this WSL workflow):

```bash
cmake .. -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=true \
  -DBUILD_GRAPHICAL_EXAMPLES=true \
  -DBUILD_PYTHON_BINDINGS=false \
  -DCHECK_FOR_UPDATES=OFF

ninja -j"$(nproc)"
sudo ninja install
sudo ldconfig
```

### Install udev rules (copy from your cloned `librealsense`)

Do not depend on apt udev rules in this workflow. Copy the rule from your `librealsense` clone:

```bash
cd ~/librealsense   # replace with your librealsense clone path
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo udevadm settle
```

> [!NOTE]
> On WSL, even with correct udev rules, non-`sudo` RealSense access may still fail. That does not automatically mean your setup is wrong.

## 4. Device checks first (`rs-enumerate-devices`, `realsense-viewer`)

Validate the SDK path before debugging ROS:

```bash
rs-enumerate-devices
realsense-viewer
```

Check:

- `rs-enumerate-devices` prints model / serial / firmware
- `realsense-viewer` shows live video

> [!WARNING]
> Close `realsense-viewer` before starting `realsense2_camera`, or the ROS node may fail to open the device.

### About `sudo`

A common WSL pattern:

- non-`sudo` `rs-enumerate-devices` fails
- `sudo rs-enumerate-devices` works

This does not necessarily mean ROS will fail, but it usually indicates you will need the WSL-specific runtime workarounds below (`sudo` + Fast DDS UDP).

## 5. Add `realsense-ros` to the workspace (source build)

Use the official repo and pin to the version we validated:

```bash
cd <COLCON_WS>/src
git clone https://github.com/realsenseai/realsense-ros.git
cd realsense-ros
git checkout 4.56.4
```

Build:

```bash
cd <COLCON_WS>
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select-regex '^realsense2_'
```

> [!IMPORTANT]
> This is a source-build workflow. You do **not** need `apt install ros-humble-realsense2-camera`.

## 6. Make `realsense2_camera` use your custom `librealsense`

### 6.1 Reconfigure `realsense2_camera` to `/usr/local`

```bash
cd <COLCON_WS>
colcon build --symlink-install \
  --packages-select realsense2_camera \
  --cmake-force-configure \
  --cmake-args -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
```

### 6.2 Set runtime library path

```bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
```

### 6.3 Verify the plugin resolves to `/usr/local`

```bash
ldd <COLCON_WS>/install/realsense2_camera/lib/librealsense2_camera.so | grep librealsense2
```

You should see `/usr/local/lib/librealsense2.so...`.

## 7. WSL-specific ROS 2 pitfall: root/user transport (Fast DDS SHM)

If `realsense2_camera` runs with `sudo` and ORB-SLAM3 runs as a normal user, a common failure mode is:

- topics are visible
- subscribers are connected
- but no image payload arrives (Pangolin shows a black frame / `WAITING FOR IMAGES`)

Workaround (set on both sides):

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

## 8. RealSense RGB-D settings that were stable in WSL (tested)

To make depth + aligned depth actually stream under WSL:

- `640x480@15` for color and depth
- `enable_sync:=true`
- `align_depth.enable:=true`
- `enable_gyro:=false`
- `enable_accel:=false`
- `enable_infra1:=false`
- `enable_infra2:=false`

## 9. Working commands (reproducible)

> [!IMPORTANT]
> Close `realsense-viewer` first.

### Terminal A (RealSense node, root)

```bash
sudo /bin/bash -lc '
source /opt/ros/humble/setup.bash &&
source <COLCON_WS>/install/setup.bash &&
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 &&
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH &&
export ROS_LOG_DIR=/tmp/roslog_realsense &&
mkdir -p /tmp/roslog_realsense &&
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera \
  -r __node:=camera \
  -p enable_sync:=true \
  -p align_depth.enable:=true \
  -p enable_gyro:=false \
  -p enable_accel:=false \
  -p enable_infra1:=false \
  -p enable_infra2:=false \
  -p rgb_camera.color_profile:=640,480,15 \
  -p depth_module.depth_profile:=640,480,15
'
```

### Terminal B (ORB-SLAM3 + Pangolin, user)

```bash
source /opt/ros/humble/setup.bash
source <COLCON_WS>/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOG_DIR=/tmp/roslog_orb
mkdir -p /tmp/roslog_orb

ros2 launch orbslam3_ros2 tum_rgbd.launch.py \
  settings_file:=<COLCON_WS>/src/orbslam3_ros2/config/RGB-D/RealSense_D435i_save.yaml \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  enable_pangolin:=true
```

Success signal (ORB log):

- `First KF:0; Map init KF:0`
- `New Map created with ... points`

## 10. One-command launch (same workarounds built-in)

This package provides:

- `launch/realsense_d435i_rgbd_wsl_save.launch.py`

Run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py
```

> [!NOTE]
> This launch prompts for `sudo` because the RealSense node runs as root in this workflow.

## 11. Atlas save (`.osa`)

Use:

- `config/RGB-D/RealSense_D435i_save.yaml`

Atlas output path:

- `src/orbslam3_ros2/maps/d455_map.osa`

ORB-SLAM3 writes the file on shutdown (`Ctrl-C`).

To load an existing atlas, uncomment and set in `RealSense_D435i_save.yaml`:

```yaml
System.LoadAtlasFromFile: "src/orbslam3_ros2/maps/d455_map"
```
