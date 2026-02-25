# Open3D Viewer Server (uv example)

## What this is

This example provides an Open3D WebVisualizer server so you can inspect `map.ply` (exported by `orbslam3_export_osa`) in a browser, with optional `traj.xyz` overlay.

It is **not** a ROS package and is **not** included in `colcon build`.

This viewer runs without creating a local window; it works in WSL/headless servers.

## Prerequisites

- `uv` installed (`uv --version`)
- Python 3.10+
- A point cloud such as `examples/open3d_viewer_server/out/map.ply` exported from `orbslam3_export_osa`

## Quickstart (local)

From the `src/orbslam3_ros2` repo root, first export to the example output folder:

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
mkdir -p examples/open3d_viewer_server/out
cd maps
../../../install/orbslam3_ros2/lib/orbslam3_ros2/orbslam3_export_osa \
  --voc_file ../../../install/orbslam3_ros2/share/orbslam3_ros2/Vocabulary/ORBvoc.txt \
  --settings_file ../config/RGB-D/RealSense_D435i_save.yaml \
  --atlas_file d455_map \
  --out_ply ../examples/open3d_viewer_server/out/map.ply \
  --out_traj_xyz ../examples/open3d_viewer_server/out/traj.xyz
cd ../examples/open3d_viewer_server
```

Then start the viewer:

```bash
uv sync
uv run open3d-viewer-server --pcd out/map.ply --bind 127.0.0.1 --port 8888
```

Open in browser:

```text
http://127.0.0.1:8888
```

Open in browser: `http://<ip>:<port>`

Note: in this ORB-SLAM3 fork, offline atlas load is most reliable when `orbslam3_export_osa` is run from `src/orbslam3_ros2/maps` and `--atlas_file` is passed as the atlas basename (for example `d455_map`).

## Quickstart (LAN / remote)

To let another machine on the same LAN view the map:

```bash
cd examples/open3d_viewer_server
uv run open3d-viewer-server --pcd out/map.ply --bind 0.0.0.0 --port 8888
```

Then on the client machine, open:

```text
http://<server-ip>:8888
```

Make sure the port is reachable (firewall / network policy).

## SSH tunnel mode (recommended)

This is the safest option and usually requires no firewall changes.

On the server:

```bash
cd examples/open3d_viewer_server
uv run open3d-viewer-server --pcd out/map.ply --bind 127.0.0.1 --port 8888
```

On the client machine:

```bash
ssh -L 8888:127.0.0.1:8888 user@server
```

Then open in the client browser:

```text
http://127.0.0.1:8888
```

## Trajectory overlay

`traj.xyz` format: one point per line, `x y z`.

Example:

```bash
cd examples/open3d_viewer_server
uv run open3d-viewer-server --pcd out/map.ply --traj_xyz out/traj.xyz --bind 127.0.0.1 --port 8888
```

## Troubleshooting

- Port already in use: choose another port, e.g. `--port 8890`.
- Browser cannot connect / no image: use SSH tunnel mode first to eliminate firewall/network issues.
- Empty point cloud or missing file: verify `orbslam3_export_osa` output paths and confirm `examples/open3d_viewer_server/out/map.ply` exists and is non-empty.
