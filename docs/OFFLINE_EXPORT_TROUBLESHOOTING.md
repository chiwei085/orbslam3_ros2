# Offline Export Troubleshooting

This document covers repo-specific pitfalls for `orbslam3_export_osa` atlas export.

## 1. Atlas path loading (`LoadAtlasFromFile()` path handling)

Current exporter behavior (this repo):

- `--atlas_file` accepts relative or absolute paths.
- The CLI normalizes the atlas path, temporarily switches to the atlas parent directory, and calls `LoadAtlasFromFile()` with the atlas basename internally.

Preferred usage is a direct atlas path, for example:

```bash
--atlas_file src/orbslam3_ros2/maps/d455_map.osa
```

If you are using an older binary (before the path-handling fix) and atlas loading fails with a path-related error, the historical workaround is:

1. `cd src/orbslam3_ros2/maps`
2. Pass atlas basename (with or without `.osa`) instead of a longer path

Example:

```bash
cd src/orbslam3_ros2/maps

../../install/orbslam3_ros2/lib/orbslam3_ros2/orbslam3_export_osa \
  --voc_file ../../install/orbslam3_ros2/share/orbslam3_ros2/Vocabulary/ORBvoc.txt \
  --settings_file ../config/RGB-D/RealSense_D435i_save.yaml \
  --atlas_file d455_map \
  --out_ply ../examples/open3d_viewer_server/out/map.ply \
  --out_traj_xyz ../examples/open3d_viewer_server/out/traj.xyz
```

This workaround is kept here for compatibility with older builds. The current exporter no longer requires manual `cd` + basename in normal use.

## 2. Multi-map atlas export behavior

A loaded `.osa` atlas may contain multiple maps. Exporting only the "current" map can silently produce incomplete output.

The exporter in this repo aggregates data from `GetAllMaps()` (points/keyframes across all maps in the loaded atlas), which avoids the common "why is my export partial?" trap.

## 3. Other notes

- `--atlas_file` accepts values with or without the `.osa` suffix.
- `ORBvoc.txt` is installed under `install/orbslam3_ros2/share/orbslam3_ros2/Vocabulary/ORBvoc.txt` (you do not need a local `third_party/ORB_SLAM3` checkout for export).
- Use `--no_traj` if you only need the point cloud output.
