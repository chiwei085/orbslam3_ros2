#!/usr/bin/env python3

import argparse
import os
import sys
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open3D WebVisualizer server for exported ORB-SLAM3 map point clouds"
    )
    parser.add_argument(
        "--pcd",
        required=True,
        help="Path to point cloud file (.ply or .pcd), e.g. /tmp/map.ply",
    )
    parser.add_argument(
        "--traj_xyz",
        default=None,
        help="Optional trajectory file with one 'x y z' point per line",
    )
    parser.add_argument(
        "--bind",
        default="0.0.0.0",
        help="Bind IP for Open3D WebRTC server (WEBRTC_IP)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8888,
        help="Port for Open3D WebRTC server (WEBRTC_PORT)",
    )
    return parser.parse_args()


def _load_traj_xyz(path: Path) -> np.ndarray:
    try:
        data = np.loadtxt(path, dtype=np.float64)
    except OSError as exc:
        raise FileNotFoundError(f"trajectory file not found: {path}") from exc
    except ValueError as exc:
        raise ValueError(f"failed to parse trajectory xyz file: {path}: {exc}") from exc

    if data.size == 0:
        raise ValueError(f"trajectory file is empty: {path}")
    if data.ndim == 1:
        if data.shape[0] != 3:
            raise ValueError(f"trajectory file must have 3 columns (x y z): {path}")
        data = data.reshape(1, 3)
    if data.shape[1] != 3:
        raise ValueError(f"trajectory file must have exactly 3 columns (x y z): {path}")
    return data


def _make_traj_geometry(o3d, traj: np.ndarray):
    points = o3d.utility.Vector3dVector(traj)
    if len(traj) < 2:
        traj_pcd = o3d.geometry.PointCloud()
        traj_pcd.points = points
        traj_pcd.paint_uniform_color([1.0, 0.1, 0.1])
        return traj_pcd

    lines = np.column_stack(
        (
            np.arange(0, len(traj) - 1, dtype=np.int32),
            np.arange(1, len(traj), dtype=np.int32),
        )
    )
    line_set = o3d.geometry.LineSet()
    line_set.points = points
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.paint_uniform_color([1.0, 0.1, 0.1])
    return line_set


def _make_point_material(o3d):
    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = "defaultUnlit"
    material.point_size = 2.0
    return material


def _make_line_material(o3d):
    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = "unlitLine"
    try:
        material.line_width = 2.0
    except Exception:
        pass
    return material


def _run_webrtc_o3dvisualizer(o3d, pcd, traj_geometry, args) -> int:
    if not hasattr(o3d.visualization, "O3DVisualizer"):
        raise RuntimeError(
            "Open3D O3DVisualizer is unavailable in this build. "
            "Upgrade Open3D in this example env (e.g. `uv sync` after bumping open3d)."
        )

    gui = o3d.visualization.gui
    app = gui.Application.instance
    app.initialize()

    vis = o3d.visualization.O3DVisualizer("ORB-SLAM3 Open3D Web Viewer", 1280, 720)
    vis.show_settings = True
    vis.add_geometry("map", pcd, _make_point_material(o3d))
    if traj_geometry is not None:
        vis.add_geometry("traj", traj_geometry, _make_line_material(o3d))
    vis.reset_camera_to_default()

    app.add_window(vis)
    app.run()
    return 0


def _run(args: argparse.Namespace) -> int:
    pcd_path = Path(args.pcd)
    if not pcd_path.exists():
        raise FileNotFoundError(f"point cloud file not found: {pcd_path}")

    suffix = pcd_path.suffix.lower()
    if suffix not in {".ply", ".pcd"}:
        raise ValueError(f"--pcd must be a .ply or .pcd file: {pcd_path}")

    os.environ["WEBRTC_IP"] = args.bind
    os.environ["WEBRTC_PORT"] = str(args.port)

    import open3d as o3d

    pcd = o3d.io.read_point_cloud(str(pcd_path))
    if pcd.is_empty():
        raise RuntimeError(f"loaded point cloud is empty: {pcd_path}")

    traj_geometry = None
    if args.traj_xyz:
        traj_path = Path(args.traj_xyz)
        traj = _load_traj_xyz(traj_path)
        traj_geometry = _make_traj_geometry(o3d, traj)

    o3d.visualization.webrtc_server.enable_webrtc()
    print(f"Open3D version: {o3d.__version__}")
    print(f"WEBRTC_IP={os.environ['WEBRTC_IP']} WEBRTC_PORT={os.environ['WEBRTC_PORT']}")
    print("This server does not create a local OpenGL window.")
    print(f"Loaded point cloud: {pcd_path}")
    if args.traj_xyz:
        print(f"Loaded trajectory: {args.traj_xyz}")
    print(f"Open in browser: http://<server-ip>:{args.port}")
    print(
        "If this runs on a remote machine, SSH tunnel is recommended: "
        f"ssh -L {args.port}:127.0.0.1:{args.port} user@server"
    )

    return _run_webrtc_o3dvisualizer(o3d, pcd, traj_geometry, args)


def main() -> int:
    args = parse_args()
    try:
        return _run(args)
    except KeyboardInterrupt:
        print("\nStopping viewer server.")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
