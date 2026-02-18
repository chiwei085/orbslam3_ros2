#!/usr/bin/env python3

import argparse
import json
import os
import shlex
import subprocess
import sys
from datetime import datetime
from pathlib import Path

MIN_PLAY_SECONDS_REQUIRED = 8.0


def required_path(path: Path, name: str) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.exists():
        raise SystemExit(f"{name} not found: {resolved}")
    return resolved


def run_capture(cmd: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False
    )


def build_dataset_paths(ws: Path, dataset: str) -> tuple[Path, Path, Path]:
    ds = dataset.strip()
    bag = ws / "src" / "orbslam3_ros2" / "data" / "tum" / "bags" / f"{ds}_observations"
    voc = ws / "src" / "orbslam3_ros2" / "third_party" / "ORB_SLAM3" / "Vocabulary" / "ORBvoc.txt"
    if ds.startswith("fr1_"):
        settings_name = "TUM1.yaml"
    elif ds.startswith("fr2_"):
        settings_name = "TUM2.yaml"
    elif ds.startswith("fr3_"):
        settings_name = "TUM3.yaml"
    else:
        settings_name = "TUM1.yaml"
    settings = (
        ws
        / "src"
        / "orbslam3_ros2"
        / "third_party"
        / "ORB_SLAM3"
        / "Examples"
        / "RGB-D"
        / settings_name
    )
    return bag, voc, settings


def verify_runtime(ws: Path, artifact_dir: Path) -> dict:
    core_bin = ws / "install" / "orbslam3_ros2" / "lib" / "orbslam3_ros2" / "orbslam3_core_node"
    if not core_bin.exists():
        raise SystemExit(f"core binary not found: {core_bin}")
    ldd = run_capture(["ldd", str(core_bin)])
    (artifact_dir / "ldd_orbslam3_core_node.txt").write_text(ldd.stdout, encoding="utf-8")
    if ldd.returncode != 0:
        raise SystemExit("ldd failed for orbslam3_core_node")
    g2o_line = next((line for line in ldd.stdout.splitlines() if "libg2o.so" in line), "")
    if ".conan2" not in g2o_line:
        raise SystemExit(
            f"runtime check failed: libg2o.so is not resolved from Conan cache.\nline: {g2o_line}"
        )

    conanfile_dir = ws / "src" / "orbslam3_ros2"
    graph = run_capture(["conan", "graph", "info", str(conanfile_dir)])
    (artifact_dir / "conan_graph_info.txt").write_text(graph.stdout, encoding="utf-8")
    if graph.returncode == 0:
        refs = []
        for line in graph.stdout.splitlines():
            if "g2o" in line.lower() or "eigen/" in line.lower():
                refs.append(line.strip())
        (artifact_dir / "conan_graph_refs.txt").write_text("\n".join(refs) + "\n", encoding="utf-8")
    else:
        refs = []

    g2o_path = ""
    for line in ldd.stdout.splitlines():
        if "libg2o.so" in line and "=>" in line:
            g2o_path = line.split("=>", 1)[1].split("(", 1)[0].strip()
            break

    return {
        "libg2o_path": g2o_path,
        "conan_graph_refs": refs,
    }


def git_fingerprint(pkg_dir: Path) -> dict:
    rev = run_capture(["git", "-C", str(pkg_dir), "rev-parse", "HEAD"])
    status = run_capture(["git", "-C", str(pkg_dir), "status", "--porcelain"])
    return {
        "git_head": rev.stdout.strip() if rev.returncode == 0 else "",
        "git_dirty": bool(status.stdout.strip()) if status.returncode == 0 else False,
        "git_status_porcelain": status.stdout,
    }


def write_manifest(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description="Run ORBSLAM3 non-test_mode E2E launch_test gate")
    ap.add_argument("--ws", required=True, type=Path, help="colcon workspace root")
    ap.add_argument("--dataset", default="", help="dataset preset, e.g. fr1_xyz")
    ap.add_argument("--bag", type=Path, help="E2E rosbag2 path")
    ap.add_argument("--voc", type=Path, help="ORBvoc.txt path")
    ap.add_argument("--settings", type=Path, help="ORB settings yaml")
    mode_group = ap.add_mutually_exclusive_group()
    mode_group.add_argument("--smoke", action="store_true", help="run Gate-1 only (single cycle)")
    mode_group.add_argument("--full", action="store_true", help="run full gates (default)")
    ap.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="limit playback duration by frames (approx fps based)",
    )
    ap.add_argument("--fps", type=float, default=30.0, help="fps used with --max-frames")
    ap.add_argument(
        "--test-launch",
        type=Path,
        default=Path("src/orbslam3_ros2/test/launch/test_e2e_lifecycle_bag.launch.py"),
        help="launch_test file path (relative to ws or absolute)",
    )
    ap.add_argument(
        "--artifact-dir",
        type=Path,
        default=Path("/tmp/orbslam3_e2e_artifacts"),
        help="directory to store gate artifacts/log summaries",
    )
    ap.add_argument(
        "--ros-log-dir",
        type=Path,
        default=Path("/tmp/ros_log_orbslam3_e2e"),
        help="ROS log directory",
    )
    args = ap.parse_args()

    ws = required_path(args.ws, "workspace")
    if args.dataset:
        ds_bag, ds_voc, ds_settings = build_dataset_paths(ws, args.dataset)
        bag = required_path(args.bag if args.bag else ds_bag, "bag")
        voc = required_path(args.voc if args.voc else ds_voc, "voc")
        settings = required_path(args.settings if args.settings else ds_settings, "settings")
    else:
        if not args.bag or not args.voc or not args.settings:
            raise SystemExit("either --dataset or all of --bag/--voc/--settings must be provided")
        bag = required_path(args.bag, "bag")
        voc = required_path(args.voc, "voc")
        settings = required_path(args.settings, "settings")

    gate_mode = "smoke" if args.smoke else "full"
    play_seconds = None
    if args.max_frames > 0:
        play_seconds = max(float(args.max_frames) / max(args.fps, 1.0), 0.1)

    test_launch = args.test_launch
    if not test_launch.is_absolute():
        test_launch = ws / test_launch
    test_launch = required_path(test_launch, "test launch")
    ros_log_dir = args.ros_log_dir.expanduser().resolve()
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    artifact_dir = args.artifact_dir.expanduser().resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    summary_file = artifact_dir / "test_summary.json"
    manifest_file = artifact_dir / "manifest.json"

    setup = ws / "install" / "local_setup.bash"
    required_path(setup, "workspace local_setup.bash")
    runtime_info = verify_runtime(ws, artifact_dir)
    git_info = git_fingerprint(ws / "src" / "orbslam3_ros2")

    run_meta = {
        "timestamp_utc": datetime.utcnow().isoformat() + "Z",
        "workspace": str(ws),
        "bag": str(bag),
        "voc": str(voc),
        "settings": str(settings),
        "gate_mode": gate_mode,
        "play_seconds": play_seconds,
        "ros_log_dir": str(ros_log_dir),
        "test_launch": str(test_launch),
    }
    (artifact_dir / "run_meta.json").write_text(
        json.dumps(run_meta, indent=2) + "\n", encoding="utf-8"
    )

    manifest = {
        "timestamp_utc": run_meta["timestamp_utc"],
        "mode": gate_mode,
        "dataset": args.dataset,
        "bag_path": str(bag),
        "voc_file": str(voc),
        "settings_file": str(settings),
        "play_seconds": play_seconds,
        "max_frames": args.max_frames,
        "fps": args.fps,
        "min_play_seconds_required": MIN_PLAY_SECONDS_REQUIRED,
        "invalid_config": False,
        "result": "fail",
        "tf_lookup_success": False,
        "counters": {},
        "runtime": runtime_info,
        "git": git_info,
        "summary_file": str(summary_file),
        "latest_ros_log_dir": "",
    }

    if play_seconds is not None and play_seconds < MIN_PLAY_SECONDS_REQUIRED:
        manifest["invalid_config"] = True
        manifest["result"] = "fail"
        manifest["error"] = (
            f"play_seconds={play_seconds} is shorter than "
            f"min_play_seconds_required={MIN_PLAY_SECONDS_REQUIRED}"
        )
        write_manifest(manifest_file, manifest)
        raise SystemExit(2)

    play_seconds_export = (
        f"export ORBSLAM3_E2E_PLAY_SECONDS={shlex.quote(str(play_seconds))} && "
        if play_seconds is not None
        else ""
    )
    cmd = "".join(
        [
            "source /opt/ros/humble/setup.bash && ",
            f"source {shlex.quote(str(setup))} && ",
            f"export ROS_LOG_DIR={shlex.quote(str(ros_log_dir))} && ",
            f"export ORBSLAM3_E2E_BAG_PATH={shlex.quote(str(bag))} && ",
            f"export ORBSLAM3_E2E_VOC_FILE={shlex.quote(str(voc))} && ",
            f"export ORBSLAM3_E2E_SETTINGS_FILE={shlex.quote(str(settings))} && ",
            f"export ORBSLAM3_E2E_GATE_MODE={shlex.quote(gate_mode)} && ",
            f"export ORBSLAM3_E2E_SUMMARY_FILE={shlex.quote(str(summary_file))} && ",
            play_seconds_export,
            f"launch_test {shlex.quote(str(test_launch))}",
        ]
    )

    env = {
        "HOME": os.environ.get("HOME", ""),
        "TERM": os.environ.get("TERM", "xterm-256color"),
        "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
    }
    print("+ env -i ... bash -lc", cmd)
    completed = subprocess.run(["bash", "--noprofile", "--norc", "-lc", cmd], env=env, text=True)
    latest_log = max(
        (p for p in ros_log_dir.iterdir() if p.is_dir()),
        key=lambda p: p.stat().st_mtime,
        default=None,
    )
    if latest_log is not None:
        (artifact_dir / "latest_ros_log_dir.txt").write_text(
            str(latest_log) + "\n", encoding="utf-8"
        )
        manifest["latest_ros_log_dir"] = str(latest_log)
    if summary_file.exists():
        summary = json.loads(summary_file.read_text(encoding="utf-8"))
        cycles = summary.get("cycles", [])
        if cycles:
            last = cycles[-1]
            status = last.get("status") or {}
            manifest["counters"] = status
            manifest["tf_lookup_success"] = bool(last.get("tf_lookup_success", False))
        manifest["test_summary"] = summary
    manifest["result"] = "pass" if completed.returncode == 0 else "fail"
    write_manifest(manifest_file, manifest)
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
