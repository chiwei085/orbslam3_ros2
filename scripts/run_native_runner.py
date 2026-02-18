#!/usr/bin/env python3
import argparse
import os
import shlex
import subprocess
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", required=True, type=Path)
    ap.add_argument("--tum-dir", required=True, type=Path)
    ap.add_argument("--voc", required=True, type=Path)
    ap.add_argument("--settings", required=True, type=Path)
    ap.add_argument("--depth-scale", default="5000")
    ap.add_argument("--max-frames", default="300")
    args = ap.parse_args()

    ws = args.ws.resolve()
    runner = ws / "install/orbslam3_ros2/lib/orbslam3_ros2/orbslam3_native_runner"
    if not runner.exists():
        raise SystemExit(f"runner not found: {runner}")

    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(ws / 'install/setup.bash'))} && "
        f"{shlex.quote(str(runner))} "
        f"--tum-dir {shlex.quote(str(args.tum_dir.resolve()))} "
        f"--voc {shlex.quote(str(args.voc.resolve()))} "
        f"--settings {shlex.quote(str(args.settings.resolve()))} "
        f"--depth-scale {shlex.quote(str(args.depth_scale))} "
        f"--max-frames {shlex.quote(str(args.max_frames))}"
    )

    env = {
        "HOME": os.environ.get("HOME", ""),
        "TERM": os.environ.get("TERM", "xterm-256color"),
        "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
    }

    print("+ env -i ... bash -lc", cmd)
    subprocess.check_call(["bash", "--noprofile", "--norc", "-lc", cmd], env=env)


if __name__ == "__main__":
    main()
