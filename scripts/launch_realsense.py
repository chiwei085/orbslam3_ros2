#!/usr/bin/env python3
import argparse
import os
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Exec wrapper for launching realsense2_camera_node without shell glue."
    )
    parser.add_argument("--exec-path", required=True, help="Path to realsense2_camera_node")
    parser.add_argument(
        "exec_args",
        nargs=argparse.REMAINDER,
        help="Arguments for the target executable (prefix with --)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    exec_path = str(Path(args.exec_path))
    if not os.path.isfile(exec_path):
        print(f"[launch_realsense] executable not found: {exec_path}", file=sys.stderr)
        return 2

    exec_args = list(args.exec_args)
    if exec_args and exec_args[0] == "--":
        exec_args = exec_args[1:]

    argv = [exec_path] + exec_args
    print(f"[launch_realsense] exec: {argv}", file=sys.stderr, flush=True)
    os.execvpe(exec_path, argv, os.environ.copy())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
