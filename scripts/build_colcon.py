#!/usr/bin/env python3
import argparse
import subprocess
from pathlib import Path


def sh(cmd: list[str], *, cwd: Path | None = None) -> None:
    print("+", " ".join(cmd))
    subprocess.check_call(cmd, cwd=cwd)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", required=True, type=Path, help="colcon workspace root")
    ap.add_argument("--pkg", default="orbslam3_ros2")
    ap.add_argument(
        "--build-type",
        default="RelWithDebInfo",
        choices=["Debug", "Release", "RelWithDebInfo"],
    )
    ap.add_argument("--clean", action="store_true")
    args = ap.parse_args()

    ws = args.ws.resolve()
    if not ws.exists():
        raise SystemExit(f"workspace not found: {ws}")

    if args.clean:
        for name in ("build", "install", "log"):
            path = ws / name
            if path.exists():
                sh(["rm", "-rf", str(path)])

    sh(
        [
            "colcon",
            "build",
            "--packages-select",
            args.pkg,
            "--cmake-args",
            f"-DCMAKE_BUILD_TYPE={args.build_type}",
        ],
        cwd=ws,
    )


if __name__ == "__main__":
    main()
