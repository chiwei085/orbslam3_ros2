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
    ap.add_argument(
        "--edge",
        action="store_true",
        help="enable edge-device build profile (default: off)",
    )
    args = ap.parse_args()

    ws = args.ws.resolve()
    if not ws.exists():
        raise SystemExit(f"workspace not found: {ws}")

    if args.clean:
        for name in ("build", "install", "log"):
            path = ws / name
            if path.exists():
                sh(["rm", "-rf", str(path)])

    cmd = ["colcon", "build", "--packages-select", args.pkg]
    cmake_args = [f"-DCMAKE_BUILD_TYPE={args.build_type}"]
    if args.edge:
        cmd.extend(["--parallel-workers", "1"])
        cmd.extend(
            [
                "--executor",
                "sequential",
                "--event-handlers",
                "console_direct+",
            ]
        )
        cmake_args.extend(
            [
                "-DBUILD_TESTING=OFF",
                "-DCMAKE_CXX_FLAGS_RELEASE=-O2 -g0",
                "-DCMAKE_C_FLAGS_RELEASE=-O2 -g0",
            ]
        )
    cmd.extend(["--cmake-args", *cmake_args])

    sh(cmd, cwd=ws)


if __name__ == "__main__":
    main()
