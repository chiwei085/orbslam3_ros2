#!/usr/bin/env python3
import argparse
import os
import platform
import subprocess
from pathlib import Path


def sh(cmd: list[str], *, cwd: Path | None = None, env: dict[str, str] | None = None) -> None:
    print("+", " ".join(cmd))
    subprocess.check_call(cmd, cwd=cwd, env=env)


def build_env(parallel_jobs: int) -> dict[str, str]:
    jobs = str(max(1, parallel_jobs))
    return {
        **dict(os.environ),
        "CMAKE_BUILD_PARALLEL_LEVEL": jobs,
        "MAKEFLAGS": f"-j{jobs}",
        "NINJAFLAGS": f"-j{jobs}",
    }


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
    ap.add_argument(
        "--ninja-conan",
        action="store_true",
        help="use Conan toolchain output and force Ninja generator",
    )
    ap.add_argument(
        "--profile",
        type=Path,
        default=None,
        help="Conan profile path; default auto-detects linux_x86_64/linux_armv8",
    )
    ap.add_argument(
        "--skip-conan",
        action="store_true",
        help="skip conan install and reuse existing build/conan outputs",
    )
    args = ap.parse_args()

    ws = args.ws.resolve()
    if not ws.exists():
        raise SystemExit(f"workspace not found: {ws}")
    pkg_dir = (ws / "src" / args.pkg).resolve()
    if args.ninja_conan and not pkg_dir.exists():
        raise SystemExit(f"package path not found: {pkg_dir}")

    if args.clean:
        for name in ("build", "install", "log"):
            path = ws / name
            if path.exists():
                sh(["rm", "-rf", str(path)])

    effective_build_type = "Release" if args.edge else args.build_type
    cmd = ["colcon", "build", "--packages-select", args.pkg]
    cmake_args = [f"-DCMAKE_BUILD_TYPE={effective_build_type}"]
    if args.ninja_conan:
        if args.profile is None:
            machine = platform.machine().lower()
            profile_name = "linux_armv8" if machine in {"aarch64", "arm64"} else "linux_x86_64"
            profile = pkg_dir / "conan" / "profiles" / profile_name
        else:
            profile = args.profile.resolve()
        if not profile.exists():
            raise SystemExit(f"Conan profile not found: {profile}")

        conan_output_dir = ws / "build" / "conan"
        toolchain = conan_output_dir / "conan_toolchain.cmake"
        if not args.skip_conan:
            sh(
                [
                    "conan",
                    "install",
                    str(pkg_dir),
                    f"-pr:h={profile}",
                    f"-pr:b={profile}",
                    f"-s:h=build_type={effective_build_type}",
                    f"-s:b=build_type={effective_build_type}",
                    "-of",
                    str(conan_output_dir),
                    "-b",
                    "missing",
                ],
                cwd=ws,
            )
        if not toolchain.exists():
            raise SystemExit(f"Conan toolchain missing: {toolchain}")
        cmake_args.extend(
            [
                "-G",
                "Ninja",
                f"-DCMAKE_TOOLCHAIN_FILE={toolchain}",
                f"-DCMAKE_PREFIX_PATH={conan_output_dir}",
            ]
        )

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
                "-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O2 -g0",
                "-DCMAKE_C_FLAGS_RELWITHDEBINFO=-O2 -g0",
            ]
        )
    cmd.extend(["--cmake-args", *cmake_args])
    if args.edge:
        env = build_env(1)
    else:
        cpu_count = os.cpu_count() or 2
        env = build_env(min(4, max(1, cpu_count // 2)))

    sh(cmd, cwd=ws, env=env)


if __name__ == "__main__":
    main()
