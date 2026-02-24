#!/usr/bin/env python3

import argparse
import os
import pathlib
import shlex
import shutil
import subprocess
import tarfile
from collections.abc import Iterable, Sequence

ROOT = pathlib.Path(__file__).resolve().parents[1]
ORB = ROOT / "third_party" / "ORB_SLAM3"


def _run(cmd: Sequence[str], cwd: pathlib.Path) -> None:
    print(f"+ {shlex.join(cmd)}  (cwd: {cwd})", flush=True)
    subprocess.run(list(cmd), cwd=str(cwd), check=True)


def _read_cmake_cache_var(cache: pathlib.Path, key: str) -> str | None:
    if not cache.exists():
        return None
    prefix = f"{key}:"
    for line in cache.read_text(errors="ignore").splitlines():
        if line.startswith(prefix):
            # e.g. CMAKE_GENERATOR:INTERNAL=Ninja
            parts = line.split("=", 1)
            return parts[1].strip() if len(parts) == 2 else None
    return None


def _pick_generator(requested: str) -> str:
    # If user asks Ninja but it's not available, fall back.
    if requested.lower() == "ninja" and shutil.which("ninja") is None:
        print("! ninja not found; falling back to 'Unix Makefiles'", flush=True)
        return "Unix Makefiles"
    return requested


def _cmake_configure(
    src_dir: pathlib.Path,
    build_dir: pathlib.Path,
    generator: str,
    build_type: str,
    extra_cmake_args: Iterable[str],
) -> None:
    build_dir.mkdir(parents=True, exist_ok=True)

    generator = _pick_generator(generator)

    cache = build_dir / "CMakeCache.txt"
    if cache.exists():
        prev_gen = _read_cmake_cache_var(cache, "CMAKE_GENERATOR")
        if prev_gen and prev_gen != generator:
            print(
                f"! generator changed: '{prev_gen}' -> '{generator}', wiping {build_dir}",
                flush=True,
            )
            shutil.rmtree(build_dir)
            build_dir.mkdir(parents=True, exist_ok=True)
        else:
            return

    args = [
        "cmake",
        "-S",
        str(src_dir),
        "-B",
        str(build_dir),
        "-G",
        generator,
        f"-DCMAKE_BUILD_TYPE={build_type}",
        "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
    ]
    args += list(extra_cmake_args)
    _run(args, cwd=build_dir)


def _cmake_build(build_dir: pathlib.Path, jobs: int) -> None:
    # portable parallel builds via `cmake --build --parallel N`
    _run(["cmake", "--build", str(build_dir), "--parallel", str(jobs)], cwd=build_dir)


def _safe_extract_tgz(tgz: pathlib.Path, dst: pathlib.Path) -> None:
    dst = dst.resolve()
    with tarfile.open(tgz, "r:gz") as t:
        members = t.getmembers()
        for m in members:
            target = (dst / m.name).resolve()
            if not str(target).startswith(str(dst) + os.sep):
                raise RuntimeError(f"Refusing to extract outside dst: {m.name}")
        t.extractall(path=dst, members=members)


def _stage_install(prefix: pathlib.Path) -> None:
    lib = ORB / "lib" / "libORB_SLAM3.so"
    if lib.exists():
        (prefix / "lib").mkdir(parents=True, exist_ok=True)
        _run(
            ["cmake", "-E", "copy_if_different", str(lib), str(prefix / "lib" / lib.name)], cwd=ORB
        )

    inc_src = ORB / "include"
    if inc_src.exists():
        inc_dst = prefix / "include" / "ORB_SLAM3"
        inc_dst.parent.mkdir(parents=True, exist_ok=True)
        _run(["cmake", "-E", "copy_directory", str(inc_src), str(inc_dst)], cwd=ORB)


def main() -> int:
    DEFAULT_JOBS = 8

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--jobs",
        type=int,
        default=int(os.environ.get("ORB_BUILD_JOBS", str(DEFAULT_JOBS))),
    )
    ap.add_argument(
        "--prefix",
        type=str,
        default=os.environ.get("ORB_INSTALL_PREFIX", ""),
    )
    ap.add_argument(
        "--generator",
        type=str,
        default=os.environ.get("ORB_CMAKE_GENERATOR", "Ninja"),
    )
    ap.add_argument(
        "--build-type",
        type=str,
        default=os.environ.get("ORB_BUILD_TYPE", "Release"),
    )
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--skip-vocab", action="store_true")
    args = ap.parse_args()

    jobs: int = args.jobs
    prefix = pathlib.Path(args.prefix).resolve() if args.prefix else None
    generator: str = args.generator
    build_type: str = args.build_type
    std_args = [
        "-DCMAKE_CXX_STANDARD=17",
        "-DCMAKE_CXX_STANDARD_REQUIRED=ON",
        "-DCMAKE_CXX_EXTENSIONS=OFF",
    ]

    if args.clean:
        for d in [
            ORB / "Thirdparty" / "DBoW2" / "build",
            ORB / "Thirdparty" / "g2o" / "build",
            ORB / "Thirdparty" / "Sophus" / "build",
            ORB / "build",
        ]:
            if d.exists():
                print(f"rm -rf {d}", flush=True)
                shutil.rmtree(d)

    extra_orb = []
    if prefix:
        extra_orb += [
            f"-DCMAKE_PREFIX_PATH={prefix}",
            f"-DPangolin_DIR={prefix}/lib/cmake/Pangolin",
        ]
    extra_orb += [
        "-DCMAKE_CXX_FLAGS=-std=c++17",
    ]

    thirdparty = [
        ("DBoW2", ORB / "Thirdparty" / "DBoW2"),
        ("g2o", ORB / "Thirdparty" / "g2o"),
        ("Sophus", ORB / "Thirdparty" / "Sophus"),
    ]
    for name, src in thirdparty:
        b = src / "build"
        print(f"== Build Thirdparty/{name} ==", flush=True)
        _cmake_configure(src, b, generator, build_type, extra_cmake_args=std_args)
        _cmake_build(b, jobs)

    if not args.skip_vocab:
        voc_dir = ORB / "Vocabulary"
        txt = voc_dir / "ORBvoc.txt"
        tgz = voc_dir / "ORBvoc.txt.tar.gz"
        if (not txt.exists()) and tgz.exists():
            print("== Extract Vocabulary/ORBvoc.txt.tar.gz ==", flush=True)
            _safe_extract_tgz(tgz, voc_dir)

    print("== Build ORB_SLAM3 ==", flush=True)
    orb_build = ORB / "build"
    _cmake_configure(ORB, orb_build, generator, build_type, extra_cmake_args=std_args + extra_orb)
    _cmake_build(orb_build, jobs)

    if prefix:
        print(f"== Stage install into {prefix} ==", flush=True)
        _stage_install(prefix)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
