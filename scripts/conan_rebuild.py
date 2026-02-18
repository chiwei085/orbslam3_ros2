#!/usr/bin/env python3
import argparse
import os
import re
import subprocess
from pathlib import Path

EIGEN_FLAGS = [
    "-DEIGEN_DONT_VECTORIZE",
    "-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT",
    "-DEIGEN_MAX_ALIGN_BYTES=0",
    "-fno-fast-math",
]


def sh(cmd: list[str], *, cwd: Path | None = None) -> str:
    print("+", " ".join(cmd))
    try:
        return subprocess.check_output(cmd, cwd=cwd, text=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as exc:
        print(exc.output)
        raise


def find_flags_make(build_folder: Path) -> Path | None:
    matches = sorted(build_folder.rglob("flags.make"))
    return matches[0] if matches else None


def extract_build_folders(conan_output: str) -> list[Path]:
    folders: list[Path] = []
    for line in conan_output.splitlines():
        match = re.search(r"Build folder\s+(.+)$", line.strip())
        if match:
            path = Path(match.group(1).strip())
            if path.exists():
                folders.append(path)
    return folders


def validate_flags_make(flags_make: Path) -> list[str]:
    txt = flags_make.read_text(errors="ignore")
    return [flag for flag in EIGEN_FLAGS if flag not in txt]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", required=True, type=Path, help="Conan profile path (host/build)")
    ap.add_argument("--ref", default="g2o_orbslam3/*", help="Conan package ref to rebuild")
    ap.add_argument("--cwd", default=".", type=Path, help="Directory to run conan install in")
    ap.add_argument("--build-missing", action="store_true")
    ap.add_argument(
        "--mode",
        choices=["auto", "install", "create"],
        default="auto",
        help="auto: prefer recipe-level create for g2o_orbslam3 to avoid rebuilding full graph",
    )
    ap.add_argument(
        "--recipe-dir",
        type=Path,
        default=None,
        help="Recipe directory for create mode (default: <cwd>/conan/recipes/g2o_orbslam3)",
    )
    args = ap.parse_args()

    profile = args.profile.resolve()
    cwd = args.cwd.resolve()
    if not profile.exists():
        raise SystemExit(f"profile not found: {profile}")

    mode = args.mode
    if mode == "auto":
        if args.ref.startswith("g2o_orbslam3/") or args.ref.startswith("g2o_orbslam3"):
            mode = "create"
        else:
            mode = "install"

    if mode == "create":
        recipe_dir = (
            args.recipe_dir.resolve()
            if args.recipe_dir is not None
            else (cwd / "conan" / "recipes" / "g2o_orbslam3").resolve()
        )
        if not recipe_dir.exists():
            raise SystemExit(f"g2o recipe dir not found: {recipe_dir}")
        cmd = [
            "conan",
            "create",
            str(recipe_dir),
            f"-pr:h={profile}",
            f"-pr:b={profile}",
        ]
        out = sh(cmd, cwd=recipe_dir)
        print(out)
    else:
        cmd = [
            "conan",
            "install",
            str(cwd),
            f"-pr:h={profile}",
            f"-pr:b={profile}",
            f"--build={args.ref}",
        ]
        if args.build_missing:
            cmd.append("--build=missing")
        out = sh(cmd, cwd=cwd)
        print(out)

    conan_home = Path(os.environ.get("CONAN_HOME", str(Path.home() / ".conan2"))).resolve()
    build_root = conan_home / "p" / "b"
    if not build_root.exists():
        raise SystemExit(f"Conan build root not found: {build_root}")

    print(f"[i] Conan home: {conan_home}")
    build_folders = extract_build_folders(out)
    checked: list[Path] = []

    for build_folder in build_folders:
        flags_make = find_flags_make(build_folder)
        if not flags_make:
            continue
        checked.append(flags_make)
        missing = validate_flags_make(flags_make)
        if not missing:
            print(f"[ok] flags.make contains required Eigen ABI flags: {flags_make}")
            return

    # Fallback: scan all g2o build folders to avoid false negatives from cache ordering.
    candidates = sorted(
        build_root.glob("g2o_*/**/flags.make"), key=lambda p: p.stat().st_mtime, reverse=True
    )
    for flags_make in candidates:
        checked.append(flags_make)
        missing = validate_flags_make(flags_make)
        if not missing:
            print(f"[ok] flags.make contains required Eigen ABI flags: {flags_make}")
            return

    checked_str = "\n".join(str(p) for p in checked[:20]) or "(none)"
    raise SystemExit(
        f"Could not find any g2o flags.make with required Eigen ABI flags.\nChecked:\n{checked_str}"
    )


if __name__ == "__main__":
    main()
