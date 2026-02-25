#!/usr/bin/env python3

import os
import subprocess
import sys
from pathlib import Path


def _run(cmd: list[str], cwd: Path | None = None) -> None:
    subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=True)


def _is_git_repo(path: Path) -> bool:
    try:
        subprocess.run(
            ["git", "-C", str(path), "rev-parse", "--is-inside-work-tree"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return True
    except subprocess.CalledProcessError:
        return False


def _find_submodule_root(candidate_root: Path) -> Path | None:
    modules_file = candidate_root / ".gitmodules"
    if not modules_file.is_file():
        return None
    try:
        text = modules_file.read_text(encoding="utf-8")
    except OSError:
        return None
    if "orbslam3_ros2_vendor" not in text:
        return None
    if not _is_git_repo(candidate_root):
        return None
    return candidate_root


def _vendor_sources_ready(vendor_dir: Path) -> bool:
    return (vendor_dir / "third_party" / "Pangolin" / "CMakeLists.txt").is_file() and (
        vendor_dir / "third_party" / "ORB_SLAM3" / "CMakeLists.txt"
    ).is_file()


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    pkg_dir = script_dir.parent
    src_dir = pkg_dir.parent
    ws_dir = src_dir.parent
    vendor_dir = src_dir / "orbslam3_ros2_vendor"

    if vendor_dir.is_dir():
        if _vendor_sources_ready(vendor_dir):
            print(f"[bootstrap_vendor] vendor package already exists: {vendor_dir}")
            return 0
        print(
            "[bootstrap_vendor] vendor package exists but sources are incomplete; "
            "trying vendor repo submodules"
        )
        if _is_git_repo(vendor_dir):
            try:
                _run(["git", "submodule", "update", "--init", "--recursive"], cwd=vendor_dir)
            except subprocess.CalledProcessError as exc:
                print(
                    "[bootstrap_vendor] failed to initialize vendor repo submodules. "
                    "Check third_party submodule URLs/credentials.",
                    file=sys.stderr,
                )
                return exc.returncode
            if _vendor_sources_ready(vendor_dir):
                print(f"[bootstrap_vendor] vendor package ready: {vendor_dir}")
                return 0
        print(
            "[bootstrap_vendor] vendor package exists but is incomplete. "
            "Populate third_party/Pangolin and third_party/ORB_SLAM3 "
            "(submodules or vendored source).",
            file=sys.stderr,
        )
        return 1

    submodule_root = _find_submodule_root(ws_dir) or _find_submodule_root(pkg_dir)
    if submodule_root:
        print(f"[bootstrap_vendor] initializing submodule from {submodule_root}")
        _run(["git", "submodule", "update", "--init", "--recursive"], cwd=submodule_root)
        if vendor_dir.is_dir():
            print(f"[bootstrap_vendor] vendor package ready: {vendor_dir}")
            return 0
        print(
            f"[bootstrap_vendor] submodule update completed, but {vendor_dir} was not created",
            file=sys.stderr,
        )
        return 1

    repo_url = os.environ.get("ORBROS_VENDOR_REPO_URL", "").strip()
    if not repo_url:
        print(
            "[bootstrap_vendor] No orbslam3_ros2_vendor submodule entry was found.\n"
            "Set ORBROS_VENDOR_REPO_URL and rerun to clone the vendor package, for example:\n\n"
            "  export ORBROS_VENDOR_REPO_URL='https://github.com/chiwei085/orbslam3_ros2_vendor.git'\n"
            "  ./src/orbslam3_ros2/scripts/bootstrap_vendor.py\n\n"
            "Optional:\n"
            "  export ORBROS_VENDOR_REPO_REF='main'   # branch / tag / commit",
            file=sys.stderr,
        )
        return 1

    print(f"[bootstrap_vendor] cloning vendor package from {repo_url}")
    _run(["git", "clone", "--recursive", repo_url, str(vendor_dir)])

    repo_ref = os.environ.get("ORBROS_VENDOR_REPO_REF", "").strip()
    if repo_ref:
        _run(["git", "checkout", repo_ref], cwd=vendor_dir)
        _run(["git", "submodule", "update", "--init", "--recursive"], cwd=vendor_dir)

    if not _vendor_sources_ready(vendor_dir):
        print(
            "[bootstrap_vendor] cloned vendor package, but third_party sources are missing. "
            "Run `git -C src/orbslam3_ros2_vendor submodule update --init --recursive` "
            "or fix the vendor repo submodule setup.",
            file=sys.stderr,
        )
        return 1

    print(f"[bootstrap_vendor] vendor package ready: {vendor_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
