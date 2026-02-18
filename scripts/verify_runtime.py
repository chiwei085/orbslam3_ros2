#!/usr/bin/env python3
import argparse
import re
import subprocess
from pathlib import Path


def sh(cmd: list[str]) -> str:
    print("+", " ".join(cmd))
    return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True, type=Path)
    args = ap.parse_args()

    bin_path = args.bin.resolve()
    if not bin_path.exists():
        raise SystemExit(f"binary not found: {bin_path}")

    out = sh(["ldd", str(bin_path)])
    print(out)

    def grab(lib: str) -> str | None:
        match = re.search(rf"{re.escape(lib)}\s*=>\s*(\S+)", out)
        return match.group(1) if match else None

    g2o = grab("libg2o.so")
    stdcpp = grab("libstdc++.so.6")
    libgcc = grab("libgcc_s.so.1")
    jpeg = grab("libjpeg.so.8")

    print("\n[summary]")
    print(f"  libg2o.so      : {g2o}")
    print(f"  libstdc++.so.6 : {stdcpp}")
    print(f"  libgcc_s.so.1  : {libgcc}")
    print(f"  libjpeg.so.8   : {jpeg}")

    if not g2o or ".conan2" not in g2o:
        raise SystemExit("libg2o.so is not resolved from Conan cache. Check RPATH/RUNPATH.")

    print("[ok] runtime g2o resolved from Conan cache.")


if __name__ == "__main__":
    main()
