#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a command, tee combined stdout/stderr to a file and console."
    )
    parser.add_argument("--log-file", required=True)
    parser.add_argument("cmd", nargs=argparse.REMAINDER, help="Command to run (prefix with --)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cmd = list(args.cmd)
    if cmd and cmd[0] == "--":
        cmd = cmd[1:]
    if not cmd:
        print("[run_and_tee] missing command", file=sys.stderr, flush=True)
        return 2

    log_path = Path(args.log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    with log_path.open("wb") as log_fp:
        header = f"[run_and_tee] exec: {cmd}\n".encode("utf-8", errors="replace")
        log_fp.write(header)
        log_fp.flush()
        sys.stdout.buffer.write(header)
        sys.stdout.buffer.flush()

        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=None,
            env=os.environ.copy(),
        )

        assert proc.stdout is not None
        for chunk in iter(lambda: proc.stdout.read1(4096), b""):
            log_fp.write(chunk)
            log_fp.flush()
            sys.stdout.buffer.write(chunk)
            sys.stdout.buffer.flush()

        return proc.wait()


if __name__ == "__main__":
    raise SystemExit(main())
