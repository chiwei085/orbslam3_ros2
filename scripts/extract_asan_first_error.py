#!/usr/bin/env python3
import argparse
import re
import time
from pathlib import Path


ASAN_START_RE = re.compile(r"ERROR: AddressSanitizer:")
LSAN_START_RE = re.compile(r"ERROR: LeakSanitizer:")
ASAN_SUMMARY_RE = re.compile(r"SUMMARY: AddressSanitizer:")
LSAN_SUMMARY_RE = re.compile(r"SUMMARY: LeakSanitizer:")
ASAN_ABORT_RE = re.compile(r"ABORTING")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract the first ASAN error block from a log file."
    )
    parser.add_argument("--input-log", required=True)
    parser.add_argument("--output-file", default="/tmp/orbslam3_first_asan.txt")
    return parser.parse_args()


def extract_first_sanitizer_block(lines: list[str]) -> list[str]:
    start = None
    for i, line in enumerate(lines):
        if ASAN_START_RE.search(line) or LSAN_START_RE.search(line):
            start = i
            break
    if start is None:
        return []

    block: list[str] = []
    saw_summary = False
    for line in lines[start:]:
        block.append(line)
        if ASAN_SUMMARY_RE.search(line) or LSAN_SUMMARY_RE.search(line):
            saw_summary = True
        if ASAN_ABORT_RE.search(line):
            break
        if saw_summary and line.strip() == "":
            break
    return block


def main() -> int:
    args = parse_args()
    input_log = Path(args.input_log)
    output_file = Path(args.output_file)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    # Always truncate/overwrite to avoid stale sanitizer reports from previous runs.
    output_file.write_text("", encoding="utf-8")

    if not input_log.exists():
        msg = (
            "no sanitizer error found\n"
            f"timestamp={time.time():.6f}\n"
            f"log={input_log}\n"
            f"note=input log not found\n"
        )
        output_file.write_text(msg, encoding="utf-8")
        print(f"[extract_asan_first_error] input log not found: {input_log}", flush=True)
        return 1

    lines = input_log.read_text(encoding="utf-8", errors="replace").splitlines(keepends=True)
    block = extract_first_sanitizer_block(lines)
    if not block:
        msg = f"no sanitizer error found\ntimestamp={time.time():.6f}\nlog={input_log}\n"
        output_file.write_text(msg, encoding="utf-8")
        print(msg.strip(), flush=True)
        return 0

    output_file.write_text("".join(block), encoding="utf-8")
    print(f"[extract_asan_first_error] wrote {output_file}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
