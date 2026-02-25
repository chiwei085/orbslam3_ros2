#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path


def _strip_quotes(text: str) -> str:
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in {"'", '"'}:
        return text[1:-1]
    return text


def _parse_save_atlas_from_settings(settings_file: Path) -> list[str]:
    if not settings_file.is_file():
        raise FileNotFoundError(f"settings file not found: {settings_file}")

    atlas_targets: list[str] = []
    for raw_line in settings_file.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        if key not in {"SaveAtlasToFile", "System.SaveAtlasToFile"}:
            continue
        value = value.split("#", 1)[0].strip()
        value = _strip_quotes(value)
        if not value:
            continue
        atlas_targets.append(value if value.endswith(".osa") else f"{value}.osa")
    return atlas_targets


def _resolve_expected_paths(args) -> list[Path]:
    expected: list[Path] = []
    if args.output_dir and args.expect_file:
        output_dir = Path(args.output_dir)
        if not output_dir.is_absolute():
            output_dir = Path(args.workspace_root) / output_dir
        for rel in args.expect_file:
            expected.append(output_dir / rel)

    if args.settings_file:
        settings_file = Path(args.settings_file)
        atlas_candidates = _parse_save_atlas_from_settings(settings_file)
        for item in atlas_candidates:
            path = Path(item)
            if not path.is_absolute():
                path = Path(args.workspace_root) / path
            expected.append(path)

    # Preserve order while removing duplicates.
    deduped: list[Path] = []
    seen = set()
    for path in expected:
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(path)
    return deduped


def _path_ready(path: Path, min_mtime: float | None, require_nonempty: bool) -> tuple[bool, str]:
    if not path.exists():
        return False, "missing"
    if require_nonempty and path.is_file() and path.stat().st_size <= 0:
        return False, "empty"
    if min_mtime is not None and path.stat().st_mtime < min_mtime:
        return False, f"stale(mtime={path.stat().st_mtime:.3f} < {min_mtime:.3f})"
    return True, "ok"


def main() -> int:
    parser = argparse.ArgumentParser(description="Check ORB-SLAM3 save outputs after shutdown.")
    parser.add_argument("--workspace-root", default=".")
    parser.add_argument("--settings-file")
    parser.add_argument("--output-dir")
    parser.add_argument("--expect-file", action="append", default=[])
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--min-mtime", type=float)
    parser.add_argument("--require-nonempty", action="store_true")
    args = parser.parse_args()

    expected_paths = _resolve_expected_paths(args)
    if not expected_paths:
        print(
            "[check_outputs] no expected outputs resolved (provide settings_file or output_dir+expect-file)",
            file=sys.stderr,
            flush=True,
        )
        return 2

    deadline = time.monotonic() + args.timeout_sec
    print("[check_outputs] expecting:", file=sys.stderr, flush=True)
    for path in expected_paths:
        print(f"  - {path}", file=sys.stderr, flush=True)
    if args.min_mtime is not None:
        print(f"[check_outputs] min_mtime={args.min_mtime:.3f}", file=sys.stderr, flush=True)
    if args.require_nonempty:
        print("[check_outputs] require_nonempty=1", file=sys.stderr, flush=True)

    while time.monotonic() < deadline:
        statuses = {
            path: _path_ready(path, args.min_mtime, args.require_nonempty)
            for path in expected_paths
        }
        not_ready = [path for path, (ok, _reason) in statuses.items() if not ok]
        if not not_ready:
            print("[check_outputs] all expected outputs found", file=sys.stderr, flush=True)
            return 0
        time.sleep(0.5)

    statuses = {
        path: _path_ready(path, args.min_mtime, args.require_nonempty) for path in expected_paths
    }
    print("[check_outputs] outputs not ready after timeout:", file=sys.stderr, flush=True)
    for path, (ok, reason) in statuses.items():
        if ok:
            continue
        print(f"  - {path}: {reason}", file=sys.stderr, flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
