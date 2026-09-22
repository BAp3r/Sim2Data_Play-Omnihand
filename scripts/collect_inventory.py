#!/usr/bin/env python3
"""Bounded, read-only inventory. Standard library only; no downloads or installs.

Only the requested JSON report is written. Scan roots are explicit; directory
symlinks are not traversed. A root may itself resolve through a symlink/mount.
Time budgets are cooperative: an OS call blocked on a dead NAS can exceed them.
Reports may contain local paths and must stay out of public Git repositories.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any

ASSET_SUFFIXES = {".usd", ".usda", ".usdc", ".usdz", ".urdf", ".xacro", ".xml", ".stl", ".obj", ".glb", ".step"}
VIDEO_SUFFIXES = {".mp4", ".mov", ".mkv", ".avi"}
TOKENS = ("cardbox", "cardboard", "airbot", "omnihand")
SKIP_DIRS = {".git", ".venv", "__pycache__", "$RECYCLE.BIN", ".Trash"}


def run_command(argv: list[str], timeout: float = 8.0) -> dict[str, Any]:
    """No shell expansion, no input prompts; bounded captured output."""
    if not shutil.which(argv[0]):
        return {"available": False, "command": argv[0]}
    try:
        proc = subprocess.run(argv, capture_output=True, text=True, errors="replace", timeout=timeout, check=False)
        raw = (proc.stdout + proc.stderr).strip()
        return {"available": True, "returncode": proc.returncode, "output": raw[:16000], "output_truncated": len(raw) > 16000}
    except subprocess.TimeoutExpired:
        return {"available": True, "error": "timeout"}
    except OSError as exc:
        return {"available": True, "error": str(exc)}


def scan_root(root: Path, *, kind: str = "assets", max_files: int = 100000,
              max_seconds: float = 45.0, max_matches: int = 200) -> dict[str, Any]:
    """Inventory filenames/metadata, not USD composition, physics or licensing."""
    if kind not in {"assets", "videos"}:
        raise ValueError("kind must be assets or videos")
    if max_files < 1 or not math.isfinite(max_seconds) or max_seconds <= 0 or max_matches < 1:
        raise ValueError("scan limits must be positive")
    started = time.monotonic()
    report: dict[str, Any] = {"requested_root": str(root), "kind": kind, "complete": False,
        "files_seen": 0, "directories_seen": 0, "symlinks_skipped": 0, "suffix_counts": {},
        "matches": [], "matches_total": 0, "matches_truncated": False, "errors": []}
    counts: Counter[str] = Counter()
    try:
        root = root.expanduser().resolve(strict=True)
        if not root.is_dir():
            raise NotADirectoryError(str(root))
    except (OSError, RuntimeError) as exc:
        report["errors"].append(str(exc))
        return report
    report["resolved_root"] = str(root)
    pending = [root]
    stop_reason: str | None = None
    while pending and stop_reason is None:
        directory = pending.pop()
        report["directories_seen"] += 1
        if time.monotonic() - started >= max_seconds:
            stop_reason = "time_limit"
            break
        try:
            with os.scandir(directory) as entries:
                for entry in entries:
                    if time.monotonic() - started >= max_seconds:
                        stop_reason = "time_limit"
                        break
                    try:
                        if entry.is_symlink() or (hasattr(os.path, "isjunction") and os.path.isjunction(entry.path)):
                            report["symlinks_skipped"] += 1
                            continue
                        if entry.is_dir(follow_symlinks=False):
                            if entry.name not in SKIP_DIRS:
                                pending.append(Path(entry.path))
                            continue
                        if not entry.is_file(follow_symlinks=False):
                            continue
                        if report["files_seen"] >= max_files:
                            stop_reason = "file_limit"
                            break
                        report["files_seen"] += 1
                        path = Path(entry.path)
                        suffix = path.suffix.lower()
                        counts[suffix or "<none>"] += 1
                        rel = path.relative_to(root).as_posix()
                        normalized = re.sub(r"[\s_\-]", "", rel.lower())
                        candidate = suffix in VIDEO_SUFFIXES if kind == "videos" else (
                            suffix in {".urdf", ".xacro"} or (suffix in ASSET_SUFFIXES and any(token in normalized for token in TOKENS)))
                        if candidate:
                            report["matches_total"] += 1
                            if len(report["matches"]) < max_matches:
                                stat = entry.stat(follow_symlinks=False)
                                report["matches"].append({"path": rel, "bytes": stat.st_size, "mtime_ns": stat.st_mtime_ns})
                    except OSError as exc:
                        if len(report["errors"]) < 20:
                            report["errors"].append(f"{entry.path}: {exc}")
        except OSError as exc:
            if len(report["errors"]) < 20:
                report["errors"].append(f"{directory}: {exc}")
    report["suffix_counts"] = dict(sorted(counts.items()))
    report["matches"].sort(key=lambda item: item["path"])
    report["matches_truncated"] = report["matches_total"] > len(report["matches"])
    report["complete"] = stop_reason is None and not report["errors"]
    report["stop_reason"] = stop_reason
    report["elapsed_seconds"] = round(time.monotonic() - started, 3)
    report["scope_note"] = "Complete refers only to eligible traversed directories; child symlinks and excluded caches were skipped. Candidate names are not verified assets."
    return report


def installed_versions() -> dict[str, str | None]:
    result: dict[str, str | None] = {}
    for name in ("isaacsim", "isaacsim-core", "isaaclab", "torch", "lerobot", "mjlab", "mujoco", "usd-core", "numpy", "uv"):
        try:
            result[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            result[name] = None
    return result


def write_report(path: Path, report: dict[str, Any], *, overwrite: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if not overwrite:
        # Exclusive creation prevents silently replacing an existing diagnostic.
        with path.open("x", encoding="utf-8") as stream:
            stream.write(text)
        return
    fd, temp = tempfile.mkstemp(prefix=".inventory-", suffix=".json", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-root", type=Path, action="append", default=[], help="Explicit mounted/UNC library or model directory; repeatable")
    parser.add_argument("--video-root", type=Path, help="Optional directory of real reference videos; metadata only")
    parser.add_argument("--repo", type=Path, help="Optional existing Git checkout for status (no remote URLs or credentials read)")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--max-files", type=int, default=100000)
    parser.add_argument("--max-seconds", type=float, default=45.0)
    parser.add_argument("--max-matches", type=int, default=200)
    parser.add_argument("--overwrite", action="store_true", help="Explicitly replace an existing report")
    args = parser.parse_args(argv)
    if args.max_files < 1 or not math.isfinite(args.max_seconds) or args.max_seconds <= 0 or args.max_matches < 1:
        parser.error("scan limits must be positive")
    if args.out.exists() and not args.overwrite:
        parser.error("output already exists; choose a new name or pass --overwrite")
    report: dict[str, Any] = {
        "schema_version": 1, "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "platform": {"system": platform.system(), "release": platform.release(), "machine": platform.machine()},
        "python": {"version": platform.python_version(), "executable": sys.executable},
        "packages_in_this_interpreter_only": installed_versions(),
        "tools": {name: shutil.which(name) for name in ("uv", "git", "git-lfs", "ffmpeg", "ffprobe", "blender", "nvidia-smi", "ssh", "codex")},
        "commands": {}, "scans": [],
        "limitations": ["Does not search unrequested disks or inspect private keys.",
            "Does not prove Vulkan/RTX rendering, USD dependencies, robot dynamics, or LeRobot round-trip compatibility.",
            "Missing Python distributions may be installed in a different environment or a standalone simulator.",
            "Scans follow no child symlinks; rerun with an explicit linked root when needed."]}
    commands = {
        "gpu": ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"],
        "uv": ["uv", "--version"], "git_lfs": ["git", "lfs", "version"],
        "ffmpeg": ["ffmpeg", "-version"], "blender": ["blender", "--version"],
    }
    if platform.system() == "Linux":
        commands["network_mounts"] = ["findmnt", "--json", "--types", "nfs,nfs4,cifs", "--output", "TARGET,SOURCE,FSTYPE"]
        commands["disk_free"] = ["df", "-h", "."]
    if args.repo:
        commands["git_status"] = ["git", "-C", str(args.repo), "status", "--short"]
        commands["git_commit"] = ["git", "-C", str(args.repo), "rev-parse", "HEAD"]
        commands["git_remote_names"] = ["git", "-C", str(args.repo), "remote"]
    for label, command in commands.items():
        report["commands"][label] = run_command(command)
    roots = [(root, "assets") for root in args.asset_root]
    if args.video_root:
        roots.append((args.video_root, "videos"))
    for root, kind in roots:
        report["scans"].append(scan_root(root, kind=kind, max_files=args.max_files,
            max_seconds=args.max_seconds, max_matches=args.max_matches))
    try:
        write_report(args.out, report, overwrite=args.overwrite)
    except OSError as exc:
        print(f"Cannot write report: {exc}", file=sys.stderr)
        return 1
    print(f"Report written: {args.out}")
    incomplete = [scan for scan in report["scans"] if not scan["complete"] or scan["matches_truncated"]]
    if incomplete:
        print("WARNING: some scan results are incomplete/truncated; inspect errors and stop_reason.", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
