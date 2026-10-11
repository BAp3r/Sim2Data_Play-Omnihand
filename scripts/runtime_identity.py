"""Read selected runtime identities without importing Kit or modifying environments.

Output includes private local paths. Store it only in an ignored evidence directory.
An enclosing repository commit is explicitly NOT an upstream Isaac Lab identity.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata as metadata
import importlib.util
import json
from pathlib import Path
import platform
import subprocess
import sys


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inspect_runtime() -> dict:
    result = {"python": platform.python_version(), "executable": sys.executable,
              "packages": {}, "production_binding_approved": False}
    for name in ("isaacsim", "isaaclab", "torch", "numpy", "lerobot"):
        try:
            dist = metadata.distribution(name)
        except metadata.PackageNotFoundError:
            result["packages"][name] = None
            continue
        item = {"version": dist.version, "location": str(dist.locate_file("")),
                "direct_url": dist.read_text("direct_url.json")}
        result["packages"][name] = item
        if name != "isaaclab":
            continue
        spec = importlib.util.find_spec(name)
        if not spec or not spec.origin:
            continue
        source = Path(spec.origin).resolve()
        item["source"] = str(source)
        root = next((p for p in source.parents if (p / "isaaclab.sh").is_file()), None)
        if root is None:
            continue
        item["source_root"] = str(root)
        item["selected_source_sha256"] = {}
        for rel in ("VERSION", "source/isaaclab/config/extension.toml",
                    "source/isaaclab/isaaclab/__init__.py",
                    "source/isaaclab/isaaclab/sim/simulation_context.py",
                    "source/isaaclab/isaaclab/sim/simulation_cfg.py"):
            path = root / rel
            if path.is_file():
                item["selected_source_sha256"][rel] = digest(path)
        item["release_file"] = (root / "VERSION").read_text().strip() if (root / "VERSION").is_file() else None
        item["upstream_commit"] = None
        item["source_identity_status"] = "unversioned_copy" if not (root / ".git").exists() else "git_checkout_pending_review"
        if (root / ".git").exists():
            # Per-command trust exception only; never write global safe.directory.
            prefix = ["git", "-c", f"safe.directory={root}", "-C", str(root)]
            item["git"] = {}
            for command in (("rev-parse", "HEAD"), ("status", "--porcelain")):
                proc = subprocess.run(prefix + list(command), capture_output=True, text=True, timeout=20)
                item["git"][" ".join(command)] = {"code": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr}
    config = Path(sys.executable).parent.parent / "pyvenv.cfg"
    result["pyvenv_config"] = config.read_text() if config.is_file() else None
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = inspect_runtime()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
