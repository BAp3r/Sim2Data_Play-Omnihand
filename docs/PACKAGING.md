# Packaging and runtime boundaries

This change prepares the package layout for the next source move. It does not
move Python files, create Git submodules, edit the root `pyproject.toml` or
`uv.lock`, or claim that either runtime can be resolved on this machine.

## Target layout

The main session should move the current source into three `src/` trees:

```text
packages/
  sim2data_core/src/sim2data/
    __init__.py                 # compatibility namespace and version owner
    core.py
    preflight.py
    assets/
  sim2data_isaac/src/sim2data/
    __init__.py                 # namespace extension
    backends/isaaclab/
  sim2data_lerobot/src/sim2data/
    __init__.py                 # namespace extension
    export/
environments/
  sim/pyproject.toml + uv.lock  # Isaac Lab runtime
  data/pyproject.toml + uv.lock # LeRobot runtime
third_party/
  IsaacLab/                     # Git submodule, selected upstream commit
  lerobot/                      # Git submodule, selected upstream commit
```

The package projects use `setuptools` `src` discovery and the distribution
names `sim2data-core`, `sim2data-isaac`, and `sim2data-lerobot`. The two
adapter distributions depend only on `sim2data-core`. Their heavy upstream
dependencies belong to the corresponding environment project, so installing
the three editable projects for CPU contract tests does not pull Isaac Lab,
Isaac Sim, CUDA, Torch, or LeRobot.

## Import compatibility

The current repository has one regular `sim2data` package and its
`__init__.py` only exposes `__version__`. After the source move, the main
session must make each distribution's `sim2data/__init__.py` use the same
`pkgutil.extend_path` namespace pattern. Keep `__version__` in the core
distribution and keep the other two initializers side-effect free. Also add
the normal package initializers for `sim2data.assets`,
`sim2data.backends`, `sim2data.backends.isaaclab`, and
`sim2data.export` where the moved source requires them.

With that change, these existing imports remain valid:

```python
from sim2data.core import Timing
from sim2data.preflight import assess_scene
from sim2data.assets.manifest import validate_manifest
from sim2data.backends.isaaclab.scene_frames import SE3
```

The namespace initializer is a source-move task and is intentionally absent
from this packaging-only commit. A regular package initializer without
`extend_path` would mask the other editable distributions depending on path
order.

## Environment sources and locks

`environments/sim/pyproject.toml` declares local editable sources for the core
and Isaac adapters plus a non-editable path source for
`../../third_party/IsaacLab`. It records the requested Isaac Lab identity in
the README: version `2.3.0`, commit
`3c6e67bb5c7ada942a6d1884ab69338f57596f77`.

`environments/data/pyproject.toml` follows the same pattern and points
`lerobot` at `../../third_party/lerobot`. Its requested identity is version
`0.5.2`, commit `377e82c9a61887803d079f29a7a3688e51d2ea05`.

These path sources are deliberate. They prevent a lock from silently selecting
an index release that differs from the reviewed source. The submodules and
their package metadata must exist before uv can resolve either project. The
main session should verify the submodule commit and package project name,
then run the actual lock command in each environment and commit the generated
`uv.lock`. No lock is fabricated here.

The root CPU project should later use local editable sources for all three
lightweight packages and keep the Isaac/LeRobot path sources out of that
project. A root CPU sync is a contract-test environment only; it cannot prove
Isaac physics, rendering, or LeRobot writer compatibility.

## Migration blockers

The current `preflight.py` defaults to the repository-relative
`configs/scene_spec.draft.json`. Once it is installed from an arbitrary
environment, that relative path is not a reliable package resource. Before
moving the source, the main session must either require an explicit `--scene`
path for CLI use or add a documented repository/config resolver; do not hide a
machine path or copy a private calibration file into a wheel.

The Isaac Lab candidate currently visible locally (`0.47.3`, with extension
`0.47.2`) is not the requested upstream identity, and the prior Isaac 5.1
smoke exited 139 without a bound runtime. The LeRobot source has not yet been
locked or used for the official writer/loader roundtrip. Both remain runtime
blockers. Creating these TOMLs is not an SDK, GPU, or dataset validation.

## Lightweight checks

Before source moves, these files can be checked without network access:

```powershell
uv run --frozen --offline --no-python-downloads python -c "import tomllib; from pathlib import Path; [tomllib.loads(p.read_text(encoding='utf-8')) for p in Path('packages').glob('*/pyproject.toml')]; [tomllib.loads(p.read_text(encoding='utf-8')) for p in Path('environments').glob('*/pyproject.toml')]; print('TOML_OK')"
```

Do not replace the missing `uv.lock` files with hand-written lock content. The
actual upstream checkout, interpreter, package metadata and dependency graph
must be available when the main session performs the two runtime resolves.
