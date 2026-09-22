# Sim2Data data environment

This project is the independent LeRobot writer/loader environment. It consumes
the local `sim2data-core` and `sim2data-lerobot` packages through editable path
sources and resolves `lerobot` only from the checked out
`../../third_party/lerobot` submodule.

The declared upstream identity is LeRobot `0.5.2`, commit
`377e82c9a61887803d079f29a7a3688e51d2ea05`. The submodule must be checked out
at that commit before creating `uv.lock`; this branch does not generate a lock
file or run the official SDK format roundtrip. The official writer/loader
smoke remains a D-stage validation.

Do not add Isaac Lab or simulator dependencies here. The environment must keep
dataset export and simulator dependencies independently lockable.
