# Sim2Data simulation environment

This project is the runtime boundary for Isaac Lab. It consumes the local
`sim2data-core` and `sim2data-isaac` packages through editable path sources and
resolves `isaaclab` only from the checked out `../../third_party/IsaacLab`
submodule.

The declared upstream identity is Isaac Lab `v2.3.0`, commit
`3c6e67bb5c7ada942a6d1884ab69338f57596f77`. The submodule must be checked out
at that commit before creating `uv.lock`; this branch does not generate a lock
file or download Isaac Sim, CUDA, or GPU assets.

The current local runtime candidates (`0.47.3` and extension `0.47.2`) are not
silently substituted for the declared upstream. A package name/version or
Python/Isaac Sim compatibility mismatch must be resolved during the actual
runtime lock and smoke step.
