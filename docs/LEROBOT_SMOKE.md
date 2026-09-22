# LeRobot v3 synthetic smoke

`sim2data.export` is a small adapter around the official LeRobot v3 writer. It
accepts a reviewed channel and camera schema, validates finite named vectors,
RGB shapes, and simulation clock evidence, then calls the SDK's
`create` → `add_frame` → `save_episode` → `finalize` flow. Each writer owns a
new output root and leaves privileged values in a sidecar JSON file.

The smoke schema deliberately uses three virtual, low resolution RGB streams:
`smoke_test_overhead`, `smoke_test_wrist_left`, and `smoke_test_wrist_right`.
It is only a format check and contains no robot, calibration, contact, or
physics evidence. Production collection remains disabled until the robot and
scene manifests are complete.

Run the integration tests in an environment that has the official package,
NumPy, Torch, and the `pyav` extra:

```text
python -m unittest tests.integration.test_lerobot_v3 -v
```

The standalone smoke runner uses one private root per invocation and does not
upload to the Hub:

```text
python scripts/lerobot_smoke.py --root <private-new-root> --report <private-report.json>
```

The runner writes two three-frame episodes, forces small data/video limits so
the episode video offsets are exercised, opens the finalized dataset with
delta-timestamp windows, reads a Torch batch, and reopens it without windows.
The integration test additionally checks all camera offsets, statistics,
decoded pixels on both sides of an episode boundary, episode-local window
padding, caller buffer reuse, empty/NaN/missing/delayed frame rejection,
sidecars, and single-writer ownership. A report path is opened exclusively so
an existing evidence report cannot be silently replaced.

The installed LeRobot package must expose `CODEBASE_VERSION == "v3.0"` and the
four methods `create`, `add_frame`, `save_episode`, and `finalize`. A package
that reports v3.0 but only exposes the older `save` method is rejected so the
result cannot be mistaken for this contract. Passing this smoke test is
evidence about serialization and loader behavior only; it is not M4 or a
physical grasp, timing, calibration, or dataset-quality acceptance.
