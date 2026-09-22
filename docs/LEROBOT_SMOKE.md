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
uv run --no-project --offline --no-python-downloads --python <verified-sdk-python> python -m unittest tests.integration.test_lerobot_v3 -v
```

The standalone smoke runner uses one private root per invocation and does not
upload to the Hub:

```text
uv run --no-project --offline --no-python-downloads --python <verified-sdk-python> python scripts/lerobot_smoke.py --root <private-new-root> --report <private-report.json>
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

## Verified upstream source

The current official source check uses the LeRobot repository at commit
`b64fe1ed9f11eeac53ee821356d2797601701054` from
`https://github.com/huggingface/lerobot.git`. Its project metadata reports
`lerobot==0.6.2`, and its dataset metadata reports `CODEBASE_VERSION ==
"v3.0"`. The source exposes the required `create`, `add_frame`,
`save_episode`, and `finalize` methods; current releases call the RGB encoder
argument `rgb_encoder`, which the adapter detects while retaining compatibility
with older `camera_encoder`/`vcodec` surfaces.

The verification run unpacked the reviewed source archive, built a private
`lerobot-0.6.2` wheel using a packaging-only shim and no dependency resolution, and placed it in an
isolated overlay ahead of an existing Python 3.12.13 interpreter. The overlay did
not modify the shared environment or install large packages. It therefore
proves the official source plus the existing dependency set used by that
interpreter, not a fresh resolver-complete environment. The private identity,
stdout, and JSON reports are retained under
`.local/evidence/m1/export_official/`.

With that overlay, the remote Python 3.12.13 / Torch 2.11 environment passed
the 63-test suite present in the D worktree snapshot and the two official SDK integration tests. The
standalone runner passed both direct and offline `uv run` invocations: two
three-frame synthetic episodes, three H.264 RGB streams, statistics, video
boundary reads, episode-local windows, Torch batch shapes, sidecars, finalize,
and reopen all succeeded. A separate existing 0.4.3 environment also passed
the two integration tests through the legacy compatibility branch. These are
format and loader checks using synthetic data; they do not authorize physical
collection or establish the production dependency lock.

The earlier local 0.5.2 checkout was not authenticated against upstream; its
compatibility test is not the official-source acceptance record. The newly
generated `environments/data/uv.lock` is a separate 64-package candidate; the
overlay run did not install or validate that entire lock. Source hashes, the
packaging shim, and exact private commands remain in the evidence directory.
