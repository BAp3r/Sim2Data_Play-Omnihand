# Asset audit evidence and binding state

This document records the evidence behind `configs/asset_manifest.json`. It is
an M1 implementation record, not a physical acceptance report. The manifest is
deliberately `STATIC_EVIDENCE_INDEXED_PHYSICAL_VALIDATION_PENDING`, and
`production_collection_allowed` remains `false`.

## Evidence ledger

| Item | Evidence and provenance | This turn | Binding limit |
| --- | --- | --- | --- |
| Isaac Sim card box USD | `.local/evidence/m1/card_box_usd_parse_20260922.json`; raw command output is `.local/evidence/m1/card_box_usd_parse_command_output_20260922.json` and the private stdin parser is `.local/evidence/m1/card_box_usd_parse_20260922.py` | A single named USD and seven explicitly named dependency files were opened or hashed read-only through the existing Isaac 6.1 candidate interpreter. SSH connection timeout was 10 s and remote command timeout was 40 s. No directory scan or GPU process was started. | The report proves static USD metadata and file presence. It does not prove Kit shader resolution, PhysX cooking, contact behavior, mass, inertia, rigidity, or grasp suitability. |
| AIRBOT ROS2 candidate | `.local/worktrees/a-assets/reports/github_urdf/foiegreis_airbot_play.urdf` and `foiegreis_LICENSE`; pinned source commit `7792960fb60f3118d9827b641dcc441e9ae2d06f` | Snapshot and license were produced by the earlier A audit and reused read-only. Their SHA256 values were rechecked this turn with `Get-FileHash -Algorithm SHA256`. | The pinned URL identifies the claimed source revision; this turn did not refetch the Git object or resolve the mesh package. The six names are URDF candidate joints, not confirmed motor IDs or action channels. |
| DISCOVERSE AIRBOT candidate | `.local/worktrees/a-assets/reports/github_urdf/discoverse_airbot_play_v3_gripper_fixed.urdf`; pinned source commit `d67f47c084aba0e0cf422a8725235f8b9238655a` | URDF snapshot and hash were reused from the earlier A audit and rechecked read-only. The earlier raw repository license snapshot was not retained. | `MIT` is a source claim carried from the earlier audit and remains license evidence pending. The fixed gripper and all arm names remain candidate geometry only. |
| OmniHand private URDFs | `.local/worktrees/a-assets/reports/remote_omnihand_left.urdf`, `remote_omnihand_right.urdf`, and `remote_omnihand_LICENSE` | Snapshots and hashes were reused from the earlier A audit and rechecked read-only. | These are private URDF snapshots. The official model ZIP archives were not fetched or hash-bound, so the snapshot cannot be called an official archive match. |
| OmniHand SDK table | `.local/evidence/m1/API_PYTHON_O10_026740d_20260922.md`; source index `.local/evidence/m1/omnihand_sdk_source_audit_20260922.json`; README path discovery is `.local/evidence/m1/agillink_README_026740d_20260922.md` | `git ls-remote --symref` resolved `refs/heads/main` to `026740d9fdd8ba32b0605fa702a992b322076f1b`. The README and `doc/en/API_PYTHON_O10.md` were fetched as small text; the official O10 page was fetched separately. | The official SDK/API claims are now source-verified. Hardware identity, official model ZIP hash binding, and simulator/observation mapping remain open. |

The exact private connection command and captured output for the card box are
kept under `.local/evidence/m1/`; public files refer to logical paths only and
do not contain the NAS or server address. The command form was:

```powershell
Get-Content .local/evidence/m1/card_box_usd_parse_20260922.py -Raw |
  ssh -o BatchMode=yes -o ConnectTimeout=10 `
      -o ServerAliveInterval=5 -o ServerAliveCountMax=1 `
      <server-from-.local/paths.json> `
      "timeout 40s <Isaac-6.1-python-from-.local/paths.json> -"
```

The private record contains the concrete host and interpreter values. The
parser used `pxr.Usd`, `UsdGeom`, `UsdPhysics`, and `Sdf.AssetPath`; it did not
modify the source stage.

The SDK source commands and hashes are recorded in the private source index.
The repository check returned `refs/heads/main` at
`026740d9fdd8ba32b0605fa702a992b322076f1b`. README path discovery selected
`doc/en/API_PYTHON_O10.md`; its Git blob is
`d479e89cff3e2b38b33ba2d5a5f0747f3107a72d` and the decoded raw-file SHA256 is
`44df6abf37de24ebdfa81d96f0471cff92f3612c69a10eee18149b418fd9875d`. The
README and API document state O10 has 10 active plus 6 passive degrees of
freedom, a 0--4096 motor-position range, ten active angle values, and sixteen
total active-plus-passive angle values. The API table supplies the left/right
joint order, angle limits, and velocity limits already listed in the manifest.
The official O10 page capture verifies the four 20260827 URDF/model links;
the ZIP files themselves were not downloaded or hashed. The verified README/API
documents do not state an SDK release number, so the manifest leaves
`sdk_version` null rather than retaining the earlier unverified `1.1.8` claim.
A bounded tag listing was attempted with the same low-speed limit but timed out
before refs were returned; that failure is retained in the private source index.

## Card box result

The selected candidate is
`5.1.0/Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01.usd`
with SHA256
`7401195d235442dd5c19fb7408ec0e67bfb7aa8c1235e97d313c90b3fbb4985d`.
The stage reports `metersPerUnit=1`, Z-up, default prim `/Root`, and mesh
`/Root/SM_CardBoxA_01`. Its authored extent is approximately
`[-35,-25,0]` to `[35,25,50]` with a local scale of `0.01`, giving world
bounds of approximately `0.70 x 0.50 x 0.50 m`.

The mesh has enabled collision APIs and `boundingCube` approximation in the
static file. No rigid-body prim is authored. The current parse did not capture
PhysX cooked-data status, so the manifest leaves that field unresolved. Mass,
center of mass, inertia, contact acceptance, and physical rigidity are all
unknown. This warehouse prop is therefore a visual/collision prop candidate,
not a bound desktop task target. It cannot be used directly for tabletop
grasping until a dynamic scene wrapper has an approved physical model or an
explicit audited approximation.

The direct material references are `MI_LampCeilingA.mdl`,
`T_CardBoxA_D.png`, `T_CardBoxA_N.png`, and `T_CardBoxA_ORM.png`. The MDL
file's recorded resource tokens add
`Alum_Anodized_roughness.png`, `T_Floor_01_D.png`, and `T_Floor_01_N.png` to
the dependency closure. All seven named files were present and hashed in the
private parse record. This is file-closure evidence only; MDL module loading
and shader resolution remain a runtime check.

## Robot and hand boundary

Both AIRBOT files expose a `link6` flange boundary and six revolute joint names
in the static URDF snapshots. The manifest now labels `active_joint_count` and
`command_channel_names` as URDF revision candidates. Hardware actuator IDs,
zero offsets, signs, modes, and action dimension remain null. The original
gripper and any original camera frame are not reused for the OmniHand/D405
assembly.

The O10 records contain ten active channels and six claimed mimic relationships
per side in the private URDF snapshots. The official API confirms ten active
values and sixteen total active-plus-passive angle values. The
`state_channel_count_before_tactile=16` entry therefore describes the SDK
angle-readback layout; it is explicitly not an observation vector dimension.
`state_observation_dimension` is null because physical feedback versus
SDK-derived passive values still requires an observability audit. The API also
documents 1D tactile readback, but its simulator feature and real sensor
binding remain open. The CAD filename `play-zy-o10(1).stp` is not model or
hardware identity evidence; it cannot establish that both installed hands are
O10.

## Next interfaces and blockers

The next A/B consumer may use the relative card-box path, source hashes,
`link6` boundary candidates, and the private evidence references. Before an
actuator or contact scene is bound, it must receive:

- an approved AIRBOT hardware/model revision and source asset package;
- an O10/O12 badge and anatomical side confirmation for each hand;
- a retained official OmniHand model archive, with its ZIP hash bound to the
  official page links (the API document is now commit-bound);
- the flange-to-mount-to-hand-root and camera-housing-to-optical transforms;
- measured hand/mount/camera mass properties and card-box physical properties;
- a Kit/PhysX scene-open, dependency, collision, and RGB check.

No contact task C or production collection is enabled by this audit. The GPU
smoke remains blocked by the previously recorded RTX/Vulkan startup failure
and resource contention; that failure is not evidence about the card box.
