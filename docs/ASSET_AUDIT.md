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
| OmniHand SDK table | Expected source is `API_PYTHON_O10.md` at commit `026740d9fdd8ba32b0605fa702a992b322076f1b`. No retained raw file or command output exists in this worktree. | No SDK fetch was performed in this correction. | All detailed SDK limits, ranges, channel counts and active order are marked `UNVERIFIED_SOURCE_CLAIM_PENDING_FETCH`; they are candidate notes only. The nearby D evidence is LeRobot-only and is not SDK evidence. |

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

The O10 records contain ten claimed active channels and six claimed mimic
relationships per side in the private URDF snapshots. The
`state_channel_count_before_tactile=16` entry is explicitly an unverified SDK
layout claim, not an observation vector dimension. `state_observation_dimension`
is null and physical feedback versus SDK-derived values still requires an
observability audit. The CAD filename `play-zy-o10(1).stp` is not model or
hardware identity evidence; it cannot establish that both installed hands are
O10.

## Next interfaces and blockers

The next A/B consumer may use the relative card-box path, source hashes,
`link6` boundary candidates, and the private evidence references. Before an
actuator or contact scene is bound, it must receive:

- an approved AIRBOT hardware/model revision and source asset package;
- an O10/O12 badge and anatomical side confirmation for each hand;
- a retained official OmniHand archive and `API_PYTHON_O10.md` evidence bound
  to its commit;
- the flange-to-mount-to-hand-root and camera-housing-to-optical transforms;
- measured hand/mount/camera mass properties and card-box physical properties;
- a Kit/PhysX scene-open, dependency, collision, and RGB check.

No contact task C or production collection is enabled by this audit. The GPU
smoke remains blocked by the previously recorded RTX/Vulkan startup failure
and resource contention; that failure is not evidence about the card box.
