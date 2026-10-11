"""Export executed joint/TCP samples without promoting incomplete runs to success.

TCP/end-link composition follows the rigid-frame convention used by RoboTwin's
Robot._trans_from_gripper_to_endlink and Base_Task.get_place_pose. Robot-specific
offsets are derived from this simulation's accepted grasp, never copied from a
different gripper. No third-party runtime or source code is vendored.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from plan_contact_trajectory import RobotModel, pose, positions_for, sha256


def export_trajectory(spec_path, episode, output):
    spec=json.loads(Path(spec_path).read_text(encoding='utf-8'))
    root=Path(episode); output=Path(output);output.mkdir(parents=True,exist_ok=False)
    report=json.loads((root/'result.json').read_text(encoding='utf-8'))
    capture=json.loads((root/'capture/capture.json').read_text(encoding='utf-8'))
    profile=json.loads(Path(spec['profile']).read_text(encoding='utf-8'))
    models={}; bases={}; offsets={}; active={};indices={};plans={}
    for side in ('left','right'):
        model=models[side]=RobotModel(spec[side]['urdf'],side)
        bases[side]=pose(profile['robots'][side]['base_xyz_m'],profile['robots'][side]['base_rpy_rad'])
        plan=plans[side]=json.loads(Path(spec[side]['plan']).read_text(encoding='utf-8'))
        active[side]=profile['gripper_commissioning'][side]['active_joints']
        mapped=[dict(a,open_rad=q,close_rad=q) for a,q in zip(active[side],plan['hand_close'])]
        palm=bases[side]@model.fk(positions_for(model.joints,mapped,model.arm_names,plan['grasp'],1))[f"{side}_hand__{'l' if side=='left' else 'R'}_palm"]
        # Task TCP: centre of the nominal grasped box, axes parallel to palm.
        tcp=palm.copy();tcp[:3,3]=plan['box_center']
        offsets[side]=np.linalg.inv(palm)@tcp
        indices[side]=list(range(6))+report['initialization'][side]['hand_open']['joint_indices']
    def tcp_for(side,values):
        model=models[side]
        mapped=[dict(a,open_rad=q,close_rad=q) for a,q in zip(active[side],values[6:])]
        fk=model.fk(positions_for(model.joints,mapped,model.arm_names,values[:6],1))
        matrix=bases[side]@fk[f"{side}_hand__{'l' if side=='left' else 'R'}_palm"]@offsets[side]
        q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
        return dict(position_m=matrix[:3,3].tolist(),quaternion_wxyz=q[[3,0,1,2]].tolist())
    count=0
    with (root/'capture/frames.jsonl').open() as f,(root/'capture/auxiliary.jsonl').open() as a,(output/'trajectory.jsonl').open('w') as dst:
        for line,auxline in zip(f,a,strict=True):
            frame=json.loads(line);aux=json.loads(auxline)
            if frame['index']!=aux['index']:raise ValueError('frame/command alignment mismatch')
            row=dict(index=frame['index'],timestamp_s=frame['timestamp'],physics_step=frame['physics_step'],
                     absolute_physics_step=aux['simulation']['physical_step'],phase=aux['simulation']['phase'],arms={})
            for side in ('left','right'):
                names=[f'{side}_arm__joint{i}' for i in range(1,7)]+[x['name'] for x in active[side]]
                measured=[aux['simulation'][side]['q'][i] for i in indices[side]]
                command=[aux['active_commands'][n] for n in names]
                row['arms'][side]=dict(joint_names=names,measured_q_rad=measured,
                    measured_qd_rad_s=[aux['simulation'][side]['qd'][i] for i in indices[side]],
                    commanded_q_rad=command,gripper_amount=frame['action'][6 if side=='left' else 13],
                    measured_tcp_world=tcp_for(side,measured),commanded_tcp_world=tcp_for(side,command))
            dst.write(json.dumps(row)+'\n');count+=1
    if count!=capture['frame_count']:raise ValueError('frame count mismatch')
    manifest=dict(synthetic=True,complete_episode=report['passed'],last_phase=report['phase'],frames=count,
        fps=capture['fps'],timing='pre-action measured q/TCP and command for following interval',
        tcp_definition='synthetic nominal grasp centre; palm-aligned axes; fixed per-side palm-to-TCP',
        T_palm_tcp={s:t.tolist() for s,t in offsets.items()},
        mimic_policy='FK expands source URDF mimic from measured independent joints; not separately observed passive state',
        source_result_sha256=sha256(root/'result.json'),
        input_urdf_sha256={s:sha256(Path(spec[s]['urdf'])) for s in models},
        gates=report['gates'],failure_reason=report.get('error'),
        reference='https://github.com/RoboTwin-Platform/RoboTwin/blob/main/envs/robot/robot.py')
    (output/'trajectory_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    for key in ('spec','episode','out'):parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args();print(json.dumps(export_trajectory(args.spec,args.episode,args.out),indent=2))
