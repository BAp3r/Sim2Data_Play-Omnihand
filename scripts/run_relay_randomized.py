"""Sequential synthetic pose experiments using the existing relay executor.

Every seed is counted, including planning rejections. Failed episodes never
enter a successful dataset; process cleanup and physics outcomes are separate.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys
import time

from run_relay_episode import execute
from relay_recording import REQUIRED_GATES
from relay_randomization import initial_pose


def write(path, value):
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value,indent=2),encoding='utf-8')
    tmp.replace(path)


def run(args):
    if args.count<1:raise ValueError('positive count required')
    root=Path(__file__).resolve().parents[1]
    recipe=json.loads(args.spec.read_text(encoding='utf-8'))
    plan=json.loads((root/recipe['left']['plan']).read_text(encoding='utf-8'))
    args.out.mkdir(parents=True,exist_ok=False)
    spec_dir=args.out/'specs';spec_dir.mkdir()
    status=dict(synthetic=True,scope='development_randomized_relay_experiments',
                requested_attempts=args.count,started_attempts=0,completed_attempts=0,
                successful_episodes=0,trials=[],status='running',
                baseline_spec_sha256=hashlib.sha256(args.spec.read_bytes()).hexdigest(),
                git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
                code_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                             (root/'scripts').glob('*.py')})
    write(args.out/'batch_result.json',status)
    for index in range(args.count):
        seed=args.seed+index
        name=f'trial_{index:03d}_seed_{seed}'
        out=(args.out/name).resolve();spec=(spec_dir/(name+'.json')).resolve()
        write(spec,dict(recipe,randomization_seed=seed))
        sample=initial_pose(seed,plan['box_center'])
        entry=dict(index=index,seed=seed,randomization=sample,out=str(out),status='running')
        status['trials'].append(entry);status['started_attempts']+=1
        write(args.out/'batch_result.json',status)
        started=time.monotonic()
        try:
            code=execute(SimpleNamespace(sim_python=args.sim_python.resolve(),spec=spec,out=out,
                close_timeout=60,runtime_timeout=1800,use_uv=True,
                export_python=args.export_python,export_pythonpath=args.export_pythonpath))
            result=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {}
            physics_success=result.get('passed') is True and all(result.get('gates',{}).get(k) is True for k in REQUIRED_GATES)
            entry.update(status='success' if physics_success else 'failed',executor_exit_code=code,
                phase=result.get('phase'),gates=result.get('gates',{}),error=result.get('error'),
                frames=result.get('capture',{}).get('frame_count',0),
                real_articulations=result.get('real_articulations',False),
                physics_success=physics_success,
                process_exit=json.loads((out/'process_exit.json').read_text()))
            if physics_success:status['successful_episodes']+=1
        except Exception as exc:
            entry.update(status='executor_error',physics_success=False,error=repr(exc))
        entry['elapsed_s']=time.monotonic()-started
        status['completed_attempts']+=1
        status['failure_phases']=dict(Counter(t.get('phase','executor_error') for t in status['trials']
                                             if t.get('status') not in ('success','running')))
        write(args.out/'batch_result.json',status)
        print(json.dumps({k:entry.get(k) for k in ('index','seed','status','phase','frames','elapsed_s')}),flush=True)
        # Infrastructure faults cannot test the randomized task. Preserve the
        # current seed and stop rather than count repeated startup errors.
        if not entry.get('real_articulations'):
            status['status']='infrastructure_blocked';break
    else:
        status['status']='completed'
    write(args.out/'batch_result.json',status)
    return 0 if status['status']=='completed' else 2


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('spec','sim-python','out'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--count',type=int,default=20)
    p.add_argument('--seed',type=int,default=2026100900)
    p.add_argument('--export-python',type=Path)
    p.add_argument('--export-pythonpath',action='append',default=[])
    raise SystemExit(run(p.parse_args()))
