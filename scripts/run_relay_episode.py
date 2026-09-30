"""Own one simulation process, bound Kit cleanup, then export only a successful capture."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def execute(args):
    root=Path(__file__).resolve().parents[1]
    out=args.out.resolve()
    if out.exists():raise ValueError("fresh output directory required")
    out.parent.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env["OMNI_KIT_ACCEPT_EULA"]="YES"
    env["PYTHONPATH"]=os.pathsep.join(str(root/p) for p in ("packages/sim2data_core/src","scripts"))
    command=[str(args.sim_python),str(root/"scripts/isaac_relay.py"),"--spec",str(args.spec.resolve()),"--out",str(out)]
    (out.parent/(out.name+"_command.json")).write_text(json.dumps({"argv":command,"cwd":str(root)},indent=2),encoding="utf-8")
    started=time.monotonic();closing=None;terminated=False
    with (out.parent/(out.name+".log")).open("w",encoding="utf-8") as log:
        child=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name=="nt" else 0,start_new_session=os.name!="nt")
        while child.poll() is None:
            try:
                report=json.loads((out/"result.json").read_text(encoding="utf-8"))
                if report.get("shutdown",{}).get("attempted") and closing is None:closing=time.monotonic()
            except (OSError,json.JSONDecodeError):pass
            if (closing is not None and time.monotonic()-closing>args.close_timeout) or time.monotonic()-started>args.runtime_timeout:
                terminated=True
                if os.name=="nt":subprocess.run(["taskkill","/PID",str(child.pid),"/T","/F"],stdout=log,stderr=subprocess.STDOUT)
                else:os.killpg(child.pid,signal.SIGTERM)
                break
            time.sleep(1)
        code=child.wait(timeout=30)
    out.mkdir(exist_ok=True)
    exit_record=dict(pid=child.pid,exit_code=code,owner_terminated=terminated,
                     normal_exit=code==0 and not terminated,kit_close_timeout=terminated and closing is not None)
    (out/"process_exit.json").write_text(json.dumps(exit_record,indent=2),encoding="utf-8")
    report=json.loads((out/"result.json").read_text(encoding="utf-8")) if (out/"result.json").exists() else {}
    if not report.get("passed"):
        return 3
    if args.export_python is None:
        (out/"export_status.json").write_text(json.dumps({"exported":False,"reason":"official SDK interpreter not specified"}),encoding="utf-8")
        return 4
    env["PYTHONPATH"]=os.pathsep.join([str(root/"packages/sim2data_core/src"),str(root/"packages/sim2data_lerobot/src"),str(root/"scripts"),*args.export_pythonpath])
    export_command=[str(args.export_python),str(root/"scripts/export_relay.py"),"--capture",str(out/"capture"),"--out",str(out/"dataset")]
    with (out/"export.log").open("w",encoding="utf-8") as log:
        result=subprocess.run(export_command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800)
    (out/"export_status.json").write_text(json.dumps({"argv":export_command,"exit_code":result.returncode,
        "exported":result.returncode==0,"official_loader_report":"dataset/relay_readback.json"},indent=2),encoding="utf-8")
    return result.returncode


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    for key in ("sim-python","spec","out"):parser.add_argument("--"+key,type=Path,required=True)
    parser.add_argument("--export-python",type=Path)
    parser.add_argument("--export-pythonpath",action="append",default=[])
    parser.add_argument("--close-timeout",type=float,default=60)
    parser.add_argument("--runtime-timeout",type=float,default=1200)
    raise SystemExit(execute(parser.parse_args()))
