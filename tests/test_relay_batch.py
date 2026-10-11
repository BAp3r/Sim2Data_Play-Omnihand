import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from relay_recording import REQUIRED_GATES
from run_relay_randomized import run


class BatchTests(unittest.TestCase):
    def test_counts_failed_seed_and_requires_every_success_gate(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);plan=root/'plan.json';spec=root/'spec.json'
            plan.write_text(json.dumps({'box_center':[-.23,.13,.0145]}))
            spec.write_text(json.dumps({'left':{'plan':str(plan)}}))
            args=SimpleNamespace(spec=spec,sim_python=Path(sys.executable),out=root/'batch',
                                 count=2,seed=10,export_python=None,export_pythonpath=[])
            seeds=[]
            def execute(job):
                seeds.append(json.loads(job.spec.read_text())['randomization_seed'])
                job.out.mkdir()
                gates={k:True for k in REQUIRED_GATES}
                if len(seeds)==1:gates['final_released_stable']=False
                (job.out/'result.json').write_text(json.dumps(dict(passed=True,gates=gates,
                    phase='complete',real_articulations=True)))
                (job.out/'process_exit.json').write_text(json.dumps({'normal_exit':False,'kit_close_timeout':True}))
                return 4
            with patch('run_relay_randomized.execute',side_effect=execute),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run(args),0)
            result=json.loads((args.out/'batch_result.json').read_text())
            self.assertEqual(seeds,[10,11])
            self.assertEqual(result['completed_attempts'],2)
            self.assertEqual(result['successful_episodes'],1)
            self.assertEqual([t['status'] for t in result['trials']],['failed','success'])
            self.assertTrue(all(not t['process_exit']['normal_exit'] for t in result['trials']))
