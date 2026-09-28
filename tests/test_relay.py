import unittest
import json
import sys
import tempfile
import importlib.util
from pathlib import Path
from sim2data.relay import RelayPhase, RelayStateMachine
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from export_relay import export_relay


class RelayTests(unittest.TestCase):
    def test_requires_each_stage_and_has_single_timeline(self):
        machine=RelayStateMachine(timeout_s=2)
        self.assertEqual(machine.tick(.01,False),RelayPhase.RESET_SETTLE)
        visited=[]
        while machine.phase not in (RelayPhase.COMPLETE, RelayPhase.FAILED):
            visited.append(machine.phase)
            machine.tick(.01,True)
        self.assertLess(visited.index(RelayPhase.LEFT_RELEASE),visited.index(RelayPhase.RIGHT_CLOSE))
        self.assertLess(visited.index(RelayPhase.LEFT_RETREAT),visited.index(RelayPhase.RIGHT_APPROACH))
        self.assertIn(RelayPhase.RIGHT_TRANSFER,visited)
        self.assertIn(RelayPhase.RIGHT_LOWER,visited)
        self.assertEqual(machine.phase, RelayPhase.COMPLETE)

    def test_timeout_and_fatal_contact_failure(self):
        machine=RelayStateMachine(timeout_s=.1)
        self.assertEqual(machine.tick(.2,False), RelayPhase.FAILED)
        machine=RelayStateMachine(); self.assertEqual(machine.tick(.01,False,fatal=True), RelayPhase.FAILED)

    def test_failed_trial_cannot_create_a_dataset(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/"capture.json").write_text(json.dumps({
                "sample_kind":"synthetic_physx_dual_arm_relay","task_success":False}))
            with self.assertRaisesRegex(ValueError,"successful"):
                export_relay(root,root/"dataset")
            self.assertFalse((root/"dataset").exists())

    def test_fatal_failure_wins_over_transition(self):
        self.assertEqual(RelayStateMachine().tick(.01,True,fatal=True),RelayPhase.FAILED)


@unittest.skipUnless(importlib.util.find_spec("numpy") and importlib.util.find_spec("scipy"), "scientific geometry dependencies unavailable")
class FootprintTests(unittest.TestCase):
    def test_center_inside_does_not_accept_overhanging_box(self):
        from isaac_relay import footprint_inside
        self.assertTrue(footprint_inside([0,0,0],[1,0,0,0],[.08,.06,.06],[0,0],[.2,.2]))
        self.assertFalse(footprint_inside([.09,0,0],[1,0,0,0],[.08,.06,.06],[0,0],[.2,.2]))

    def test_rotated_corners_are_checked(self):
        from isaac_relay import footprint_inside
        import math
        self.assertFalse(footprint_inside([0,0,0],[math.cos(math.pi/8),0,0,math.sin(math.pi/8)],
                                         [.18,.18,.06],[0,0],[.2,.2]))
