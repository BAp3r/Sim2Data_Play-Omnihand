"""Checks of the contact evidence predicate, not physics acceptance."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from isaac_contact_trial import contact_lift_gate, state_is_bounded, validate_planned_scene


class ContactGateTests(unittest.TestCase):
    def row(self, **kwargs):
        return dict(phase="hold", box_position=[0,0,.14], support_force_N=0,
                    ground_force_N=0, hand_contact_force_N=.4, box_velocity=[0,0,0,0,0,0],
                    q=[0], qd=[0], effort=[0], box_quaternion_wxyz=[1,0,0,0], **kwargs)

    def test_continuous_contact_lift(self):
        self.assertTrue(contact_lift_gate([self.row() for _ in range(240)], .1)["passed"])

    def test_toss_or_table_support_cannot_pass(self):
        for key,value in (("hand_contact_force_N",0),("support_force_N",.4),
                          ("ground_force_N",.4),("box_velocity",[0,0,.5,0,0,0])):
            row=self.row();row[key]=value
            self.assertFalse(contact_lift_gate([row]*480,.1)["passed"])

    def test_interrupted_hold_is_not_accumulated(self):
        rows=[self.row() for _ in range(480)]
        for i in (100,200,300,400):rows[i]["hand_contact_force_N"]=0
        self.assertFalse(contact_lift_gate(rows,.1)["passed"])

    def test_invalid_or_explosive_readback_fails_closed(self):
        for key, value in (("q", [float("nan")]), ("qd", [51]), ("effort", [None]),
                           ("box_position", [0, 0, float("inf")]),
                           ("box_velocity", [0, 0, 0, 100, 0, 0])):
            row = self.row(); row[key] = value
            self.assertFalse(state_is_bounded(row))
            self.assertFalse(contact_lift_gate([row]*480, .1)["passed"])

    def test_invalid_time_step_cannot_inflate_hold(self):
        for dt in (0, -1, float("nan"), float("inf"), 2):
            with self.assertRaises(ValueError):
                contact_lift_gate([self.row()], .1, dt)

    def test_hold_uses_actual_physics_rate(self):
        self.assertFalse(contact_lift_gate([self.row()]*240, .1, 1/1000)["passed"])
        self.assertTrue(contact_lift_gate([self.row()]*1000, .1, 1/1000)["passed"])

    def test_rejected_or_stale_plan_cannot_execute(self):
        with self.assertRaises(ValueError):
            validate_planned_scene({}, "current")
        plan = dict(scene_kind="thor_cardbox", coordinate_frame="world", profile_sha256="current",
                    execution_allowed=False)
        with self.assertRaises(ValueError):
            validate_planned_scene(plan, "current")
        validate_planned_scene(plan, "current", scene_only=True)
        plan["execution_allowed"] = True
        validate_planned_scene(plan, "current")
        with self.assertRaises(ValueError):
            validate_planned_scene(plan, "changed")
        plan["coordinate_frame"] = "base"
        with self.assertRaises(ValueError):
            validate_planned_scene(plan, "current", scene_only=True)
