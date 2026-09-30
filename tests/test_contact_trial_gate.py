"""Checks of the contact evidence predicate, not physics acceptance."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from isaac_contact_trial import contact_lift_gate, contact_release_gate, state_is_bounded, validate_planned_scene, validate_preload, validate_hand_plan


class ContactGateTests(unittest.TestCase):
    def test_release_requires_final_stable_support_and_no_hand(self):
        row=self.row();row.update(phase="retreat", support_force_N=.08*9.81, hand_contact_force_N=0.)
        self.assertTrue(contact_release_gate([row]*1000,.08,.001)["passed"])
        bad=dict(row,hand_contact_force_N=.2)
        self.assertFalse(contact_release_gate([row]*1000+[bad],.08,.001)["passed"])
        self.assertFalse(contact_release_gate([dict(row,support_force_N=0.)]*1000,.08,.001)["passed"])

    def test_plan_cannot_restore_thumb_rotation_or_unmapped_reset(self):
        import json, copy
        profile=json.loads((Path(__file__).parents[1]/"configs/commissioning.synthetic.json").read_text())
        for side in ("left","right"):
            active=profile["gripper_commissioning"][side]["active_joints"]
            opened=[a["open_rad"] for a in active]
            plan=dict(side=side,hand_open=opened,hand_close=[a["close_rad"] for a in active],
                      start_configuration={"hand":opened})
            validate_hand_plan(profile,plan)
            bad=copy.deepcopy(plan);bad["hand_close"][0]+=.1
            with self.assertRaises(ValueError):validate_hand_plan(profile,bad)
            bad=copy.deepcopy(plan);bad["start_configuration"]["hand"][0]+=.1
            with self.assertRaises(ValueError):validate_hand_plan(profile,bad)
            # Source zero is a valid direct open pose for the right open jaw.
            # Reject a mismatch with the mapped pose, not zero as a number.
            if side == "right":
                self.assertEqual(opened, [0.]*10)
                validate_hand_plan(profile,plan)

    def test_preload_rejects_fixed_hand_channels(self):
        request = dict(plan_sha256="p", profile_sha256="c", urdf_sha256="u",
                       joint_names=["thumb_roll", "thumb_mcp"], effort_Nm=[.1, .2],
                       synthetic=True, requested_force_is_not_measured=True)
        kwargs = dict(plan_sha="p", profile_sha="c", urdf_sha="u",
                      active_names=request["joint_names"], effort_limits=[1., 1.],
                      allowed_effort_names=["thumb_mcp"])
        with self.assertRaisesRegex(ValueError, "fixed hand"):
            validate_preload(request, **kwargs)
        request["effort_Nm"][0] = 0.
        self.assertEqual(validate_preload(request, **kwargs), [0., .2])

    def test_preload_identity_order_and_effort_bounds(self):
        import copy
        request = dict(plan_sha256="p", profile_sha256="c", urdf_sha256="u",
                       joint_names=["active1", "active2"], effort_Nm=[.2, -.3],
                       synthetic=True, requested_force_is_not_measured=True)
        def check(value):
            return validate_preload(value, plan_sha="p", profile_sha="c", urdf_sha="u",
                                    active_names=["active1", "active2"], effort_limits=[1., 1.])
        self.assertEqual(check(request), [.2, -.3])
        for key, value in (("plan_sha256", "stale"), ("urdf_sha256", "other"),
                           ("joint_names", ["active2", "active1"]),
                           ("joint_names", ["active1", "mimic"]),
                           ("effort_Nm", [.2]), ("effort_Nm", [1.01, 0]),
                           ("effort_Nm", [float("nan"), 0]), ("effort_Nm", [True, 0]),
                           ("synthetic", False), ("requested_force_is_not_measured", False)):
            invalid = copy.deepcopy(request); invalid[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                check(invalid)

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
