"""CPU checks of the commissioning acceptance gate, not PhysX validation."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("finger_response", Path(__file__).parents[1] / "scripts/isaac_finger_response.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ResponseGateTests(unittest.TestCase):
    def check_gate(self, movement=True, residual=0.001, effort=0.1, stages=4):
        stage_rows, movement_rows, mimic, frames, evidence = self.strict_fixture()
        movement_rows[0]["passed"] = movement
        mimic[0]["max_abs_residual_rad"] = residual
        for frame in frames:
            frame["measured_effort"] = None if effort is None else [effort]*3
        return module.response_passed(
            stage_rows[:stages], movement_rows, mimic, frames, 0.15, evidence)

    def test_complete_evidence(self):
        self.assertTrue(self.check_gate())

    def test_missing_runtime_sidecars_cannot_authorize_contact(self):
        stages, movement, mimic, frames, _ = self.strict_fixture()
        self.assertFalse(module.response_passed(stages, movement, mimic, frames, .15))

    def test_zero_motion_cannot_pass_tracking(self):
        self.assertFalse(self.check_gate(movement=False))

    def test_mimic_failure(self):
        self.assertFalse(self.check_gate(residual=0.2))
        self.assertFalse(self.check_gate(residual=float("nan")))

    def test_incomplete_or_nonfinite_readback(self):
        self.assertFalse(self.check_gate(effort=None))
        self.assertFalse(self.check_gate(effort=float("nan")))
        self.assertFalse(self.check_gate(stages=3))

    @staticmethod
    def strict_fixture():
        names = ["arm", "hand", "mimic"]
        stages = [
            {"phase_index": i, "amount": amount, "samples": 2,
             "hand_max_abs_error_rad": 0.01, "arm_max_abs_drift_rad": 0.001}
            for i, amount in enumerate(module.AMOUNTS)
        ]
        frames = []
        for phase_index, amount in enumerate(module.AMOUNTS):
            frames.append({
                "phase_index": phase_index, "amount": amount,
                "q": [0.0, amount, amount * 2.0], "qd": [0.0, 0.0, 0.0],
                "measured_effort": [0.0, 0.1, 0.0],
                "commanded": {"arm": 0.0, "hand": amount},
            })
        mimic = [{"dof_name": "mimic", "source_dof_name": "hand",
                  "max_abs_residual_rad": 0.001, "commanded": False,
                  "usd_relationship": "/World/mimic:referenceJoint",
                  "usd_reference": "/World/hand", "usd_gearing": 1.0,
                  "usd_offset_degrees": 0.0}]
        evidence = {
            "expected_amounts": list(module.AMOUNTS), "expected_dof_count": len(names),
            "all_dof_names": names, "command_names": ["arm", "hand"],
            "mimic_names": ["mimic"],
            "reset": {
                "direct_joint_state_writes": 1,
                "arm_reset": {"joint_names": ["arm"], "q": [0.0]},
                "hand_open": {"joint_names": ["hand"], "target_q": [0.0],
                               "q_before": [0.0], "q_after": [0.0],
                               "direct_active_only": True, "mimic_commanded": False},
            },
            "limits": {name: {"lower": -2.0, "upper": 2.0, "max_velocity": 2.0,
                              "max_effort": 2.0, "stiffness": 4.0, "damping": 0.15}
                       for name in names},
            "gains": [[20.0, 4.0, 4.0], [2.0, 0.15, 0.15]],
            "max_efforts": [40.0, 1.5, 1.5],
            "mimic_relationships": mimic,
        }
        movement = [{"dof_name": "hand", "target_span": 1.0,
                     "phase_means": [0.0, 1.0, 0.0], "passed": True}]
        return stages, movement, mimic, frames, evidence

    def test_strict_runtime_evidence_requires_reset_and_command_subset(self):
        stages, movement, mimic, frames, evidence = self.strict_fixture()
        self.assertTrue(module.response_passed(stages, movement, mimic, frames, 0.15, evidence))

        missing_reset = dict(evidence)
        missing_reset["reset"] = {}
        self.assertFalse(module.response_passed(stages, movement, mimic, frames, 0.15, missing_reset))

        command_mimic = [dict(frames[0])]
        command_mimic[0]["commanded"] = {"arm": 0.0, "hand": 0.0, "mimic": 0.0}
        self.assertFalse(module.response_passed(stages, movement, mimic, command_mimic, 0.15, evidence))

    def test_endpoint_targets_require_current_limits(self):
        active = [{"dof_name": "hand", "open_rad": -0.5, "close_rad": 0.5}]
        properties = [{"lower": -1.0, "upper": 1.0, "hasLimits": True}]
        self.assertEqual(module._resolve_endpoint_targets(active, [0], properties)[:2], ([-0.5], [0.5]))
        active[0]["close_rad"] = 1.1
        with self.assertRaises(ValueError):
            module._resolve_endpoint_targets(active, [0], properties)
