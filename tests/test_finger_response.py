"""CPU checks of the commissioning acceptance gate, not PhysX validation."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("finger_response", Path(__file__).parents[1] / "scripts/isaac_finger_response.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ResponseGateTests(unittest.TestCase):
    def check_gate(self, movement=True, residual=0.001, effort=0.1, stages=4):
        return module.response_passed(
            [{"hand_max_abs_error_rad": 0.01}] * stages,
            [{"passed": movement}], [{"max_abs_residual_rad": residual}],
            [{"measured_effort": None if effort is None else [effort]}], 0.15)

    def test_complete_evidence(self):
        self.assertTrue(self.check_gate())

    def test_zero_motion_cannot_pass_tracking(self):
        self.assertFalse(self.check_gate(movement=False))

    def test_mimic_failure(self):
        self.assertFalse(self.check_gate(residual=0.2))
        self.assertFalse(self.check_gate(residual=float("nan")))

    def test_incomplete_or_nonfinite_readback(self):
        self.assertFalse(self.check_gate(effort=None))
        self.assertFalse(self.check_gate(effort=float("nan")))
        self.assertFalse(self.check_gate(stages=3))
