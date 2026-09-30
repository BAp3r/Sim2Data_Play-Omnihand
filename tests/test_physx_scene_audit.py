"""Contact-buffer safety checks; these tests are not a PhysX acceptance."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from physx_scene_audit import validate_contact_matrix


class Matrix:
    def __init__(self, shape, value=0.0):
        self.shape = shape
        self.value = value

    def __getitem__(self, index):
        return [[self.value] * 3 for _ in range(self.shape[1])]


class ContactMatrixTests(unittest.TestCase):
    def test_single_sensor_retains_filter_columns(self):
        report = validate_contact_matrix(Matrix((1, 2, 3)), ["table", "hand"],
                                         sensor_count=1, filter_count=2)
        self.assertEqual(report["matrix_shape"], [1, 2, 3])

    def test_extra_sensors_cannot_be_flattened_into_filter_columns(self):
        for shape in ((2, 2, 3), (2, 3), (1, 1, 3)):
            with self.assertRaises(ValueError):
                validate_contact_matrix(Matrix(shape), ["table", "hand"])

    def test_nonfinite_forces_and_bad_filter_metadata_fail_closed(self):
        for value in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                validate_contact_matrix(Matrix((1, 2, 3), value), ["table", "hand"])
        for arguments in ({"sensor_count": 0}, {"filter_count": 0}):
            with self.assertRaises(ValueError):
                validate_contact_matrix(Matrix((1, 2, 3)), ["table", "hand"], **arguments)
        with self.assertRaises(ValueError):
            validate_contact_matrix(Matrix((1, 2, 3)), ["table", "table"])


if __name__ == "__main__":
    unittest.main()
