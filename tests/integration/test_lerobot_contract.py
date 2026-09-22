"""Dependency-free checks for the export schema boundary."""

from __future__ import annotations

import unittest

from sim2data.export import ExportSchema


class ExportSchemaContractTests(unittest.TestCase):
    def test_camera_shapes_are_frozen(self) -> None:
        cameras = {"observation.images.confirmed": (4, 5, 3)}
        schema = ExportSchema(
            name="approved-schema",
            state_names=("state",),
            action_names=("action",),
            camera_shapes=cameras,
        )
        cameras["observation.images.confirmed"] = (8, 9, 3)
        self.assertEqual(schema.camera_shapes["observation.images.confirmed"], (4, 5, 3))
        with self.assertRaises(TypeError):
            schema.camera_shapes["observation.images.confirmed"] = (1, 1, 3)  # type: ignore[index]

    def test_named_vectors_are_copied(self) -> None:
        states = ["state"]
        actions = ["action"]
        schema = ExportSchema(
            name="approved-schema",
            state_names=states,
            action_names=actions,
            camera_shapes={"observation.images.confirmed": (4, 5, 3)},
        )
        states.append("unexpected")
        actions.append("unexpected")
        self.assertEqual(schema.state_names, ("state",))
        self.assertEqual(schema.action_names, ("action",))


if __name__ == "__main__":
    unittest.main()
