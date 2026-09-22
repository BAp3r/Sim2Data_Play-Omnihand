import copy
import json
from pathlib import Path
import unittest
import subprocess
import sys
import tempfile

from sim2data.preflight import assess_scene


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.spec = json.loads((Path(__file__).resolve().parents[1] / "configs/scene_spec.draft.json").read_text(encoding="utf-8"))

    def test_actual_draft_blocks_collection_and_reports_external_bin(self):
        result = assess_scene(self.spec)
        self.assertFalse(result["production_collection_allowed"])
        self.assertIn("task.bin_world_pose", result["unresolved_parameters"])
        self.assertIn("robots.right.command_channel_names", result["unresolved_parameters"])

    def test_unit_change_is_reported_as_contract_conflict(self):
        self.spec['world_frame']['length_unit'] = 'mm'
        self.assertIn('world_frame.length_unit', assess_scene(self.spec).get('invalid_contracts', []))

    def test_draft_is_incomplete_but_matches_fixed_contracts(self):
        self.assertEqual(assess_scene(self.spec)['invalid_contracts'], [])

    def test_unknown_relay_thresholds_are_required(self):
        self.assertIn('task.relay_gate.minimum_left_arm_clearance_m', assess_scene(self.spec)['unresolved_parameters'])

    def test_depth_cannot_be_enabled_without_schema_extension(self):
        self.spec['dataset']['depth_enabled'] = True
        self.assertIn('dataset.depth_enabled', assess_scene(self.spec)['invalid_contracts'])

    def test_wrist_camera_must_not_attach_to_a_finger(self):
        self.spec['camera_candidates']['wrist_left']['parent_frame'] = 'left_index_finger'
        self.assertIn('camera_candidates.wrist_left.parent_frame', assess_scene(self.spec)['invalid_contracts'])

    def test_noninteger_physics_decimation_is_a_conflict(self):
        self.spec['timing_initial_test_proposal']['physics_hz'] = 250
        self.assertIn('timing_initial_test_proposal', assess_scene(self.spec)['invalid_contracts'])

    def test_right_regrasp_cannot_skip_release_and_stability_stage(self):
        self.spec['task']['stages'] = ['left_pick_from_left_table', 'right_regrasp_from_relay_zone']
        self.assertIn('task.stages', assess_scene(self.spec).get('invalid_contracts', []))

    def test_arm_and_camera_chains_require_the_physical_mount(self):
        missing = assess_scene(self.spec)['unresolved_parameters']
        for key in ('robots.left.hand_asset', 'robots.left.T_flange_mount',
                    'robots.left.T_mount_hand_root',
                    'camera_candidates.wrist_left.T_mount_camera_housing',
                    'camera_candidates.wrist_left.T_housing_color_optical'):
            self.assertIn(key, missing)

    def test_enabling_flag_does_not_bypass_evidence(self):
        self.spec["production_collection_enabled"] = True
        result = assess_scene(self.spec)
        self.assertFalse(result["production_collection_allowed"])
        self.assertIn("PRODUCTION_FLAG_MUST_REMAIN_FALSE_AT_M0", result["blockers"])

    def test_removed_sections_are_reported_instead_of_ignored(self):
        del self.spec["robots"]
        self.assertIn("robots.left.arm_asset", assess_scene(self.spec)["unresolved_parameters"])

    def test_null_task_is_a_blocker(self):
        self.spec["task"] = None
        self.assertIn("TASK_IS_NOT_CONFIRMED_TABLE_RELAY", assess_scene(self.spec)["blockers"])

    def test_partly_filled_transform_is_still_unresolved(self):
        self.spec["robots"]["left"]["T_world_base"] = {"translation": [0, 0, 0], "quaternion_wxyz": None}
        self.assertIn("robots.left.T_world_base", assess_scene(self.spec)["unresolved_parameters"])

    def test_optional_depth_extrinsics_not_required_for_rgb(self):
        result = assess_scene(copy.deepcopy(self.spec))
        self.assertNotIn("camera_candidates.wrist_left.T_parent_depth_optical", result["unresolved_parameters"])


class PreflightCliTests(unittest.TestCase):
    def test_draft_is_read_only_and_existing_report_is_preserved(self):
        root = Path(__file__).resolve().parents[1]
        scene = root / 'configs/scene_spec.draft.json'
        original = scene.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / 'report.json'
            command = [sys.executable, '-m', 'sim2data.preflight', '--scene', str(scene), '--out', str(out)]
            first = subprocess.run(command, cwd=root, capture_output=True, text=True)
            self.assertEqual(first.returncode, 2, first.stderr)
            report = json.loads(out.read_text(encoding='utf-8'))
            self.assertFalse(report['production_collection_allowed'])
            self.assertEqual(report['invalid_contracts'], [])
            marker = b'preserve previous report\n'
            out.write_bytes(marker)
            second = subprocess.run(command, cwd=root, capture_output=True, text=True)
            self.assertNotEqual(second.returncode, 0)
            self.assertEqual(out.read_bytes(), marker)
        self.assertEqual(scene.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
