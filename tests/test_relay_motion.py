import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from relay_motion import resolve_motion_timing, sample_waypoints


class WaypointTimingTests(unittest.TestCase):
    def test_does_not_cut_screened_corner_and_stops_on_both_sides(self):
        path=[[0.,0.],[1.,0.],[1.,1.]]
        self.assertEqual(sample_waypoints(path,.5,[1]),[1.,0.])
        before=sample_waypoints(path,.5-1e-4,[1]);after=sample_waypoints(path,.5+1e-4,[1])
        self.assertEqual(before[1],0.);self.assertEqual(after[0],1.)
        self.assertLess(1-before[0],2e-7);self.assertLess(after[1],2e-7)

    def test_default_preserves_global_smoothstep_and_endpoints(self):
        path=[[0.],[1.],[3.]]
        self.assertEqual(sample_waypoints(path,0),[0.]);self.assertEqual(sample_waypoints(path,1),[3.])
        self.assertEqual(sample_waypoints(path,.5),[1.])
        self.assertAlmostEqual(sample_waypoints(path,.25)[0],.3125)

    def test_invalid_stops_cannot_change_path_order(self):
        for stops in ([0],[2],[1,1],[1.0]):
            with self.assertRaises(ValueError):sample_waypoints([[0],[1],[2]],.5,stops)


class MotionTimingRecipeTests(unittest.TestCase):
    def test_defaults_and_single_variable_overrides_are_isolated(self):
        defaults = resolve_motion_timing()
        self.assertEqual(defaults, {"right_lower_s": 4.0, "right_release_s": 2.0})
        self.assertEqual(resolve_motion_timing({}), defaults)
        self.assertEqual(resolve_motion_timing({"right_lower_s": 6}),
                         {"right_lower_s": 6.0, "right_release_s": 2.0})
        self.assertEqual(resolve_motion_timing({"right_release_s": 4.0}),
                         {"right_lower_s": 4.0, "right_release_s": 4.0})
        changed = resolve_motion_timing({"right_lower_s": 6.0})
        changed["right_release_s"] = 9.0
        self.assertEqual(resolve_motion_timing(), defaults)

    def test_rejects_unknown_malformed_bool_nonfinite_and_out_of_range_values(self):
        invalid = (
            None,
            [],
            {"right_transfer_s": 6.0},
            {"right_lower_s": True},
            {"right_release_s": False},
            {"right_lower_s": float("nan")},
            {"right_release_s": float("inf")},
            {"right_lower_s": float("-inf")},
            {"right_lower_s": 0.999},
            {"right_release_s": 10.001},
            {"right_lower_s": "6"},
        )
        for config in invalid:
            with self.subTest(config=config), self.assertRaises(ValueError):
                resolve_motion_timing(config)

    def test_accepts_inclusive_duration_bounds(self):
        self.assertEqual(resolve_motion_timing({"right_lower_s": 1, "right_release_s": 10}),
                         {"right_lower_s": 1.0, "right_release_s": 10.0})

    @unittest.skipUnless(importlib.util.find_spec("numpy"),
                         "run-level relay preflight test requires numpy")
    def test_invalid_recipe_is_rejected_before_kit_start(self):
        import isaac_relay

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            profile, manifest, spec = root / "profile.json", root / "manifest.json", root / "spec.json"
            profile.write_text("{}", encoding="utf-8")
            manifest.write_text("{}", encoding="utf-8")
            spec.write_text(json.dumps({"profile": str(profile), "manifest": str(manifest),
                                        "motion_timing": {"right_lower_s": True}}), encoding="utf-8")
            args = SimpleNamespace(spec=spec, out=root / "trial")
            with patch.object(isaac_relay, "_start_kit") as start_kit:
                self.assertEqual(isaac_relay.run(args), 3)
            start_kit.assert_not_called()
            report = json.loads((args.out / "result.json").read_text(encoding="utf-8"))
            self.assertFalse(report["passed"])
            self.assertIn("right_lower_s", report["error"])
