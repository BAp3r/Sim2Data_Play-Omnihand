import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from relay_motion import sample_waypoints


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
