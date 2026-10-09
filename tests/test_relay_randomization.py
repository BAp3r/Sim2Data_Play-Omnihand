import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from relay_randomization import initial_pose


class RandomizationTests(unittest.TestCase):
    def test_reproducible_bounded_distinct_initial_poses(self):
        center=[-.23,.13,.0145]
        samples=[initial_pose(seed,center) for seed in range(20)]
        self.assertEqual(samples,[initial_pose(seed,center) for seed in range(20)])
        self.assertEqual(len({tuple(s['box_center_m']) for s in samples}),20)
        for sample in samples:
            self.assertLessEqual(max(map(abs,sample['offset_xy_m'])),.005)
            self.assertLessEqual(abs(sample['yaw_rad']),sample['yaw_half_range_rad'])
            self.assertEqual(sample['box_center_m'][2],center[2])
            self.assertAlmostEqual(sum(x*x for x in sample['quaternion_wxyz']),1.)
        self.assertEqual(center,[-.23,.13,.0145])

    def test_rejects_invalid_seed_and_position(self):
        for seed in (True,1.2,'42'):
            with self.assertRaises(ValueError): initial_pose(seed,[0,0,0])
        with self.assertRaises(ValueError): initial_pose(1,[float('nan'),0,0])
