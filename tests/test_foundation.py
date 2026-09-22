from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

from sim2data.core import (ChannelLayout, FrameClock, Timing, component_seed,
                          resolve_local_asset, validate_episode_clocks)
from scripts.collect_inventory import scan_root, write_report


class TimingTests(unittest.TestCase):
    def test_integer_decimation(self):
        timing = Timing(240, 30)
        self.assertEqual(timing.ticks_per_frame, 8)
        self.assertEqual(timing.timestamp(30), 1.0)

    def test_invalid_rates(self):
        for rates in [(0, 30), (240, 0), (250, 30), (240.0, 30), (True, 1)]:
            with self.subTest(rates=rates), self.assertRaises(ValueError):
                Timing(*rates)

    def test_negative_frame_index(self):
        with self.assertRaises(ValueError):
            Timing().timestamp(-1)


class SeedTests(unittest.TestCase):
    def test_order_independent(self):
        forward = {i: component_seed(42, i, 'geometry') for i in range(20)}
        backward = {i: component_seed(42, i, 'geometry') for i in reversed(range(20))}
        self.assertEqual(forward, backward)

    def test_component_separation(self):
        self.assertNotEqual(component_seed(42, 2, 'geometry'), component_seed(42, 2, 'lighting'))

    def test_invalid_seed(self):
        for args in [(-1, 0, 'a'), (0, -1, 'a'), (True, 0, 'a'), (0, 0, '')]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                component_seed(*args)


class AssetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root / 'card_box.usda').write_text('#usda 1.0\n', encoding='utf-8')

    def tearDown(self):
        self.temp.cleanup()

    def test_resolves_without_copy(self):
        self.assertEqual(resolve_local_asset(self.root, 'card_box.usda'), self.root / 'card_box.usda')

    def test_refuses_path_escape(self):
        for path in ('../x.usd', '/tmp/x.usd', 'C:/x.usd', r'dir\x.usd', 'https://host/x.usd'):
            with self.subTest(path=path), self.assertRaises(ValueError):
                resolve_local_asset(self.root, path)

    def test_refuses_unresolved_lfs(self):
        (self.root / 'pointer.usd').write_text('version https://git-lfs.github.com/spec/v1\noid sha256:123\nsize 1\n')
        with self.assertRaisesRegex(ValueError, 'LFS pointer'):
            resolve_local_asset(self.root, 'pointer.usd')

    def test_missing_asset(self):
        with self.assertRaises(FileNotFoundError):
            resolve_local_asset(self.root, 'missing.usd')

    def test_refuses_directory(self):
        with self.assertRaises(ValueError):
            resolve_local_asset(self.root, '.')

    def test_symlink_escape(self):
        with tempfile.TemporaryDirectory() as outside:
            target = Path(outside) / 'file.usd'
            target.write_text('data')
            try:
                (self.root / 'link.usd').symlink_to(target)
            except OSError:
                self.skipTest('symlinks unavailable')
            with self.assertRaises(ValueError):
                resolve_local_asset(self.root, 'link.usd')

    def test_root_symlink_is_allowed(self):
        with tempfile.TemporaryDirectory() as parent:
            link = Path(parent) / 'official'
            try:
                link.symlink_to(self.root, target_is_directory=True)
            except OSError:
                self.skipTest('symlinks unavailable')
            self.assertEqual(resolve_local_asset(link, 'card_box.usda'), self.root / 'card_box.usda')


class LayoutTests(unittest.TestCase):
    def test_single_string_is_not_a_named_channel_vector(self):
        with self.assertRaises(ValueError):
            ChannelLayout('xyz', ('command',))

    def test_channel_layout_cannot_change_through_callers_list(self):
        names = ['q_left', 'q_right']
        layout = ChannelLayout(names, ['command'])
        names.append('unexpected')
        self.assertEqual(layout.state_names, ('q_left', 'q_right'))
        layout.validate([0.0, 0.0], [0.0])

    def test_separate_state_and_action_dimensions(self):
        layout = ChannelLayout(('measured_motor', 'passive_joint'), ('commanded_motor',))
        layout.validate([0.0, 1.0], [0.5])

    def test_no_duplicate_names(self):
        with self.assertRaises(ValueError):
            ChannelLayout(('q', 'q'), ('a',))

    def test_dimension_check(self):
        with self.assertRaises(ValueError):
            ChannelLayout(('q',), ('a',)).validate([1, 2], [0])

    def test_nonfinite_check(self):
        for value in (float('nan'), float('inf'), '1', True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                ChannelLayout(('q',), ('a',)).validate([value], [0])


class ClockTests(unittest.TestCase):
    def frames(self):
        return [FrameClock(i, i * 8, i / 30, (i * 8, i * 8, i * 8)) for i in range(5)]

    def test_valid_clocks(self):
        self.assertEqual(validate_episode_clocks(iter(self.frames()), Timing(), 3), 5)

    def test_invalid_timestamp_types_are_not_numbers(self):
        for timestamp in (False, True, '0', None, float('nan'), float('inf')):
            frames = self.frames()
            frames[0] = replace(frames[0], timestamp=timestamp)
            with self.subTest(timestamp=timestamp), self.assertRaises(ValueError):
                validate_episode_clocks(frames, Timing(), 3)

    def test_one_step_camera_lag(self):
        frames = self.frames()
        frames[2] = replace(frames[2], camera_physics_steps=(8, 16, 16))
        with self.assertRaisesRegex(ValueError, 'different simulation steps'):
            validate_episode_clocks(frames, Timing(), 3)

    def test_frame_gap(self):
        frames = self.frames()
        frames.pop(2)
        with self.assertRaises(ValueError):
            validate_episode_clocks(frames, Timing(), 3)

    def test_wall_clock_timestamp_not_allowed(self):
        frames = self.frames()
        frames[2] = replace(frames[2], timestamp=1700000000.0)
        with self.assertRaises(ValueError):
            validate_episode_clocks(frames, Timing(), 3)

    def test_missing_camera(self):
        frames = self.frames()
        frames[2] = replace(frames[2], camera_physics_steps=(16, 16))
        with self.assertRaises(ValueError):
            validate_episode_clocks(frames, Timing(), 3)

    def test_empty_episode(self):
        with self.assertRaises(ValueError):
            validate_episode_clocks([], Timing(), 3)


class InventoryTests(unittest.TestCase):
    def test_missing_root_is_not_an_empty_inventory(self):
        with tempfile.TemporaryDirectory() as directory:
            report = scan_root(Path(directory) / 'absent')
        self.assertFalse(report['complete'])
        self.assertTrue(report['errors'])

    def test_candidate_in_parent_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'Card_Box').mkdir()
            (root / 'Card_Box' / 'model.usd').write_text('fake fixture, not a real USD asset')
            (root / 'ignored.txt').write_text('test')
            report = scan_root(root)
        self.assertTrue(report['complete'])
        self.assertEqual(report['matches'][0]['path'], 'Card_Box/model.usd')
        self.assertEqual(report['files_seen'], 2)

    def test_scan_limits_are_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for i in range(3):
                (root / f'card_box_{i}.usd').write_text('test')
            report = scan_root(root, max_files=1)
        self.assertFalse(report['complete'])
        self.assertEqual(report['stop_reason'], 'file_limit')
        self.assertEqual(report['files_seen'], 1)

    def test_match_truncation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for i in range(3):
                (root / f'airbot_{i}.urdf').write_text('test')
            report = scan_root(root, max_matches=1)
        self.assertEqual(report['matches_total'], 3)
        self.assertTrue(report['matches_truncated'])

    def test_video_metadata_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'scene.mp4').write_bytes(b'fixture')
            report = scan_root(root, kind='videos')
        self.assertEqual(report['matches'][0]['bytes'], 7)

    def test_report_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'report.json'
            write_report(path, {'first': True})
            with self.assertRaises(FileExistsError):
                write_report(path, {'second': True})
            self.assertEqual(json.loads(path.read_text()), {'first': True})

    def test_explicit_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'report.json'
            write_report(path, {'first': True})
            write_report(path, {'second': True}, overwrite=True)
            self.assertEqual(json.loads(path.read_text()), {'second': True})


if __name__ == '__main__':
    unittest.main()
