"""Pure GUI tests: no ROS, network, or vehicle connection needed."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location('navigation_view',
    Path(__file__).parents[1] / 'robotx_graey_2026/api/gui/navigation_view.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.now = 10
        self.view = module.NavigationView(lambda: self.now)

    def message(self, kind, **fields):
        self.view.consume(kind, SimpleNamespace(get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 1, **fields))

    def gps(self, fix=3, lat=330000000, timestamp=1):
        self.message('GPS_RAW_INT', lat=lat, lon=-1170000000, alt=100000,
            fix_type=fix, satellites_visible=16, eph=100, time_usec=timestamp)

    def test_no_fix_never_establishes_origin(self):
        self.gps(fix=1)
        self.assertIsNone(self.view.origin)
        self.assertFalse(self.view.snapshot()['streams']['gps']['data']['valid'])

    def test_origin_fixed_and_freshness_independent(self):
        self.gps()
        origin = list(self.view.origin)
        self.now += 3
        self.view.put('heartbeat', {'armed': False})
        self.assertFalse(self.view.snapshot()['streams']['gps']['fresh'])
        self.gps(lat=330001000, timestamp=2)
        self.assertEqual(origin, self.view.origin)
        ned = self.view.snapshot()['streams']['gps']['data']['ned']
        self.assertAlmostEqual(ned[0], 11.09, delta=.1)
        self.assertAlmostEqual(ned[1], 0, delta=.01)

    def test_cached_gps_does_not_refresh_measurement(self):
        self.gps()
        self.now += 4
        self.gps()
        self.assertFalse(self.view.snapshot()['streams']['gps']['fresh'])

    def test_trails_keep_geographic_coordinates_for_streets(self):
        self.gps()
        self.now += 1
        self.gps(lat=330001000, timestamp=2)
        points = self.view.snapshot()['trails']['gps']
        self.assertEqual(points[0][4:], [33, -117])
        self.assertEqual(points[1][4:], [33.0001, -117])
        self.assertEqual(points[0][3], 1)
        self.assertEqual(points[1][3], 0)

    def test_snapshot_detached_and_nonfinite_unavailable(self):
        self.view.put('local', {'ned': [float('nan'), 2, 3]})
        snap = self.view.snapshot()
        self.assertIsNone(snap['streams']['local']['data']['ned'][0])
        snap['streams']['local']['data']['ned'][1] = 99
        self.assertEqual(self.view.snapshot()['streams']['local']['data']['ned'][1], 2)

    def test_reboot_clears_old_frame(self):
        def global_at(t):
            self.message('GLOBAL_POSITION_INT', lat=330000000, lon=-1170000000,
                alt=100000, vx=0, vy=0, vz=0, hdg=65535, time_boot_ms=t)
        global_at(50000)
        self.view.put('local', {'ned': [1, 2, 3]})
        global_at(100)
        self.assertNotIn('local', self.view.snapshot()['streams'])
        self.assertEqual(self.view.session, 1)

    def test_reset_waits_for_new_gps_and_clears_both_tracks(self):
        self.gps(timestamp=100)
        self.view._geo('global', 33, -117, 100, {})
        self.view.put('local', {'ned': [12, 3, 2]})
        self.view.reset_display()
        snap = self.view.snapshot()
        self.assertIsNone(snap['origin'])
        self.assertEqual(snap['trails'], {'gps': [], 'global': []})
        self.assertIsNone(snap['streams']['global']['data']['ned'])
        self.view._geo('global', 34, -117, 100, {})
        self.gps(timestamp=100)
        self.gps(fix=1, timestamp=101)
        self.gps(timestamp=99)
        self.assertIsNone(self.view.origin)
        self.gps(lat=330001000, timestamp=102)
        snap = self.view.snapshot()
        self.assertEqual(snap['display_reset']['state'], 'complete')
        self.assertEqual(snap['origin'], [33.0001, -117, 100])
        self.assertEqual(snap['streams']['gps']['data']['ned'], [0, 0, 0])
        self.assertEqual(snap['streams']['local']['data']['ned'], [12, 3, 2])
        self.assertEqual(len(snap['trails']['gps']), 1)
        self.assertEqual(snap['trails']['global'], [])
        self.assertEqual(snap['origin_revision'], 1)

    def test_reset_without_prior_gps_requires_timestamp_progress(self):
        self.view.reset_display()
        self.gps(timestamp=0)
        self.gps(timestamp=100)
        self.gps(timestamp=100)
        self.assertIsNone(self.view.origin)
        self.gps(timestamp=101)
        self.assertEqual(self.view.reset_state, 'complete')

    def test_reset_timeout_requires_explicit_retry(self):
        self.gps(timestamp=100)
        self.view.reset_display()
        self.now += 16
        self.gps(timestamp=101)
        self.assertEqual(self.view.snapshot()['display_reset']['state'], 'timeout')
        self.assertIsNone(self.view.origin)
        self.assertEqual(self.view.snapshot()['trails']['gps'], [])
        self.view.reset_display()
        self.gps(timestamp=102)
        self.assertEqual(self.view.reset_state, 'complete')

    def test_repeated_click_does_not_extend_pending_reset(self):
        self.gps(timestamp=100)
        self.view.reset_display()
        self.now += 5
        self.view.reset_display()
        self.assertEqual(self.view.snapshot()['display_reset']['remaining_s'], 10)
        self.assertEqual(self.view.session, 1)

    def test_reset_rebases_cached_global_without_refreshing_its_age(self):
        self.view._geo('global', 33, -117, 100, {})
        self.gps(timestamp=100)
        self.view.reset_display()
        self.now += 4
        self.gps(lat=330001000, timestamp=101)
        sample = self.view.snapshot()['streams']['global']
        self.assertFalse(sample['fresh'])
        self.assertEqual(sample['age'], 4)
        self.assertAlmostEqual(sample['data']['ned'][0], -11.09, delta=.1)

    def test_display_reset_preserves_mission_zero_and_marker(self):
        self.gps(timestamp=100)
        reference = dict(gps_lat=33., gps_lon=-117., gps_msl_alt=100., cube_ned=[12, 3, 2])
        self.view.put('supervisor', dict(reference=reference, reference_current=True))
        self.view.reset_display()
        self.gps(lat=330001000, timestamp=101)
        snap = self.view.snapshot()
        self.assertEqual(snap['mission_reference']['cube_ned'], [12, 3, 2])
        self.assertAlmostEqual(snap['mission_reference']['display_ned'][0], -11.09, delta=.1)
        self.assertTrue(snap['mission_reference']['live'])
        self.now += 3
        self.assertFalse(self.view.snapshot()['mission_reference']['live'])


if __name__ == '__main__': unittest.main()
