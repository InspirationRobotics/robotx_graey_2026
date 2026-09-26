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


if __name__ == '__main__': unittest.main()
