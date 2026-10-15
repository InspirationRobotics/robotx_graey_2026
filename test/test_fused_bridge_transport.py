"""Production adapter callbacks with mocked transport; no vehicle access."""
import ast
import json
import math
import statistics
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from robotx_graey_2026.api.navigation.bridge_estimator import BridgeEstimator, yaw
from robotx_graey_2026.api.navigation.odometry_profile import PROFILE


class TransportTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/fused_nav_bridge.py'
        tree = ast.parse(path.read_text())
        tree.body = [n for n in tree.body if isinstance(n, ast.ClassDef)]
        self.clock = NS(now=10.)
        ns = dict(SteadyNode=object, math=math, json=json, String=NS, PROFILE=PROFILE,
                  yaw=yaw, time=NS(monotonic=lambda: self.clock.now),
                  statistics=statistics, BridgeEstimator=BridgeEstimator, require_maintenance=Mock())
        exec(compile(tree, str(path), 'exec'), ns)
        self.b = ns['FusedNavBridge'].__new__(ns['FusedNavBridge'])
        self.b.est = BridgeEstimator()
        self.b.stamps, self.b.params = {}, {}
        self.b.boot_ms = self.b.last_sent = None
        self.b.start_checked = True
        self.b.output_permitted = True
        self.b.reset_counter = 0
        self.b.pressures = deque(maxlen=80)
        self.b.vn_calibration = (10.,dict(phase='idle'))
        self.b.settings = dict(enabled=True, surface_pressure_hpa=1000., water_density=1000.)
        self.b.link, self.b.pub = Mock(), Mock()
        self.b.instance = 'test'
        self.b.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(self.clock.now*1e9)))
        self.b.est.gps.clock.offset = 0.
        self.b.est.gps.clock.uncertainty = 0.
        self.b.est.gps.clock.updated = 10.

    def pressure(self, stamp, system=1):
        m = NS(time_boot_ms=stamp, press_abs=1000.49,
               get_srcSystem=lambda: system, get_srcComponent=lambda: 1)
        self.b.consume('SCALED_PRESSURE2', m)

    def test_pressure_requires_clock_freshness_calibration_and_cube_sender(self):
        self.pressure(10000, system=42)
        self.assertIsNone(self.b.est.depth_at)
        self.pressure(10000)
        self.assertAlmostEqual(self.b.est.depth, .005, places=4)
        self.clock.now = 11.
        self.pressure(10100)
        self.assertIsNone(self.b.est.depth_at)
        self.b.settings['surface_pressure_hpa'] = 0.
        self.pressure(11000)
        self.assertIsNone(self.b.est.depth_at)

    def test_old_queued_dvl_is_not_freshened(self):
        self.b.dvl(NS(data=json.dumps(dict(stamp_ns=9000000000, valid=True,
                                          velocity=[0, 0, 0], sensor_time=1))))
        self.assertFalse(self.b.est.motion_valid)

    def test_nonfinite_pressure_blocks_output_without_crashing_status(self):
        for pressure in (math.nan, math.inf):
            self.clock.now += .1
            stamp = round(self.clock.now*1000)
            m = NS(time_boot_ms=stamp, press_abs=pressure,
                   get_srcSystem=lambda: 1, get_srcComponent=lambda: 1)
            self.b.consume('SCALED_PRESSURE2', m)
            self.b.tick()
            status = json.loads(self.b.pub.publish.call_args.args[0].data)
            self.assertIsNone(status['pressure_sample']['hpa'])
            self.assertIsNone(status['depth_m'])
            self.assertFalse(status['healthy'])
            self.b.link.odometry.assert_not_called()

    def test_profile_readback_expires_and_inhibits_output(self):
        self.b.params = {k: (10., v) for k, v in PROFILE.items()}
        self.assertTrue(self.b.configured(10.))
        self.assertFalse(self.b.configured(13.))
        self.b.tick()
        self.b.link.odometry.assert_not_called()
        self.b.link.command.assert_not_called()
        self.b.link.send.assert_not_called()

    def test_cube_restart_latches_fault(self):
        self.pressure(10000)
        self.pressure(100)
        self.assertIn('restarted', self.b.est.fault)

    def test_armed_start_latches_fault_and_missing_heartbeat_blocks_output(self):
        self.b.start_checked = False
        self.b.params = {k: (10., v) for k, v in PROFILE.items()}
        self.b.est.status = lambda now: dict(healthy=not self.b.est.fault)
        self.b.tick()
        self.b.link.odometry.assert_not_called()
        self.b.consume('HEARTBEAT', NS(base_mode=128, get_srcSystem=lambda: 1, get_srcComponent=lambda: 1))
        self.assertIn('started while armed', self.b.est.fault)
        self.b.tick()
        self.b.link.odometry.assert_not_called()
        self.b.consume('HEARTBEAT', NS(base_mode=0, get_srcSystem=lambda: 1, get_srcComponent=lambda: 1))
        self.assertTrue(self.b.est.fault)  # Must restart; no automatic recovery.

    def test_prepare_requires_confirmed_surface_and_restarts_alignment_with_reset_counter(self):
        self.b.params = {k:(10.,v) for k,v in PROFILE.items()}
        self.b.pressures.extend((8.+i*.1,1000.+i*.001) for i in range(21))
        self.b.est.set_origin((33.,-117.,0.))
        self.b.est.initialized = True
        self.b.est.position = [20.,10.,.5]
        old = self.b.est
        response = self.b.prepare(NS(data=False),NS())
        self.assertFalse(response.success)
        self.assertIs(self.b.est,old)
        response = self.b.prepare(NS(data=True),NS())
        self.assertTrue(response.success,response.message)
        self.assertEqual(self.b.est.origin,(33.,-117.,0.))
        self.assertFalse(self.b.est.initialized)
        self.assertIsNone(self.b.est.north_alignment.offset)
        self.assertEqual(self.b.reset_counter,1)
        self.assertTrue(self.b.output_permitted)
        self.b.tick()
        self.b.link.odometry.assert_not_called()
        self.b.link.command.assert_not_called()

    def test_prepare_rejects_unstable_pressure_or_failed_calibration(self):
        self.b.params = {k:(10.,v) for k,v in PROFILE.items()}
        for phase,spread in (('failed',0.),('collecting',0.),('idle',20.)):
            self.b.vn_calibration = (10.,dict(phase=phase))
            self.b.pressures = deque((8.+i*.1,1000.+spread*i/20) for i in range(21))
            response = self.b.prepare(NS(data=True),NS())
            self.assertFalse(response.success)
            self.assertEqual(self.b.reset_counter,0)

    def test_profile_never_selects_native_gps(self):
        for slot in (1, 2, 3):
            for axis in ('POSXY', 'VELXY', 'POSZ', 'YAW'):
                self.assertEqual(PROFILE[f'EK3_SRC{slot}_{axis}'], 6)

    def test_fast_receive_polling_does_not_flood_odometry(self):
        self.b.params = {k: (10., v) for k, v in PROFILE.items()}
        self.b.est.status = lambda now: dict(healthy=True)
        self.b.tick()
        self.clock.now = 10.01
        self.b.tick()
        self.assertEqual(self.b.link.odometry.call_count, 1)
        self.clock.now = 10.06
        self.b.tick()
        self.assertEqual(self.b.link.odometry.call_count, 2)
