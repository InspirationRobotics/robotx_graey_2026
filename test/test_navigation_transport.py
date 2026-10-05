"""Execute production callbacks with mocked ROS/transport, without ROS installed.

AST loading excludes imports and leaves callback bodies unchanged. These tests
exercise timestamp/validity handling, not DDS or flight-controller integration.
"""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import uuid
from enum import Enum
from unittest.mock import Mock
from robotx_graey_2026.api.navigation.gps_quality import GPSQuality


ROOT = Path(__file__).parents[1] / 'robotx_graey_2026/api/navigation'


def load_callbacks(filename, classname):
    tree = ast.parse((ROOT / filename).read_text(encoding='utf-8'))
    tree.body = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))]
    clock = NS(now=10.)
    namespace = dict(Node=object, String=NS, Bool=NS, Enum=Enum, json=json, math=math, uuid=uuid,
                     MODE_MANUAL=19,
                     time=NS(monotonic=lambda: clock.now))
    exec(compile(tree, filename, 'exec'), namespace)
    return namespace[classname].__new__(namespace[classname]), clock


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.b, self.clock = load_callbacks('nav_ekf_bridge.py', 'NavEKFBridge')
        self.b.have_att = self.b.valid = True
        self.b.imu_received = self.b.dvl_received = 10.
        self.b.imu_stamp = -1
        self.b.sensor_stamp = self.b.last_t = None
        self.b.q = (1., 0., 0., 0.)
        self.b.vb = self.b.rates = (0., 0., 0.)
        self.b.pos = [0., 0., 0.]
        self.b.yaw_off = self.b.sent = self.b.reset_counter = 0
        self.b.aligned_id = ''
        self.b.link = Mock()
        self.b.publish_status = Mock()
        self.b.get_parameter = lambda _: NS(value=True)

    def sample(self, stamp, valid=True, velocity=(1, 0, 0), sensor=None):
        self.b.on_sample(NS(data=json.dumps(dict(stamp_ns=stamp, valid=valid,
            velocity=velocity, sensor_time=sensor))))

    def test_stale_or_invalid_dvl_withholds_odometry(self):
        self.b.send_odom()
        self.assertEqual(self.b.link.odometry.call_count, 1)
        self.clock.now += .6
        self.b.send_odom()
        self.sample(1000000000, valid=False)
        self.b.send_odom()
        self.assertEqual(self.b.link.odometry.call_count, 1)

    def test_atomic_invalid_sample_never_integrates_zero_or_velocity(self):
        self.sample(1000000000)
        self.sample(1100000000, valid=False, velocity=(100, 0, 0))
        self.assertEqual(self.b.pos, [0, 0, 0])
        self.sample(1200000000)
        self.assertAlmostEqual(self.b.pos[0], .1)

    def test_replayed_dvl_sensor_time_does_not_refresh_age(self):
        self.sample(1000000000, sensor=123)
        self.clock.now += .6
        self.sample(2000000000, sensor=123)
        self.assertFalse(self.b.fresh())

    def test_nonfinite_rates_reject_imu(self):
        self.b.on_imu(NS(header=NS(stamp=NS(sec=1, nanosec=0)),
            orientation=NS(w=1., x=0., y=0., z=0.),
            angular_velocity=NS(x=math.nan, y=0., z=0.)))
        self.assertFalse(self.b.have_att)

    def test_alignment_is_once_and_expired_requests_ignored(self):
        msg = NS(data=json.dumps(dict(request_id='one', cube_ned=[4, 5, 6], expires_at=10.4)))
        self.b.align(msg); self.b.align(msg)
        self.assertEqual(self.b.pos, [4, 5, 6])
        self.assertEqual(self.b.reset_counter, 1)
        self.clock.now += 1
        self.b.align(NS(data=json.dumps(dict(request_id='two', cube_ned=[0, 0, 0], expires_at=10.4))))
        self.assertEqual(self.b.reset_counter, 1)


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.s, self.clock = load_callbacks('navigation_supervisor.py', 'NavigationSupervisor')
        self.s.samples, self.s.stamps, self.s.params = {}, {}, {}
        self.s.settings = dict(pressure_message='SCALED_PRESSURE2', surface_pressure_hpa=0., water_density=1025.)
        self.s.boot_ms = self.s.cube_origin = self.s.last_gps = self.s.last_gps_time = None
        self.s.epoch = 'original'
        self.s.logic = NS(fault='')
        self.s.gps_quality = GPSQuality()
        self.s.gps_quality.motion(self.clock.now, (0, 0, 0), True)

    def receive(self, kind, **fields):
        self.s.consume(kind, NS(get_srcSystem=lambda: 1, get_srcComponent=lambda: 1, **fields))

    def gps(self, stamp=100, fix=3, accuracy=1000, lat=330000000):
        self.receive('GPS_RAW_INT', time_usec=stamp, fix_type=fix, lat=lat,
                     lon=-1170000000, alt=100000, h_acc=accuracy)

    def test_cached_gps_expires_and_missing_accuracy_rejected(self):
        self.gps(); self.assertTrue(self.s.fresh('gps_good'))
        self.clock.now += 2; self.gps()
        self.assertIsNone(self.s.fresh('gps_good'))
        self.gps(stamp=101, accuracy=0)
        self.assertFalse(self.s.fresh('gps_good'))

    def test_gps_jump_and_fix_loss_rejected(self):
        self.gps(); self.clock.now += .2
        self.gps(stamp=101, lat=340000000)
        self.assertFalse(self.s.fresh('gps_good'))
        self.gps(stamp=101, fix=1)
        self.assertFalse(self.s.fresh('gps_good'))

    def test_depth_requires_explicit_surface_calibration(self):
        self.receive('SCALED_PRESSURE2', time_boot_ms=100, press_abs=1013.)
        self.assertTrue(math.isnan(self.s.fresh('depth')))
        self.s.settings['surface_pressure_hpa'] = 1013.
        self.receive('SCALED_PRESSURE2', time_boot_ms=101, press_abs=1113.)
        self.assertAlmostEqual(self.s.fresh('depth'), .99484, places=4)

    def test_atomic_dvl_replay_does_not_qualify_gps(self):
        self.s.dvl_sample(NS(data=json.dumps(dict(stamp_ns=100, sensor_time=12,
                                                velocity=[0, 0, 0], valid=True))))
        self.clock.now += 1
        self.s.dvl_sample(NS(data=json.dumps(dict(stamp_ns=101, sensor_time=12,
                                                velocity=[0, 0, 0], valid=True))))
        self.gps(stamp=101)
        self.assertFalse(self.s.fresh('gps_good'))
        self.assertIn('DVL', self.s.fresh('gps_reason'))

    def test_atomic_invalid_dvl_cannot_claim_stationarity(self):
        self.s.dvl_sample(NS(data=json.dumps(dict(stamp_ns=100,
                                                velocity=[0, 0, 0], valid=False))))
        self.gps()
        self.assertFalse(self.s.fresh('gps_good'))

    def test_simulated_depth_topic_isolated_from_cube_pressure(self):
        self.s.settings['depth_topic'] = '/graey/sitl/depth_m'
        self.s.sim_depth(NS(data=.5))
        self.receive('SCALED_PRESSURE2', time_boot_ms=101, press_abs=1200.)
        self.assertEqual(self.s.fresh('depth'), .5)

    def test_cube_reboot_and_origin_change_invalidate_frame(self):
        self.receive('LOCAL_POSITION_NED', time_boot_ms=10000, x=0., y=0., z=0.)
        self.receive('LOCAL_POSITION_NED', time_boot_ms=100, x=0., y=0., z=0.)
        self.assertNotEqual(self.s.epoch, 'original')
        self.assertIn('restarted', self.s.logic.fault)
        self.receive('GPS_GLOBAL_ORIGIN', latitude=1, longitude=2, altitude=3)
        self.receive('GPS_GLOBAL_ORIGIN', latitude=2, longitude=2, altitude=3)
        self.assertIn('origin changed', self.s.logic.fault)

    def test_uninitialized_or_constant_position_ekf_is_not_healthy(self):
        for flags in (1 | 2 | 8 | 32 | 32768, 1 | 2 | 8 | 32 | 128):
            self.receive('EKF_STATUS_REPORT', flags=flags, velocity_variance=.1,
                         pos_horiz_variance=.1, pos_vert_variance=.1)
            self.assertFalse(self.s.fresh('ekf'))


class MissionGateTests(unittest.TestCase):
    def setUp(self):
        self.m, self.clock = load_callbacks('mission_base.py', 'MissionBase')
        self.states = self.m.navigation_gate.__globals__['S']
        self.m.state = self.states.WAIT_NAV
        self.m.nav_run = 'run'
        self.m.nav_sequence = 0
        self.m.nav_supervised = True
        self.m.nav_interrupted = False
        self.m.nav_abort_last = -math.inf
        self.m.nav_status_time = 10.
        self.m.nav_status = dict(run_id='run', navigation_ready=True, reference_current=True)
        self.m.nav_intent_pub = Mock()
        self.m.auto_pub = Mock()
        self.m.link = Mock()
        self.m.get_logger = Mock(return_value=Mock())
        self.m.mode = 'GUIDED'
        self.m.dry = False

    def test_no_start_with_stale_or_wrong_run_permission(self):
        self.assertTrue(self.m.navigation_gate())
        self.m.nav_status['run_id'] = 'old'
        self.assertFalse(self.m.navigation_gate())
        self.m.nav_status['run_id'] = 'run'
        self.clock.now += 2
        self.assertFalse(self.m.navigation_gate())
        self.m.link.set_mode.assert_not_called()

    def test_loss_during_mission_requests_takeover_and_never_auto_resumes(self):
        self.m.state = self.states.TO_GATE
        self.m.nav_status['navigation_ready'] = False
        self.assertFalse(self.m.navigation_gate())
        self.m.link.set_mode.assert_called_once_with(19)
        self.m.nav_status['navigation_ready'] = True
        self.assertFalse(self.m.navigation_gate())
        self.m.link.arm.assert_not_called()
        self.m.link.disarm.assert_not_called()

    def test_dry_run_loss_does_not_send_mode_commands(self):
        self.m.state = self.states.TO_GATE
        self.m.dry = True
        self.m.nav_status['navigation_ready'] = False
        self.assertFalse(self.m.navigation_gate())
        self.m.link.set_mode.assert_not_called()


if __name__ == '__main__':
    unittest.main()
