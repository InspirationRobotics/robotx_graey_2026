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
from robotx_graey_2026.api.navigation.bridge_identity import BridgeIdentity
ROOT = Path(__file__).parents[1] / 'robotx_graey_2026/api/navigation'

def load_callbacks(filename, classname):
    tree = ast.parse((ROOT / filename).read_text(encoding='utf-8'))
    tree.body = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))]
    clock = NS(now=10.0)
    namespace = dict(Node=object, String=NS, Bool=NS, Enum=Enum, json=json, math=math, uuid=uuid, MODE_MANUAL=19, time=NS(monotonic=lambda : clock.now))
    exec(compile(tree, filename, 'exec'), namespace)
    return (namespace[classname].__new__(namespace[classname]), clock)

class TelemetryTests(unittest.TestCase):

    def setUp(self):
        (self.s, self.clock) = load_callbacks('navigation_supervisor.py', 'NavigationSupervisor')
        (self.s.samples, self.s.stamps, self.s.params) = ({}, {}, {})
        self.s.settings = dict(pressure_message='SCALED_PRESSURE2', surface_pressure_hpa=0.0, water_density=1025.0)
        self.s.boot_ms = self.s.cube_origin = self.s.last_gps = self.s.last_gps_time = None
        self.s.epoch = 'original'
        self.s.logic = NS(fault='')
        self.s.logic.pending = None
        self.s.switch_trace = None
        self.s.get_logger = Mock(return_value=Mock())
        self.s.bridge_identity = BridgeIdentity()
        self.s.gps_quality = GPSQuality()
        self.s.gps_quality.motion(self.clock.now, (0, 0, 0), True)

    def receive(self, kind, **fields):
        self.s.consume(kind, NS(get_srcSystem=lambda : 1, get_srcComponent=lambda : 1, **fields))

    def test_bridge_duplicates_latch_fault_and_malformed_status_does_not_refresh(self):
        for data in ('null', '[]', '42', '{}', '{"instance": 12}'):
            self.s.bridge_status(NS(data=data))
        self.assertNotIn('bridge', self.s.samples)
        for instance in ('first', 'second', 'first'):
            self.s.bridge_status(NS(data=json.dumps(dict(instance=instance, healthy=True))))
            self.clock.now += 0.1
        self.assertIn('Multiple', self.s.logic.fault)
        self.clock.now += 2
        self.s.bridge_status(NS(data='{"instance":"first","healthy":true}'))
        self.assertIn('Multiple', self.s.logic.fault)

    def test_cube_reboot_and_origin_change_invalidate_frame(self):
        self.receive('LOCAL_POSITION_NED', time_boot_ms=10000, x=0.0, y=0.0, z=0.0)
        self.receive('LOCAL_POSITION_NED', time_boot_ms=100, x=0.0, y=0.0, z=0.0)
        self.assertNotEqual(self.s.epoch, 'original')
        self.assertIn('restarted', self.s.logic.fault)
        self.receive('GPS_GLOBAL_ORIGIN', latitude=1, longitude=2, altitude=3)
        self.receive('GPS_GLOBAL_ORIGIN', latitude=2, longitude=2, altitude=3)
        self.assertIn('origin changed', self.s.logic.fault)

    def test_uninitialized_or_constant_position_ekf_is_not_healthy(self):
        for flags in (1 | 2 | 8 | 32 | 32768, 1 | 2 | 8 | 32 | 128):
            self.receive('EKF_STATUS_REPORT', flags=flags, velocity_variance=0.1, pos_horiz_variance=0.1, pos_vert_variance=0.1)
            self.assertFalse(self.s.fresh('ekf'))

class RuntimeRegressionTests(unittest.TestCase):

    def test_sitl_target_is_not_restarted_every_tick(self):
        (node, clock) = load_callbacks('sitl_mission.py', 'SitlMission')
        node.link = Mock()
        (node.last_target, node.last_target_time, node.target_yaw) = (None, -math.inf, None)
        node.attitude = 0.4
        node.command_target(1, 2, 0.5)
        for _ in range(20):
            clock.now += 0.1
            node.attitude += 0.01
            node.command_target(1, 2, 0.5)
        self.assertEqual(node.link.goto_ned.call_count, 1)
        clock.now += 2
        node.command_target(1, 2, 0.5)
        self.assertEqual(node.link.goto_ned.call_count, 2)
        self.assertEqual(node.link.goto_ned.call_args.args[-1], 0.4)
        node.command_target(2, 2, 0.5)
        self.assertEqual(node.link.goto_ned.call_count, 3)

    def test_sitl_health_loss_records_failure_and_never_sends_target(self):
        (node, clock) = load_callbacks('sitl_mission.py', 'SitlMission')
        node.intent_pub = Mock()
        node.publish_intent = Mock()
        node.stage = 'UNDERWATER_WAYPOINT'
        (node.stage_started, node.timeout) = (clock.now, 90)
        (node.status_received, node.run_id) = (clock.now, 'run')
        node.status = dict(run_id='run', state='Waiting for healthy navigation', reason='DVL stale')
        (node.link, node.write_result, node.command_target) = (Mock(), Mock(), Mock())
        node.get_logger = Mock(return_value=Mock())
        node.tick()
        self.assertEqual(node.stage, 'FAULT')
        node.write_result.assert_called_once_with(False)
        node.command_target.assert_not_called()
        node.link.disarm.assert_called_once()

    def test_vn_read_failure_closes_port_and_discards_partial_line(self):
        (node, _) = load_callbacks('vn100_node.py', 'VN100Node')
        port = Mock()
        port.read.side_effect = OSError('disconnected')
        (node.ser, node.buf) = (port, b'partial')
        node.get_logger = Mock(return_value=Mock())
        node.tick()
        port.close.assert_called_once()
        self.assertIsNone(node.ser)
        self.assertEqual(node.buf, b'')

    def test_sitl_stale_healthy_status_aborts_with_actual_reason(self):
        (node, clock) = load_callbacks('sitl_mission.py', 'SitlMission')
        node.intent_pub = Mock()
        node.publish_intent = Mock()
        node.stage = 'SURFACING'
        (node.stage_started, node.timeout) = (clock.now, 90)
        (node.status_received, node.run_id) = (clock.now - 1.17, 'run')
        node.status = dict(run_id='run', state='Surface GPS', reason='Navigation inputs healthy', navigation_ready=True)
        (node.link, node.write_result, node.command_target) = (Mock(), Mock(), Mock())
        node.get_logger = Mock(return_value=Mock())
        node.tick()
        self.assertEqual(node.stage, 'FAULT')
        self.assertIn('status age', node.failure_reason)
        self.assertNotIn('Navigation inputs healthy', node.failure_reason)
        node.write_result.assert_called_once_with(False)
        node.command_target.assert_not_called()
        node.link.disarm.assert_called_once()

    def test_simulator_does_not_restamp_cached_attitude(self):
        (node, clock) = load_callbacks('sitl_sensor_sim.py', 'SitlSensorSim')
        node.last_request = clock.now
        node.last_publish = 0.0
        node.link = Mock()
        node.imu_pub = Mock()
        node.latest = {'attitude': {'received': clock.now - 1, 'q': (1, 0, 0, 0)}}
        node.tick()
        node.imu_pub.publish.assert_not_called()

class MissionGateTests(unittest.TestCase):

    def setUp(self):
        (self.m, self.clock) = load_callbacks('mission_base.py', 'MissionBase')
        self.states = self.m.navigation_gate.__globals__['S']
        self.m.state = self.states.WAIT_NAV
        self.m.nav_run = 'run'
        self.m.nav_sequence = 0
        self.m.nav_supervised = True
        self.m.nav_interrupted = False
        self.m.nav_abort_last = -math.inf
        self.m.nav_status_time = 10.0
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
