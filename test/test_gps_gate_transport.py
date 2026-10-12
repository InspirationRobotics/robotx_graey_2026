"""Execute the real gate callbacks with mocked ROS and network dependencies."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from robotx_graey_2026.api.navigation.gps_admission import GPSAdmission, gps_input_args
from robotx_graey_2026.api.navigation.continuous_profile import PROFILE


class GateTransportTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/gps_gate.py'
        tree = ast.parse(path.read_text())
        tree.body = [x for x in tree.body if isinstance(x, ast.ClassDef)]
        self.clock = NS(now=10.1)
        env = dict(SteadyNode=object, PROFILE=PROFILE, json=json, math=math, String=NS,
                   gps_input_args=gps_input_args,
                   time=NS(monotonic=lambda: self.clock.now))
        exec(compile(tree, str(path), 'exec'), env)
        n = self.n = env['GPSGate'].__new__(env['GPSGate'])
        n.enabled = True
        n.link = Mock()
        n.pub = Mock()
        n.params = {k: (10., v) for k, v in PROFILE.items()}
        n.vn = 10.
        n.gate = GPSAdmission(dwell=0.)
        n.gate.clock.offset = 5.
        n.gate.clock.updated = 10.
        n.gate.clock.uncertainty = 0.
        n.gate.quality.motion(10.1, (0, 0, 0), True)
        n.last_forwarded = n.forwarded_fix = n.forwarded_message = None

    def msg(self, component=1):
        return NS(get_srcSystem=lambda: 1, get_srcComponent=lambda: component,
            time_usec=5100000, lat=330000000, lon=-1170000000, alt=1000,
            fix_type=3, h_acc=1000, eph=120, epv=200, vel=0, cog=65535,
            vel_acc=100, satellites_visible=15)

    def test_single_delivery_preserves_position_and_does_not_invent_epoch(self):
        m = self.msg()
        self.n.consume('GPS_RAW_INT', m)
        args = self.n.link.send.call_args.args
        self.assertEqual(args[0], 'gps_input_send')
        self.assertEqual(args[1:6], (5100000, 1, 184, 0, 0))
        self.assertEqual(args[7:9], (m.lat, m.lon))
        self.n.consume('GPS_RAW_INT', m)
        self.assertEqual(self.n.link.send.call_count, 1)

    def test_foreign_virtual_and_uncommissioned_inputs_never_forward(self):
        self.n.consume('GPS_RAW_INT', self.msg(200))
        self.n.consume('GPS2_RAW', self.msg())
        self.n.link.send.assert_not_called()
        self.n.enabled = False
        self.n.consume('GPS_RAW_INT', self.msg())
        self.n.link.send.assert_not_called()

    def test_wrong_or_stale_profile_blocks_transmission(self):
        self.n.params['GPS_PRIMARY'] = (10., 0)
        self.n.consume('GPS_RAW_INT', self.msg())
        self.n.link.send.assert_not_called()
        self.clock.now = 15.
        self.assertFalse(self.n.configured(15.))

    def test_expiry_withdraws_fix_once_and_never_refreshes_timestamp(self):
        self.n.consume('GPS_RAW_INT', self.msg())
        self.clock.now = 10.6
        self.n.tick()
        args = self.n.link.send.call_args.args
        self.assertEqual(args[1], 5100000)
        self.assertEqual(args[6], 1)  # NO_FIX
        self.n.tick()
        self.assertEqual(self.n.link.send.call_count, 2)
