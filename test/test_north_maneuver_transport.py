"""Production maneuver adapter ticks with transport mocked; no ROS/vehicle."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from robotx_graey_2026.api.navigation.north_maneuver import NorthManeuver


class TransportTests(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/north_maneuver_node.py'
        tree=ast.parse(path.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef));cls.bases=[]
        cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick']
        tree.body=[cls]
        ns=dict(json=json,math=math,time=NS(monotonic=lambda:10.),String=lambda **k:NS(**k),Bool=lambda **k:NS(**k))
        exec(compile(tree,str(path),'exec'),ns)
        self.node=ns['NorthManeuverNode']()
        self.node.logic=NorthManeuver(9.,5.);self.node.logic.last=9.95
        self.node.active=True;self.node.cancelled=False;self.node.stopped_at=None
        self.node.bridge_at=self.node.control_at=9.9
        self.node.pub=Mock();self.node.claim=Mock();self.node.link=Mock()
        self.node.bridge=dict(published_monotonic=9.9,depth_m=0.,inputs=dict(vn=True,dvl=True),
            heading_vn=dict(received=9.9,raw_yaw=40.,yaw=200.,rate=0.),
            heading_dvl=dict(velocity=[.1,0.,0.]),gps_admission=dict(qualified=True),gps_observation=dict(speed_mps=.6))
        self.node.control=dict(vehicle_fresh=True,armed=True,mode='MANUAL',sa_active=False,esc_commanded_on=True)

    def test_commands_relative_yaw_without_arming_or_mode_changes(self):
        self.node.tick()
        args=self.node.link.send.call_args.args
        self.assertEqual(args[0],'manual_control_send');self.assertGreater(args[2],0);self.assertEqual(args[5],0)
        self.node.link.command.assert_not_called();self.node.link.set_mode.assert_not_called()

    def test_mode_takeover_yields_instead_of_overwriting_pilot(self):
        self.node.control['mode']='STABILIZE';self.node.tick()
        self.node.link.send.assert_not_called()
        self.assertEqual(self.node.logic.stage,'ABORTED')

    def test_missing_inputs_abort_with_valid_json_rather_than_crash(self):
        self.node.bridge_at=self.node.control_at=-math.inf
        self.node.bridge=dict(heading_vn=dict(received=None))
        self.node.tick()
        text=self.node.pub.publish.call_args.args[0].data
        self.assertNotIn('Infinity',text);self.assertNotIn('NaN',text)
        self.assertEqual(json.loads(text)['stage'],'ABORTED')
