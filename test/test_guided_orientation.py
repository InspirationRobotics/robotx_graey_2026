import ast
import math
import unittest
from pathlib import Path
from unittest.mock import Mock
from robotx_graey_2026.api.navigation.guided_orientation import GuidedOrientation
from robotx_graey_2026.api.navigation.guided_surface_mission import GuidedSurfaceRun
from robotx_graey_2026.api.navigation.geographic_frame import GeographicFrame
from robotx_graey_2026.api.navigation.waypoint_logic import ne_to_geo


class OrientationTests(unittest.TestCase):
    def test_bearing_hold_and_wrapping(self):
        x=GuidedOrientation((1.,2.,-.1),(1.,8.))
        self.assertEqual(x.target()[:3],(1.,2.,-.1))
        self.assertAlmostEqual(x.heading,math.pi/2)
        self.assertFalse(x.ready(math.radians(85)))
        self.assertTrue(x.ready(math.radians(88)))
        x=GuidedOrientation((0.,0.,0.),(-1.,0.))
        self.assertTrue(x.ready(math.radians(-179)))

    def test_orient_once_then_position_only_despite_new_yaw_error(self):
        f=GeographicFrame();f.align(33.,-117.,(0.,0.),0.)
        lat,lon=ne_to_geo(0.,6.,(33.,-117.))
        r=GuidedSurfaceRun(dict(lat=lat,lon=lon,mode='surface',map='geographic'),'test',0.)
        o=dict(fresh=True,mode=4,armed=True,local=(0.,0.,-.1),yaw=0.,speed=0.,
               bridge=dict(enabled=True,configuration_verified=True,map_transform=f.snapshot(),estimator_health=dict(usable=True)))
        for t in (.1,.2,.3):r.step(t,o)
        self.assertEqual(r.stage,'ORIENT')
        a=r.step(.4,o);self.assertEqual(a[0][0],'guided_orient')
        self.assertEqual(a[0][1][:3],o['local'])
        o['yaw']=math.radians(89.)
        a=r.step(.6,o);self.assertEqual(r.stage,'TRANSIT')
        self.assertEqual(len(a[0][1]),3)
        o['yaw']=math.pi
        a=r.step(.8,o)
        self.assertEqual(r.stage,'TRANSIT')
        self.assertEqual(a[0][0],'guided_target');self.assertEqual(len(a[0][1]),3)
        self.assertAlmostEqual(a[0][1][1],6.)
        o['mode']=19;r.step(.9,o)
        self.assertEqual(r.stage,'ABORTED')

    def test_transport_yaw_only_during_orientation_no_speed_command(self):
        p=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/waypoint_mission.py'
        tree=ast.parse(p.read_text());tree.body=[n for n in tree.body if isinstance(n,ast.ClassDef)]
        ns=dict(Node=object);exec(compile(tree,str(p),'exec'),ns)
        node=ns['WaypointMission'].__new__(ns['WaypointMission']);node.link=Mock()
        node.send_guided_orientation((0.,0.,-.1,1.))
        self.assertEqual(node.link.send.call_args.args[5],2552)
        self.assertEqual(node.link.send.call_args.args[-2:],(1.,0.))
        node.send_guided_waypoint((0.,6.,-.1))
        self.assertEqual(node.link.send.call_args.args[5],3576)
        node.link.command.assert_not_called()
