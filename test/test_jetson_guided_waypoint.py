import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
from robotx_graey_2026.api.navigation.guided_surface_mission import GuidedSurfaceRun
from robotx_graey_2026.api.navigation.geographic_frame import GeographicFrame
from robotx_graey_2026.api.navigation.waypoint_logic import ne_to_geo


class JetsonGuidedWaypointTests(unittest.TestCase):
    def test_start_and_transit_use_ekf_without_gps_or_dvl_qualification(self):
        frame=GeographicFrame();frame.align(33.,-117.,(0.,0.),0.)
        lat,lon=ne_to_geo(6.,4.,(33.,-117.))
        r=GuidedSurfaceRun(dict(lat=lat,lon=lon,mode='surface',map='geographic'),'run',0.)
        b=dict(enabled=True,configuration_verified=True,map_transform=frame.snapshot(),
            estimator_health=dict(usable=True,sensors=dict(gps=False,dvl=False,vn=True)))
        o=dict(fresh=True,mode=19,armed=False,bridge=b,local=(1.,2.,-.1),yaw=1.,speed=0.,depth=.3)
        r.step(.1,o);self.assertEqual(r.stage,'GUIDED')
        self.assertEqual(r.step(.2,o)[0][0],'guided')
        o.update(mode=4,armed=True);r.step(.3,o);r.step(.4,o)
        self.assertEqual(r.stage,'TRANSIT')
        action,target=r.step(.5,o)[0]
        self.assertEqual(action,'guided_target')
        self.assertAlmostEqual(target[0],6.,places=6);self.assertAlmostEqual(target[1],4.,places=6)
        self.assertEqual(target[2],-.1)
        self.assertIsNone(r.gps_alignment);self.assertEqual(r.alignment_distance,0.)
        o['local']=(2.,3.,-.1)
        self.assertEqual(r.step(.61,o)[0],(action,target))

    def test_transport_sends_position_only_to_cube_not_custom_joystick_steering(self):
        p=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/waypoint_mission.py'
        tree=ast.parse(p.read_text());tree.body=[n for n in tree.body if isinstance(n,ast.ClassDef)]
        ns=dict(Node=object);exec(compile(tree,str(p),'exec'),ns)
        cls=ns['WaypointMission'];node=cls.__new__(cls);node.link=Mock()
        node.send_guided_waypoint((6.,4.,-.1))
        calls=node.link.send.call_args_list
        self.assertEqual(calls[0].args,('manual_control_send',1,0,0,500,0,0))
        packet=calls[1].args
        self.assertEqual(packet[:6],('set_position_target_local_ned_send',0,1,1,1,3576))
        self.assertEqual(packet[6:9],(6.,4.,-.1))
        self.assertTrue(packet[5] & 1024);self.assertTrue(packet[5] & 2048)
        self.assertFalse(packet[5] & 7)
        self.assertEqual(node.sent_target,(6.,4.,-.1))
        node.link.command.assert_not_called()

    def test_arrival_uses_guided_braking_then_existing_stop_disarm(self):
        frame=GeographicFrame();frame.align(33.,-117.,(0.,0.),0.)
        r=GuidedSurfaceRun(dict(lat=33.,lon=-117.,mode='surface',map='geographic'),'run',0.)
        r.stage='TRANSIT';r.surface_down=-.1;r.heading=0.;r.align_heading=0.
        o=dict(fresh=True,mode=4,armed=True,bridge=dict(map_transform=frame.snapshot()),
            local=(.2,.1,-.1),yaw=0.,speed=0.)
        for i in range(1,35):
            actions=r.step(i*.1,o)
            if actions:self.assertIn(actions[0][0],('guided_hold','disarm'))
            if r.stage=='DISARM':break
        self.assertEqual(r.stage,'DISARM')
        o['armed']=False;r.step(i*.1+.1,o)
        self.assertEqual(r.stage,'COMPLETE')

if __name__=='__main__':unittest.main()
