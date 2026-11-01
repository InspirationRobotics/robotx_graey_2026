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
        next_action,next_target=r.step(.61,o)[0]
        self.assertEqual(next_action,action)
        self.assertEqual(next_target[:3],target[:3])
        self.assertLess(next_target[3],target[3])

    def test_transport_sends_position_and_yaw_rate_and_cancels_rate_on_hold(self):
        p=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/waypoint_mission.py'
        tree=ast.parse(p.read_text());tree.body=[n for n in tree.body if isinstance(n,ast.ClassDef)]
        ns=dict(Node=object);exec(compile(tree,str(p),'exec'),ns)
        cls=ns['WaypointMission'];node=cls.__new__(cls);node.link=Mock()
        node.send_guided_waypoint((6.,4.,-.1))
        calls=node.link.send.call_args_list
        self.assertEqual(calls[0].args,('manual_control_send',1,0,0,500,0,0))
        packet=calls[1].args
        self.assertEqual(packet[:6],('set_position_target_local_ned_send',0,1,1,1,1528))
        self.assertEqual(packet[6:9],(6.,4.,-.1))
        self.assertTrue(packet[5] & 1024);self.assertFalse(packet[5] & 2048)
        self.assertFalse(packet[5] & 7)
        self.assertEqual(node.sent_target,(6.,4.,-.1))
        self.assertEqual(packet[-1],0.)
        node.send_guided_waypoint((6.,4.,-.1,.2))
        self.assertEqual(node.link.send.call_args.args[-1],.2)
        node.send_guided_waypoint((6.,4.,-.1))
        self.assertEqual(node.link.send.call_args.args[-1],0.)
        node.link.command.assert_not_called()

    def test_heading_correction_wrap_deadband_strength_and_turn_hysteresis(self):
        r=GuidedSurfaceRun(dict(lat=33.,lon=-117.,mode='surface',map='geographic'),'run',0.)
        r.surface_down=-.1
        local=(0.,0.,-.1); target=(10.,0.,-.1)
        self.assertEqual(r.forward_target(local,target,math.radians(2.))[3],0.)
        small=r.forward_target(local,target,math.radians(-10.))[3]
        large=r.forward_target(local,target,math.radians(-40.))[3]
        self.assertGreater(large,small)
        turn=r.forward_target(local,target,math.radians(-90.))
        self.assertEqual(turn[:3],local)
        self.assertAlmostEqual(turn[3],math.radians(20.))
        self.assertEqual(r.forward_target(local,target,math.radians(-30.))[:3],local)
        self.assertEqual(r.forward_target(local,target,math.radians(-20.))[:3],target)
        west=(-10.,-.01,-.1)
        self.assertEqual(r.forward_target(local,west,math.radians(179.))[3],0.)
        self.assertLess(r.forward_target(local,target,math.radians(20.))[3],0.)

    def test_east_target_turns_clockwise_then_resumes_translation(self):
        r=GuidedSurfaceRun(dict(lat=33.,lon=-117.,mode='surface',map='geographic'),'run',0.)
        r.surface_down=0.
        yaw=0.;local=(0.,0.,0.);target=(0.,10.,0.)
        first=r.forward_target(local,target,yaw)
        self.assertEqual(first[:3],local)
        self.assertGreater(first[3],0.)
        # Kinematic controller exercise only, not a Cube/vehicle simulation.
        for _ in range(200):
            command=r.forward_target(local,target,yaw)
            yaw+=command[3]*.1
        self.assertFalse(r.turning_to_target)
        self.assertEqual(command[:3],target)
        self.assertLess(abs(math.pi/2-yaw),math.radians(4.))

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
