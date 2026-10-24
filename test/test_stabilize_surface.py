import math
import unittest
from robotx_graey_2026.api.navigation.surface_control import StabilizeSurfaceControl

class StabilizeSurfaceTests(unittest.TestCase):
    def test_cardinal_rotation_and_neutral_vertical(self):
        for yaw,expected in [(0,(1,0)),(math.pi/2,(0,-1)),(math.pi,(-1,0)),(-math.pi/2,(0,1))]:
            c=StabilizeSurfaceControl()
            for i in range(100):out=c.step(i*.1,(.1,0,99),(0,0,-99),yaw,yaw)
            self.assertEqual(out[2:],(500,0))
            for a,b in zip(out[:2],expected):
                self.assertEqual((a>0)-(a<0),b)
                self.assertLessEqual(abs(a),150)
    def test_stop_clears_integral_and_brakes(self):
        c=StabilizeSurfaceControl()
        for i in range(50):c.step(i*.1,(.1,0,0),(0,0,0),0,0)
        out=c.step(5.,(0,0,0),(.1,0,0),0,0,True)
        self.assertLess(out[0],0);self.assertEqual(c.integral,[0.,0.]);self.assertEqual(out[2],500)
    def test_yaw_wrap_and_center(self):
        c=StabilizeSurfaceControl()
        self.assertEqual(c.step(0,(0,0),(0,0),math.radians(359),0)[3],0)
        self.assertGreater(c.step(.1,(0,0),(0,0),math.radians(350),0)[3],0)
    def test_invalid_data_rejected(self):
        with self.assertRaises(ValueError):StabilizeSurfaceControl().step(0,(0,0),(0,0),math.nan,0)


class SurfaceTransportTests(unittest.TestCase):
    def test_neutral_vertical_and_release_on_mode_change(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace as NS
        from unittest.mock import Mock
        from robotx_graey_2026.api.navigation.guided_surface_mission import GuidedSurfaceRun
        path=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/waypoint_mission.py'
        tree=ast.parse(path.read_text());tree.body=[n for n in tree.body if isinstance(n,ast.ClassDef)]
        ns=dict(Node=object,math=math,GuidedSurfaceRun=GuidedSurfaceRun,MODE_STABILIZE=0)
        exec(compile(tree,str(path),'exec'),ns)
        node=ns['WaypointMission'].__new__(ns['WaypointMission'])
        node.logic=GuidedSurfaceRun(dict(lat=33.,lon=-117.,mode='surface',map='geographic',alignment_corridor_confirmed=True),'r',0.)
        node.link=Mock();node.stabilize_control=StabilizeSurfaceControl();node.pilot_override_active=False
        o=dict(mode=0,armed=True,failure='',fresh=True,velocity_ned=(0,0,5),yaw=0.)
        node.send_surface(0.,(.1,0,-99),0.,o)
        self.assertEqual(node.link.send.call_args.args,('manual_control_send',1,4,0,500,0,0))
        o['mode']=19
        node.send_surface(.1,(.1,0,-99),0.,o)
        self.assertEqual(node.link.send.call_args.args,('rc_channels_override_send',1,1,*([0]*8)))
        self.assertFalse(node.pilot_override_active)
        node.link.reset_mock();node.send_surface(.2,(.1,0,0),0.,o);node.link.send.assert_not_called()
