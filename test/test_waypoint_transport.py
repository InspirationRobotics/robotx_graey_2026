import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


class WaypointTransportTest(unittest.TestCase):
    def test_live_startup_waits_for_watchdog_without_granting_permission(self):
        path=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/waypoint_mission.py'
        tree=ast.parse(path.read_text())
        tree.body=[n for n in tree.body if isinstance(n,ast.ClassDef)]
        ns=dict(Node=object,math=math)
        exec(compile(tree,str(path),'exec'),ns)
        cls=ns['WaypointMission'];node=cls.__new__(cls)
        node.sim=False;node.aborted=False;node.safety={};node.safety_time=-math.inf
        node.logic=NS(stage='WAIT_SURFACE',since=0.)
        node.supervisor=dict(published_monotonic=.1,depth_m=.1);node.supervisor_time=.1
        samples=dict(heartbeat=dict(armed=False,mode_id=19),local=dict(ned=[0,0,0],velocity=[0,0,0]),
                     attitude=dict(rpy=[0,0,0]),cube_origin=dict(lat=32.,lon=-117.,alt=0.),
                     **{'global':dict(valid=True,lat=32.,lon=-117.)})
        node.view=NS(snapshot=lambda:dict(streams={k:dict(age=0,data=v) for k,v in samples.items()}),
                     position_healthy=lambda:True)
        first=node.observation(.1)
        self.assertFalse(first['fresh']);self.assertEqual(first['failure'],'')
        self.assertIn('watchdog',node.observation(3.)['failure'])
        node.supervisor_time=.1;node.safety_time=.1
        node.safety=dict(vehicle_fresh=True,inhibit=False,sa_active=False,esc_commanded_on=True)
        self.assertTrue(node.observation(.1)['fresh'])
        node.safety['inhibit']=True
        self.assertIn('watchdog',node.observation(.1)['failure'])
