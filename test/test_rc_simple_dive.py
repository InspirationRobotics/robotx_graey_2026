import ast
import unittest
from pathlib import Path
from types import SimpleNamespace

from robotx_graey_2026.api.navigation.rc_start import RcStartEdge


ROOT=Path(__file__).parents[1]


class RcStartEdgeTests(unittest.TestCase):
    def test_held_high_never_starts_and_low_high_starts_once(self):
        edge=RcStartEdge(channel=9,high=1700)
        self.assertFalse(edge.update(SimpleNamespace(chan9_raw=1900)))
        self.assertFalse(edge.update(SimpleNamespace(chan9_raw=1000)))
        self.assertTrue(edge.update(SimpleNamespace(chan9_raw=1900)))
        self.assertFalse(edge.update(SimpleNamespace(chan9_raw=1000)))
        self.assertFalse(edge.update(SimpleNamespace(chan9_raw=1900)))
        self.assertTrue(edge.snapshot()['started'])

    def test_invalid_pwm_cannot_start(self):
        edge=RcStartEdge()
        for value in (None,0,65535,1700):
            self.assertFalse(edge.update(SimpleNamespace(chan9_raw=value)))
        self.assertFalse(edge.snapshot()['low_seen'])


class RcSimpleDiveLaunchTests(unittest.TestCase):
    def test_core_launch_uses_adapted_executor_and_competition_wait(self):
        source=(ROOT/'launch/core.launch.py').read_text()
        tree=ast.parse(source)
        self.assertIn("executable='waypoint_mission'",source)
        self.assertIn("'rc_start':True",source)
        self.assertIn("'rc_start_channel':9",source)
        self.assertIn("operator_kind='dive_here'",source)
        self.assertIn("competition=True",source)
        self.assertIn("dive=dict(depth_m=4.0)",source)
        self.assertNotIn("dive=dict(altitude_m=3.5,max_depth_m=5.0)",source)
        self.assertNotIn("executable='prequal_mission'",source)
        self.assertNotIn("executable='prequal_mission_cv'",source)
        self.assertTrue(any(isinstance(node,ast.FunctionDef)
                            and node.name=='generate_launch_description' for node in tree.body))


if __name__=='__main__':unittest.main()
