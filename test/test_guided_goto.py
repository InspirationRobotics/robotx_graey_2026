"""tools/guided_goto.py on the gateway: refuses unless armed in GUIDED, aims forward_m along the heading,
and sends as a component the gateway admits for position targets."""
import importlib.util
import math
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

from pymavlink.dialects.v20 import ardupilotmega as mav

from robotx_graey_2026.api.pixhawk.gateway_core import GatewayPolicy
from robotx_graey_2026.api.pixhawk.identity import FCU_COMPONENT, FCU_SYSTEM

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('guided_goto_under_test', ROOT/'tools/guided_goto.py')
goto = importlib.util.module_from_spec(spec)
spec.loader.exec_module(goto)


def cube(kind, **fields):
    m = NS(**fields)
    m.get_srcSystem, m.get_srcComponent, m.get_type = (lambda: FCU_SYSTEM), (lambda: FCU_COMPONENT), (lambda: kind)
    return kind, m


class FakeLink:
    def __init__(self, mode, armed, frames=None):
        self.mav = NS(flightmode=mode)
        self.frames = frames if frames is not None else [
            cube('LOCAL_POSITION_NED', x=2.0, y=-1.0, z=0.5),
            cube('ATTITUDE', yaw=math.radians(90)),
            cube('HEARTBEAT', base_mode=128 if armed else 0)]
        self.sent = []

    def drain(self, handler):
        for kind, m in self.frames:
            handler(kind, m)

    def goto_ned(self, *goal):
        self.sent.append(goal)


class GuidedGotoTests(unittest.TestCase):
    def test_target_is_forward_along_heading_at_the_same_depth(self):
        link = FakeLink('GUIDED', armed=True)
        pose = goto.Pose()
        self.assertIsNone(goto.wait_for_pose(link, pose, sleep=lambda s: None))
        n, e, d, yaw = goto.target(pose, 1.5)
        self.assertAlmostEqual(n, 2.0, places=6)          # heading east: north unchanged
        self.assertAlmostEqual(e, 0.5, places=6)
        self.assertEqual((d, yaw), (0.5, math.radians(90)))

    def test_refuses_unless_armed_in_guided(self):
        for mode, armed in (('POSHOLD', True), ('GUIDED', False)):
            problem = goto.wait_for_pose(FakeLink(mode, armed), goto.Pose(), sleep=lambda s: None)
            self.assertIn('arm it in GUIDED first', problem)

    def test_no_position_is_reported_not_streamed(self):
        clock = iter(range(100))
        problem = goto.wait_for_pose(FakeLink('GUIDED', True, frames=[]), goto.Pose(),
                                     clock=lambda: next(clock), sleep=lambda s: None)
        self.assertIn('no position', problem)

    def test_streams_the_goal_until_stopped(self):
        link, ticks = FakeLink('GUIDED', True), iter([True, True, True, False])
        goto.stream(link, (1, 2, 0.5, 0.0), sleep=lambda s: None, keep_going=lambda: next(ticks))
        self.assertEqual(link.sent, [(1, 2, 0.5, 0.0)]*3)

    def test_gateway_admits_its_position_target_only_when_permitted_and_armed(self):
        policy, now = GatewayPolicy(), time.monotonic()
        msg = mav.MAVLink_set_position_target_local_ned_message(0, FCU_SYSTEM, FCU_COMPONENT, 1, 0b100111111000,
                                                                1, 2, 0.5, 0, 0, 0, 0, 0, 0, 0, 0)
        msg.pack(mav.MAVLink(None, srcSystem=255, srcComponent=goto.COMPONENT))
        self.assertFalse(policy.admit(msg, goto.COMPONENT, now)[0])                 # no control status yet
        policy.update_control(dict(published_monotonic=now, vehicle_fresh=True, sa_active=False,
                                   esc_commanded_on=True, inhibit=None, armed=True), now)
        self.assertTrue(policy.admit(msg, goto.COMPONENT, now)[0])


if __name__ == '__main__':
    unittest.main()
