#!/usr/bin/env python3
"""GUIDED test: drive forward N metres along the current heading, then hold there.

Runs inside the graey container and talks to the Cube through telemetry_gateway as the
mission component 198 (the gateway only forwards position targets from 198):

    docker exec -it graey bash -c 'source /opt/ros/humble/setup.bash &&
        source /root/robotx_ws/install/setup.bash && cd /root/robotx_ws/src/robotx_graey_2026 &&
        python3 tools/guided_goto.py 1.0'

Before starting:
  1. Navigation started (GUI Start Navigation), so the Cube has a position.
  2. The sub ARMED in GUIDED (RadioMaster or GUI) with the kill switch in reach.
     This tool never arms or changes mode; it stops if the sub isn't armed in GUIDED.
The gateway passes the target only while the kill switch reports control permitted and
the sub armed, so SA KILL or a disarm cuts it off.

Streams the target at 5 Hz until Ctrl-C. Switch to POSHOLD BEFORE Ctrl-C: ArduSub
lurches ~1 m forward when the setpoint stream stops.
"""
import argparse
import math
import threading
import time

from robotx_graey_2026.api.navigation.frames import body_to_world
from robotx_graey_2026.api.pixhawk.identity import FCU_COMPONENT, FCU_SYSTEM
from robotx_graey_2026.api.pixhawk.mavlink import Link

COMPONENT = 198                 # the mission component in api/pixhawk/gateway_core.py
KINDS = ('LOCAL_POSITION_NED', 'ATTITUDE')     # HEARTBEAT is always added by the transport
ARMED = 128                     # MAV_MODE_FLAG_SAFETY_ARMED


class Pose:
    """The Cube's latest local position, yaw, mode and armed state."""

    def __init__(self):
        self.pos = self.yaw = self.mode = None
        self.armed = False

    def handle(self, kind, m):
        if (m.get_srcSystem(), m.get_srcComponent()) != (FCU_SYSTEM, FCU_COMPONENT):
            return
        if kind == 'LOCAL_POSITION_NED':
            self.pos = (m.x, m.y, m.z)
        elif kind == 'ATTITUDE':
            self.yaw = m.yaw
        elif kind == 'HEARTBEAT':
            self.armed = bool(m.base_mode & ARMED)

    def ready(self):
        return self.pos is not None and self.yaw is not None


def target(pose, forward_m):
    """(north, east, down, yaw): forward_m ahead of the current position, same depth and heading."""
    n, e, d = pose.pos
    tn, te = body_to_world(n, e, pose.yaw, forward_m, 0.0)
    return tn, te, d, pose.yaw


def wait_for_pose(link, pose, timeout=5.0, clock=time.monotonic, sleep=time.sleep):
    end = clock() + timeout
    while not pose.ready() and clock() < end:
        link.drain(pose.handle)
        sleep(0.1)
    pose.mode = link.mav.flightmode
    if not pose.ready():
        return 'no position/attitude from the Cube - is the gateway running and navigation started?'
    if pose.mode != 'GUIDED' or not pose.armed:
        return f'the sub is {pose.mode}, {"armed" if pose.armed else "disarmed"} - arm it in GUIDED first'
    return None


def stream(link, goal, period=0.2, sleep=time.sleep, keep_going=lambda: True):
    while keep_going():
        link.goto_ned(*goal)
        sleep(period)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('forward_m', nargs='?', type=float, default=1.0)
    args = ap.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)     # Ctrl-C stays a KeyboardInterrupt
    node = Node('guided_goto')
    link = Link('ros', COMPONENT, node=node, kinds=KINDS)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        pose = Pose()
        problem = wait_for_pose(link, pose)
        if problem:
            raise SystemExit(problem)
        goal = target(pose, args.forward_m)
        n, e, _ = pose.pos
        print(f'now ({n:.2f}, {e:.2f}) hdg {math.degrees(pose.yaw) % 360:.0f} deg -> target NED '
              f'({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}); streaming - POSHOLD first, then Ctrl-C')
        stream(link, goal)
    except KeyboardInterrupt:
        print('stopped streaming')
    finally:
        rclpy.shutdown()               # wakes the spin thread; let it end before the node goes
        spinner.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
