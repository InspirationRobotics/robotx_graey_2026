#!/usr/bin/env python3
"""Shortest autonomous run there is: arm, drive one leg, disarm. Holds its start depth.

    ros2 run robotx_graey_2026 demo_mission --ros-args -p dry_run:=false \
        -p rc_start:=true -p gate_forward:=1.0 -p hold_offset:=0.3

Built for the safety video rather than the water. It exists so an autonomous run
can be demonstrated - and killed - just under the surface in about fifteen
seconds. Both subclass hooks fall straight through, so the whole mission is the
base class's TO_GATE leg and nothing else.

THE depth PARAMETER IS IGNORED HERE, deliberately. mission_base treats depth as an
ABSOLUTE local-NED z, not an offset from the start, so depth:=0.0 does not mean
"the surface" - it means "the EKF origin's depth", and commanding zero against an
origin that sits lower makes the vehicle dive to reach it. This mission captures
the live depth at mission start instead and adds hold_offset.

WHY hold_offset SHOULD NOT BE ZERO. Graey is positively buoyant, so holding any
depth needs continuous downward thrust - the controller is never at rest. Exactly
at the surface the vertical thrusters are half in air, cavitate, produce almost
nothing, and the controller winds up; when the props finally get wetted the whole
accumulated output arrives at once and pulls the vehicle under. 0.2-0.3 m is
enough to keep them submerged and the loop stable, and still shallow enough to
film. There is no way to run GUIDED with the vertical axis disabled.

SE (CH9) starts it. SA (CH10) kills it exactly as it would mid-prequal:
kill_switch disarms and forces MANUAL, mission_base sees the mode leave GUIDED and
drops the mission. None of that is re-implemented here, so the video shows the
real path, not a demo of a demo.
"""
from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.navigation.mission_base import MissionBase, S


class DemoMission(MissionBase):
    def __init__(self):
        super().__init__('demo_mission')
        self.create_timer(0.5, self.log_depth)

    def log_depth(self):
        """Depth estimate against depth target, twice a second.

        The whole question when the vehicle drives itself down is whether the
        estimate follows it. If z tracks the descent the controller is commanding
        it; if z sits still while the sub visibly sinks, the estimate is wrong and
        the controller is chasing an error that does not exist.
        """
        if self.cur is None:
            return
        tgt = self.target[2] if self.target else float('nan')
        vz = self.vel[2] if self.vel else float('nan')
        self.get_logger().info(
            f'z={self.cur[2]:+.2f} target={tgt:+.2f} vz={vz:+.2f} {self.state.name}')

    def declare_extra_parameters(self):
        self.declare_parameter('hold_offset', 0.3)  # metres below the start depth

    def read_extra_parameters(self):
        self.hold_offset = self.get_parameter('hold_offset').value

    def enter(self, s):
        # WAIT_NAV captures start_x and start_y but not depth, so take it here, on
        # the one transition that has both a fix and a mission about to begin.
        if s is S.ARM and self.cur is not None:
            self.depth = self.cur[2] + self.hold_offset
            self.get_logger().info(
                f'start depth {self.cur[2]:+.2f} m, holding {self.depth:+.2f} m')
        super().enter(s)

    # Nothing to do at the marker - TO_GATE already drove the only leg there is.
    def do_to_marker(self):
        self.enter(S.MANEUVER)

    def do_maneuver(self):
        # Straight to DONE rather than via SURFACE, whose goto() hardcodes z=0 -
        # the EKF origin's depth, which is not the surface and not where we began.
        self.enter(S.DONE)


def main():
    run(DemoMission)
