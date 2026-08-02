#!/usr/bin/env python3
"""Prequalification mission v1 - no CV. The fallback for prequal_mission_cv.

Out through the gate, a 180 degree semicircle around the FAR side of the marker,
back through the gate, surface. NOT a full orbit. Heading is held at the start
heading throughout, so the return leg is flown in reverse - that is by design.
"""
import math

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.navigation.mission_base import MissionBase, S


class PrequalMission(MissionBase):
    def __init__(self):
        super().__init__('prequal_mission')

    def declare_extra_parameters(self):
        self.declare_parameter('uturn_radius', 1.5)
        self.declare_parameter('uturn_points', 7)

    def read_extra_parameters(self):
        self.r = self.get_parameter('uturn_radius').value
        self.npts = self.get_parameter('uturn_points').value

    def do_to_marker(self):                         # stop one radius short, on the near side
        self.goto(self.marker_f, self.marker_r - self.r, self.depth, 'marker enter')
        if self.reached():
            self.step = 0
            self.enter(S.MANEUVER)

    def do_maneuver(self):                          # semicircle around the far side
        a = -math.pi / 2 + math.pi * self.step / (self.npts - 1)
        self.goto(self.marker_f + self.r * math.cos(a),
                  self.marker_r + self.r * math.sin(a),
                  self.depth, f'uturn {self.step + 1}/{self.npts}')
        if self.reached():
            self.step += 1
            self.state_t0 = self.now()              # each waypoint gets its own timeout
            if self.step >= self.npts:
                self.enter(S.RETURN_GATE)


def main():
    run(PrequalMission)
