#!/usr/bin/env python3
"""Everything that talks to the Cube through MAVProxy, in one place.

Importing this sets the MAVLink v2 environment BEFORE pymavlink loads. That
order matters and is not optional: ODOMETRY (msg 331) does not exist on v1, and
pymavlink only reads these vars at import time. Import this module before any
other pymavlink use.

Each consumer gets its own MAVProxy udpin port and its own component id so the
Cube can tell them apart - see documentation section 3.3 for the port map.
"""
import os
import time

os.environ['MAVLINK20'] = '1'                       # ODOMETRY (331) is v2-only
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'
from pymavlink import mavutil                       # noqa: E402 - must follow the env

mavutil.set_dialect('ardupilotmega')

TARGET_SYS = 1                                      # the Cube
TARGET_COMP = 1
GCS_TYPE = mavutil.mavlink.MAV_TYPE_GCS             # what we claim to be
GCS_AUTOPILOT = mavutil.mavlink.MAV_AUTOPILOT_INVALID

MODE_GUIDED = 4                                     # ArduSub custom mode number
FRAME_LOCAL_NED = 1
FRAME_LOCAL_FRD = 20
FRAME_BODY_FRD = 12
EST_VIO = 3                                         # MAV_ESTIMATOR_TYPE_VIO
MASK_POS_YAW = 0b100111111000                       # position + yaw, ignore vel/accel


class Link:
    """A MAVProxy UDP connection plus the handful of things Graey sends over it."""

    def __init__(self, url, component, logger=None):
        if logger:
            logger.info(f'opening MAVLink {url} as component {component}')
        self.mav = mavutil.mavlink_connection(url, source_system=1,
                                              source_component=component)
        self.heartbeat()                            # MAVProxy will not route to us first

    def heartbeat(self):
        self.mav.mav.heartbeat_send(GCS_TYPE, GCS_AUTOPILOT, 0, 0, 0)

    def drain(self, handler):
        """Consume every queued message. handler(msg_type, msg) per message."""
        while True:
            m = self.mav.recv_match(blocking=False)
            if m is None:
                return
            handler(m.get_type(), m)

    def command(self, cmd, *params):
        self.mav.mav.command_long_send(TARGET_SYS, TARGET_COMP, cmd, 0,
                                       *(list(params) + [0] * (7 - len(params))))

    def set_mode(self, mode):
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)

    def arm(self):
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def disarm(self, tries=10, gap=0.1):
        for _ in range(tries):                      # repeated - a single one can be dropped
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
            time.sleep(gap)

    def goto_ned(self, n, e, down, yaw):
        self.mav.mav.set_position_target_local_ned_send(
            0, TARGET_SYS, TARGET_COMP, FRAME_LOCAL_NED, MASK_POS_YAW,
            n, e, down, 0, 0, 0, 0, 0, 0, yaw, 0)
