#!/usr/bin/env python3
"""Everything that talks to the Cube through MAVProxy, in one place.

Importing this sets the MAVLink v2 environment BEFORE pymavlink loads. That
order matters and is not optional: ODOMETRY (msg 331) does not exist on v1, and
pymavlink only reads these vars at import time. Import this module before any
other pymavlink use.

Each consumer gets its own MAVProxy udpin port and its own component id so the
Cube can tell them apart - see documentation section 3.3 for the port map.

Every send and receive is guarded. A udpout socket is CONNECTED, so once MAVProxy
stops listening the kernel answers with ICMP port-unreachable and the socket
raises ECONNREFUSED on the next call. pymavlink does not catch it, the exception
escapes the ROS timer callback, and node_util.run() exits the process. That is
how a one-second USB hub reset on 2026-08-06 killed five nodes at once - and
restarting nav_ekf_bridge is not free, because its dead-reckoned position resets
to the origin and teleports the EKF's external-nav source. Reconnecting instead
of dying keeps that state alive.
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

MODE_STABILIZE = 0                                  # ArduSub custom mode numbers, mode.h:43
MODE_GUIDED = 4
MODE_MANUAL = 19
FRAME_LOCAL_NED = 1
FRAME_LOCAL_FRD = 20
FRAME_BODY_FRD = 12
EST_VIO = 3                                         # MAV_ESTIMATOR_TYPE_VIO
MASK_POS_YAW = 0b100111111000                       # position + yaw, ignore vel/accel
MASK_POSVEL_YAWRATE = 0b010111000000                # position + velocity + yaw RATE, ignore accel
COV_UNKNOWN = [float('nan')] + [0.0] * 20           # leading NaN = "covariance unknown"
REOPEN_MIN_S = 0.5                                  # at most one socket rebuild this often
REOPEN_LOG_S = 5.0                                  # a flapping link must not fill the log


class Link:
    """A MAVProxy UDP connection plus the handful of things Graey sends over it."""

    def __init__(self, url, component, logger=None, system=1):
        self.url, self.system, self.component = url, system, component
        self.log = logger
        self.last_reopen = self.last_reopen_log = 0.0
        if logger:
            logger.info(f'opening MAVLink {url} as {system}/{component}')
        self._open()
        self.heartbeat()                            # MAVProxy will not route to us first

    def _open(self):
        self.mav = mavutil.mavlink_connection(
            self.url, source_system=self.system, source_component=self.component)

    def _reopen(self, err):
        """Rebuild the socket after the peer went away. Returns False if declined.

        RATE LIMITED, and that is not cosmetic. This is reached from send() and from
        drain(), so an unthrottled version rebuilt ~30 sockets a second per node -
        six nodes, ~180/s. Every rebuild takes a NEW ephemeral source port, and
        MAVProxy's udpin learns its reply address from whatever last sent, so replies
        end up scattered across ports we already closed. Reconnecting too eagerly
        breaks receive worse than the outage did (2026-08-07).
        """
        now = time.monotonic()
        if now - self.last_reopen < REOPEN_MIN_S:
            return False                            # too soon; let this message drop
        self.last_reopen = now
        if self.log and now - self.last_reopen_log > REOPEN_LOG_S:
            self.last_reopen_log = now
            self.log.warn(f'MAVLink {self.url} lost ({err}); reconnecting')
        try:
            self.mav.close()
        except Exception:                           # already broken; nothing to salvage
            pass
        self._open()
        try:                                        # re-register the new source port at
            self.mav.mav.heartbeat_send(            # once, or MAVProxy keeps replying to
                GCS_TYPE, GCS_AUTOPILOT, 0, 0, 0)   # the old one and we hear nothing
        except OSError:
            pass                                    # still down; the rate limit retries
        return True

    def send(self, name, *args):
        """Every outbound message goes through here so one place handles the drop.

        Retried once, and only if the reconnect actually happened: the failing call is
        usually the one that discovers the peer is gone, and the fresh socket delivers.
        Otherwise the message is dropped rather than raised - MAVLink is lossy by
        design and every caller re-sends.
        """
        for retry in (False, True):
            try:
                getattr(self.mav.mav, name)(*args)
                return
            except OSError as e:
                if retry or not self._reopen(e):
                    return

    def heartbeat(self):
        self.send('heartbeat_send', GCS_TYPE, GCS_AUTOPILOT, 0, 0, 0)

    def drain(self, handler):
        """Consume every queued message. handler(msg_type, msg) per message."""
        while True:
            try:
                m = self.mav.recv_match(blocking=False)
            except OSError as e:                    # ECONNREFUSED arrives on recv too
                self._reopen(e)
                return
            if m is None:
                return
            handler(m.get_type(), m)

    def command(self, cmd, *params):
        self.send('command_long_send', TARGET_SYS, TARGET_COMP, cmd, 0,
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
        self.send('set_position_target_local_ned_send',
                  0, TARGET_SYS, TARGET_COMP, FRAME_LOCAL_NED, MASK_POS_YAW,
                  n, e, down, 0, 0, 0, 0, 0, 0, yaw, 0)

    def goto_posvel(self, n, e, down, vn, ve, vd, yaw_rate):
        """Position WITH a velocity feed-forward, for a target that keeps moving.

        Clearing the velocity-ignore bits makes ArduSub route this to
        pos_control input_pos_vel_accel_xy() instead of wp_nav
        set_wp_destination(). That matters: wp_nav resets an S-curve on every
        message, so goto_ned() must not be sent faster than every few seconds or
        the sub never leaves its acceleration phase. This path is a streaming
        setpoint interface and is meant to be sent every tick.

        Heading is commanded as a RATE, not an angle. A yaw angle re-sent at 4 Hz
        is a sequence of discrete turn-to-heading orders, and the vehicle visibly
        steps through them; a rate rotates continuously. Pass 0.0 to hold heading.
        """
        self.send('set_position_target_local_ned_send',
                  0, TARGET_SYS, TARGET_COMP, FRAME_LOCAL_NED, MASK_POSVEL_YAWRATE,
                  n, e, down, vn, ve, vd, 0, 0, 0, 0, yaw_rate)

    def odometry(self, pos, q, vel, rates):
        """VN-100 attitude + DVL velocity + dead-reckoned position, as one ODOMETRY.

        Here rather than in nav_ekf_bridge so the guarded send covers it too - it
        is the highest-rate stream we have and the one whose node must not die.
        """
        self.send('odometry_send',
                  0, FRAME_LOCAL_FRD, FRAME_BODY_FRD,
                  pos[0], pos[1], pos[2], list(q),
                  vel[0], vel[1], vel[2], rates[0], rates[1], rates[2],
                  COV_UNKNOWN, COV_UNKNOWN, 0, EST_VIO)
