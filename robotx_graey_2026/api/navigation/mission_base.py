#!/usr/bin/env python3
"""Shared machinery for the prequalification missions.

Both missions fly the same skeleton - wait for nav, arm, dive, out through the
gate, do something at the marker, come back, surface, disarm - and differ only
in how they approach the marker and what they do there. Those two steps are the
hooks do_to_marker() and do_maneuver(); everything else lives here.

Waypoints are (forward, right) offsets in a mission frame aligned to the heading
captured at mission start, then rotated into local NED.

SAFETY: dry_run defaults True -> prints the plan and state transitions only;
never arms, never changes mode, never sends setpoints. Set dry_run:=false only
in the water.
"""
import math
import time
from enum import Enum

import os
import sys
from rclpy.node import Node
from std_msgs.msg import Bool
from robotx_graey_2026.api.navigation.frames import body_to_world
from robotx_graey_2026.api.pixhawk.mavlink import (
    Link, MODE_GUIDED, MODE_MANUAL, mavutil)

RESEND_S = 3.0                                  # unchanged targets re-sent no faster than
                                                # this: streaming restarts ArduSub's
                                                # trajectory planner and the sub crawls
DIVE_THRESH = 0.10                              # DIVE arrival, on depth alone - see reached_depth
DRY_LOG_S = 1.0                                 # streamed targets logged this often in dry run
GUIDED_NAME = 'GUIDED'                          # HEARTBEAT reports a NAME, not MODE_GUIDED's 4


class S(Enum):
    WAIT_RC = 10                                # only entered when rc_start is true
    WAIT_NAV = 0
    ARM = 1
    DIVE = 2
    TO_GATE = 3
    TO_MARKER = 4
    MANEUVER = 5                                # U-turn in v1, orbit in the CV mission
    RETURN_GATE = 6
    RETURN_HOME = 7
    SURFACE = 8
    DONE = 9


class MissionBase(Node):
    def __init__(self, name, component=198):
        super().__init__(name)
        p = self.declare_parameter
        p('mavlink', 'udpout:127.0.0.1:14553')
        p('dry_run', True)
        p('depth', 1.5)
        p('gate_forward', 4.0)
        p('marker_forward', 13.0)
        p('marker_right', 0.0)
        p('reach_thresh', 0.10)                 # 0.4 exceeded the dive delta and self-completed;
                                                # 0.25 let the orbit hand over a quarter metre
                                                # inside the ring, which the orbit inherited
        p('state_timeout', 60.0)
        p('sim_reach_time', 2.0)
        p('rc_start', False)                    # true = sit and wait for the RC button
        # BOTH must be above CH8. MANUAL_CONTROL from a joystick pins RC1-8, so an
        # abort on CH7 reads the override instead of the switch and is silently dead
        # for as long as QGC has a joystick connected.
        p('rc_start_channel', 9)                # SE, momentary - cannot be left latched
        p('rc_abort_channel', 8)               # SA, INVERTED in EdgeTX so down = kill
        p('rc_high', 1700)                      # PWM above this counts as pressed
        self.declare_extra_parameters()

        g = self.get_parameter
        self.dry = g('dry_run').value
        self.depth = g('depth').value
        self.gate_f = g('gate_forward').value
        self.marker_f = g('marker_forward').value
        self.marker_r = g('marker_right').value
        self.thresh = g('reach_thresh').value
        self.timeout = g('state_timeout').value
        self.sim_reach = g('sim_reach_time').value
        self.rc_wait = g('rc_start').value
        self.rc_start_ch = g('rc_start_channel').value
        self.rc_abort_ch = g('rc_abort_channel').value
        self.rc_high = g('rc_high').value
        self.read_extra_parameters()

        self.link = Link(g('mavlink').value, component, self.get_logger())
        self.get_logger().info(f'dry_run={self.dry} rc_start={self.rc_wait}')

        # A switch already up when we start must not count. Both channels have to be
        # seen LOW once before they will fire - otherwise a latched abort switch
        # blocks every mission, and a held start button retriggers immediately.
        self.rc = {}
        self.start_armed = False
        self.abort_armed = False
        self.mode = None                        # the Cube's flight mode, from HEARTBEAT
        self.saw_guided = False                 # ... and whether we ever got GUIDED

        self.state = S.WAIT_RC if self.rc_wait else S.WAIT_NAV
        self.state_t0 = self.now()
        self.start_x = self.start_y = self.start_yaw = 0.0
        self.cur = None
        self.cur_yaw = None
        self.yaw_rate = 0.0                     # rad/s; a CV sighting taken mid-turn is stale
        self.vel = None
        self.target = None
        self.last_send_t = 0.0
        self.step = 0                           # waypoint index within MANEUVER

        self.auto_pub = self.create_publisher(Bool, '/graey/autonomy_active', 10)
        self.create_timer(0.1, self.pump_mavlink)
        self.create_timer(0.25, self.tick)
        self.create_timer(5.0, self.request_rc)
        self.request_rc()

    def request_rc(self):
        """ArduPilot only streams RC_CHANNELS if something asks. Repeated because
        MAVProxy may not be up yet when we start, and the request is one packet."""
        if not self.dry:
            self.link.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                              mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 100000)

    # ---------- subclass hooks ----------
    def declare_extra_parameters(self):
        pass

    def read_extra_parameters(self):
        pass

    def do_to_marker(self):
        raise NotImplementedError

    def do_maneuver(self):
        raise NotImplementedError

    # ---------- helpers ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def enter(self, s):
        self.get_logger().info(f'--- {self.state.name} -> {s.name} ---')
        self.state = s
        self.state_t0 = self.now()

    def fr_to_ned(self, forward, right):
        return body_to_world(self.start_x, self.start_y, self.start_yaw, forward, right)

    def goto_ned(self, n, e, down, yaw, label):
        tgt = (n, e, down, yaw)
        changed = (self.target is None
                   or any(abs(a - b) > 0.01 for a, b in zip(tgt[:3], self.target[:3]))
                   or abs(yaw - self.target[3]) > 0.02)
        self.target = tgt
        if self.dry:
            if changed:
                self.get_logger().info(
                    f'    [dry] {label}: NED=({n:.2f},{e:.2f},{down:.2f}) '
                    f'yaw={math.degrees(yaw):.0f}')
            return
        if not changed and self.now() - self.last_send_t < RESEND_S:
            return
        self.last_send_t = self.now()
        self.link.goto_ned(n, e, down, yaw)

    def goto(self, forward, right, down, label):
        n, e = self.fr_to_ned(forward, right)
        self.goto_ned(n, e, down, self.start_yaw, label)

    def stream_posvel(self, n, e, down, vn, ve, yaw_rate, label):
        """A target that keeps moving, sent EVERY tick on purpose.

        No change gate, unlike goto_ned - Link.goto_posvel() reaches ArduSub's
        streaming setpoint interface rather than wp_nav's S-curve, so a high rate
        is what it wants. Heading is a RATE here, not an angle. The dry-run print
        is throttled because 4 Hz would bury the log.
        """
        self.target = (n, e, down, self.cur_yaw or 0.0)
        if self.dry:
            if self.now() - self.last_send_t > DRY_LOG_S:
                self.last_send_t = self.now()
                self.get_logger().info(
                    f'    [dry] {label}: NED=({n:.2f},{e:.2f},{down:.2f}) '
                    f'v=({vn:+.2f},{ve:+.2f}) yawrate={math.degrees(yaw_rate):+.0f}/s')
            return
        self.link.goto_posvel(n, e, down, vn, ve, 0.0, yaw_rate)

    def reached(self):
        if self.dry:
            return self.now() - self.state_t0 > self.sim_reach
        if self.cur is None or self.target is None:
            return False
        return math.dist(self.cur, self.target[:3]) < self.thresh

    def reached_depth(self):
        """DIVE only. reached() measures 3D distance, but on the dive the
        horizontal error is already zero, so the whole of reach_thresh is spent
        on depth: a 0.35 m descent 'arrives' at 0.20 m and the sub sets off for
        the marker half submerged. Depth gets its own tighter tolerance."""
        if self.dry:
            return self.now() - self.state_t0 > self.sim_reach
        if self.cur is None or self.target is None:
            return False
        return abs(self.cur[2] - self.target[2]) < DIVE_THRESH

    def lag(self):
        """Horizontal distance behind the streamed target. A swept target waits
        on this so it cannot run away from the vehicle."""
        if self.cur is None or self.target is None:
            return 0.0
        return math.hypot(self.cur[0] - self.target[0], self.cur[1] - self.target[1])

    def speed(self):
        """Horizontal ground speed from the EKF. Zero until velocity arrives, so a
        caller gating on it degrades to the old distance-only behaviour rather
        than stalling."""
        if self.vel is None:
            return 0.0
        return math.hypot(self.vel[0], self.vel[1])

    def pump_mavlink(self):
        self.link.heartbeat()

        def handle(kind, m):
            if kind == 'LOCAL_POSITION_NED':
                self.cur = (m.x, m.y, m.z)
                self.vel = (m.vx, m.vy, m.vz)
            elif kind == 'ATTITUDE':
                self.cur_yaw = m.yaw
                self.yaw_rate = m.yawspeed
            elif kind == 'HEARTBEAT' and m.get_srcComponent() == 1:
                self.mode = self.link.mav.flightmode
            elif kind == 'RC_CHANNELS':
                for ch in (self.rc_start_ch, self.rc_abort_ch):
                    self.rc[ch] = getattr(m, f'chan{ch}_raw', 0)
                if self.rc[self.rc_start_ch] < self.rc_high:
                    self.start_armed = True     # seen low, so a press now is real
                if self.rc[self.rc_abort_ch] < self.rc_high:
                    self.abort_armed = True
        self.link.drain(handle)

    def rc_pressed(self, channel, armed):
        return armed and self.rc.get(channel, 0) > self.rc_high

    # ---------- state machine ----------
    def tick(self):
        if self.state == S.WAIT_RC:             # idle on deck, nothing published
            if self.rc_pressed(self.rc_start_ch, self.start_armed):
                self.get_logger().info('RC START pressed')
                self.enter(S.WAIT_NAV)
            return

        if self.state != S.DONE:
            self.auto_pub.publish(Bool(data=True))

        if (self.state != S.DONE
                and self.rc_pressed(self.rc_abort_ch, self.abort_armed)):
            self.get_logger().warn('RC ABORT - disarming')
            self.enter(S.DONE)                  # DONE disarms, sets MANUAL, exits

        # A human changing flight mode takes the vehicle, and the mission must let go.
        # Without this the node kept streaming GUIDED setpoints underneath the pilot,
        # and kept publishing autonomy_active - so the LEDs stayed GREEN while someone
        # drove manually, which is the opposite of what the state indicator is for.
        #
        # Exits WITHOUT disarming, unlike every other exit here: the pilot is driving
        # and taking their thrusters away mid-manoeuvre would be worse than the bug.
        # WAIT_RC is excluded so the sub can be driven into position while a mission
        # sits armed and waiting on SE.
        if self.state not in (S.WAIT_RC, S.WAIT_NAV, S.ARM, S.DONE):
            if self.mode == GUIDED_NAME:
                self.saw_guided = True
            elif self.saw_guided:
                self.get_logger().warn(
                    f'mode changed to {self.mode} - pilot has the vehicle, mission out')
                self.auto_pub.publish(Bool(data=False))   # LEDs to yellow at once
                sys.stdout.flush()
                sys.stderr.flush()
                time.sleep(0.25)                # let that publish actually leave
                os._exit(0)

        if (self.state not in (S.WAIT_NAV, S.DONE)
                and self.now() - self.state_t0 > self.timeout):
            nxt = S.DONE if self.state == S.SURFACE else S.SURFACE
            self.get_logger().warn(f'{self.state.name} timed out -> {nxt.name}')
            self.enter(nxt)                     # a stuck SURFACE must disarm, not re-enter
                                                # itself and reset its own clock forever

        if self.state == S.WAIT_NAV:
            if self.dry or (self.cur is not None and self.cur_yaw is not None):
                if self.cur:
                    self.start_x, self.start_y = self.cur[0], self.cur[1]
                self.start_yaw = self.cur_yaw if self.cur_yaw is not None else 0.0
                self.get_logger().info(
                    f'nav ready, start=({self.start_x:.2f},{self.start_y:.2f}) '
                    f'yaw={math.degrees(self.start_yaw):.0f}')
                self.enter(S.ARM)

        elif self.state == S.ARM:
            if self.dry:
                self.get_logger().info('    [dry] set mode GUIDED + ARM')
            else:
                self.link.set_mode(MODE_GUIDED)
                self.link.arm()
            self.goto(0.0, 0.0, self.depth, 'initial hold')
            self.enter(S.DIVE)

        elif self.state == S.DIVE:
            self.goto(0.0, 0.0, self.depth, 'dive')
            if self.reached_depth():
                self.enter(S.TO_GATE)

        elif self.state == S.TO_GATE:
            self.goto(self.gate_f, 0.0, self.depth, 'through gate')
            if self.reached():
                self.enter(S.TO_MARKER)

        elif self.state == S.TO_MARKER:
            self.do_to_marker()

        elif self.state == S.MANEUVER:
            self.do_maneuver()

        elif self.state == S.RETURN_GATE:
            self.goto(self.gate_f, 0.0, self.depth, 'back through gate')
            if self.reached():
                self.enter(S.RETURN_HOME)

        elif self.state == S.RETURN_HOME:
            self.goto(0.0, 0.0, self.depth, 'return home')
            if self.reached():
                self.enter(S.SURFACE)

        elif self.state == S.SURFACE:
            self.goto(0.0, 0.0, 0.0, 'surface')
            if self.reached():
                self.enter(S.DONE)

        elif self.state == S.DONE:
            self.auto_pub.publish(Bool(data=False))
            if self.dry:
                self.get_logger().info('    [dry] DISARM + MANUAL')
            else:
                self.link.disarm()                  # motors off BEFORE the setpoint
                self.link.set_mode(MODE_MANUAL)     # stream stops, or GUIDED lurches
            self.get_logger().info('MISSION COMPLETE - disarmed, MANUAL')
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(0)                             # rclpy.shutdown() from inside a
                                                    # timer callback left it spinning
