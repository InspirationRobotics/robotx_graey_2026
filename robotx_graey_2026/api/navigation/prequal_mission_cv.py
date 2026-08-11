#!/usr/bin/env python3
"""Prequalification mission v2 - CV-assisted.

Dive, transit toward the planner's marker guess, and when pole_tracker reports a
sighting, convert it to a WORLD position and adopt that as the real marker.
Orbit at orbit_radius facing the pole the whole way, then return and surface.

The CV fix only CORRECTS THE TARGET; GUIDED still does the driving, and the fix
freezes once the orbit starts - so a detection dropout mid-orbit is harmless. If
the pole is never seen it flies the planner's coordinates unchanged.

The orbit is a SWEPT ANGLE, not a ring of waypoints. A ring cannot be flown
smoothly, for two reasons found in the water on 8/3: reach_thresh lets the sub
turn for the next waypoint before it is on the ring, so every corner is cut and
the path collapses inward (0.25 m of slop on 0.388 m spacing pulled a 0.75 m
orbit down to 0.50 m and into the pole); and each new position target resets
ArduSub's S-curve, so twelve waypoints meant twelve accelerations and a 15 cm/s
average against WPNAV_SPEED of 45. Here the angle advances at
orbit_speed/orbit_radius and the target is streamed as position PLUS the
matching tangential velocity, which ArduSub feeds to pos_control rather than
wp_nav.
"""
import math

from geometry_msgs.msg import PointStamped

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.navigation.mission_base import MissionBase, S
from robotx_graey_2026.api.navigation.frames import body_to_world

ENTRY_SPEED = 0.10                              # m/s the approach must settle below before orbiting
MAX_YAW_RATE = 0.15                             # rad/s above which a CV sighting is discarded
MAX_LEAD = 0.6                                  # rad the target may lead the sub before waiting
RADIAL_P = 0.5                                  # (m/s)/m pulling the orbit back to radius
MAX_DT = 0.5                                    # a stalled tick must not jump the sweep
LOG_S = 1.0                                     # orbit progress logged this often


class PrequalCV(MissionBase):
    def __init__(self):
        super().__init__('prequal_mission_cv')
        self.pole_ned = None                        # (n, e) world fix, None until seen
        self.pole_locked = False                    # frozen when the orbit begins
        self.a0 = None                              # orbit entry angle, frozen on entry
        self.a = 0.0                                # swept angle, advances every tick
        self.orbit_t = self.orbit_log_t = 0.0
        self.cv_open_t = 0.0                        # CV ignored until this time
        self.create_subscription(PointStamped, '/graey/pole', self.on_pole, 10)

    def enter(self, s):
        # The prequal gate is built from poles that look exactly like the marker, so
        # the first sighting after surfacing the dive is as likely to be a gate leg as
        # the target. Blind the tracker for a moment while the sub clears them.
        if s == S.TO_GATE:
            self.cv_open_t = self.now() + self.cv_blind
        super().enter(s)

    def declare_extra_parameters(self):
        # Defaults ARE the validated 8/5 configuration, because the RC one-button path does
        # not go through the GUI planner and whatever is declared here is what flies in
        # Singapore. orbit_radius is a COMMAND, not the radius flown: actual comes out
        # ~0.12 m wider at orbit_speed 0.20, so 0.8333 buys the 0.95 m measured circle.
        # That offset is calibrated against WPNAV_SPEED 35 and PSC_POSXY_P 1.0 - change
        # either and re-measure, because the error moves TOWARD the pole.
        self.declare_parameter('orbit_radius', 0.8333)
        self.declare_parameter('orbit_speed', 0.20)  # m/s along the circle
        self.declare_parameter('face_pole', True)    # false = hold start heading round the orbit
        self.declare_parameter('use_cv', True)
        self.declare_parameter('cv_blind', 5.0)      # s after the dive to ignore the tracker

    def read_extra_parameters(self):
        self.orbit_r = self.get_parameter('orbit_radius').value
        self.orbit_v = max(0.05, self.get_parameter('orbit_speed').value)  # w = v/r
        self.facing = self.get_parameter('face_pole').value
        self.use_cv = self.get_parameter('use_cv').value
        self.cv_blind = self.get_parameter('cv_blind').value

    def marker_ned(self):
        return self.pole_ned or self.fr_to_ned(self.marker_f, self.marker_r)

    def on_pole(self, msg):
        """Body-frame sighting -> world NED, adopted as the marker until locked.

        Sightings taken mid-turn are thrown away. pole_tracker medians range and
        bearing over a 10-frame window - about a second - so what arrives is a
        second-long smear of BODY-frame sightings, and rotating that by the
        INSTANTANEOUS heading mis-places it by however far the sub turned meanwhile.
        At 2 m, 20 degrees of turn throws the fix 0.7 m sideways, and since a moved
        marker changes the commanded heading it feeds itself. That put the marker in
        the pool wall twice on 8/5.
        """
        if (not self.use_cv or self.pole_locked or self.cur is None
                or self.cur_yaw is None
                or self.now() < self.cv_open_t
                or abs(self.yaw_rate) > MAX_YAW_RATE
                or self.state not in (S.TO_GATE, S.TO_MARKER)):
            return
        f, r = msg.point.x, msg.point.y
        first = self.pole_ned is None
        self.pole_ned = body_to_world(self.cur[0], self.cur[1], self.cur_yaw, f, r)
        if first:
            self.get_logger().info(
                f'*** CV LOCK: pole at NED ({self.pole_ned[0]:.2f},'
                f'{self.pole_ned[1]:.2f}) [{f:.2f} m fwd, {r:+.2f} m right] ***')

    def bearing_from(self, mn, me, n, e):
        """Angle of (n,e) about the marker, in the same convention as the sweep."""
        return math.atan2(e - me, n - mn)

    def do_to_marker(self):                         # approach, stopping one radius short
        mn, me = self.marker_ned()
        cn = self.cur[0] if self.cur else self.start_x
        ce = self.cur[1] if self.cur else self.start_y
        # Measured from HERE, not from the start point. The CV fix refines all the way
        # in, and a start-relative target SHRINKS whenever the fix moves closer - so it
        # lands behind the sub, which then reverses to reach it (seen in water 8/4).
        # From the current position the target is always the nearest point on the ring.
        dn, de = mn - cn, me - ce
        d = math.hypot(dn, de)
        if d < 1e-3:
            self.enter(S.MANEUVER)
            return
        # The nearest point ON the ring, along our own bearing - never clamped to where
        # we already are. The old max(0.0, d - orbit_r) collapsed the target onto
        # self.cur the instant the sub crossed inside orbit_r, so it stopped wherever it
        # happened to be and the orbit spiralled outward from there: entry at r=0.56 on
        # a 0.75 ring, climbing to 0.89 over the first 120 degrees (8/5).
        n, e = mn - dn / d * self.orbit_r, me - de / d * self.orbit_r
        # Heading HELD through the approach. aim_at() is open loop in the orbit, where
        # the target comes from the swept angle - but here (n,e) is derived from
        # self.cur, so aiming at the marker from it IS the sub-to-marker bearing and
        # the loop is intact. Vectored 6DOF strafes to an off-axis marker perfectly
        # well without turning, and not turning is what keeps the CV fix clean.
        self.goto_ned(n, e, self.depth, self.start_yaw, 'approach marker')
        # Close is not enough. reach_thresh hands over while wp_nav is still
        # decelerating, and the leftover velocity is RADIAL - so the orbit inherits
        # it as overshoot. Wait for the approach to actually stop.
        if self.reached() and (self.dry or self.speed() < ENTRY_SPEED):
            self.pole_locked = True
            self.a0 = None
            src = 'CV' if self.pole_ned else 'planner'
            self.get_logger().info(
                f'orbiting {src} marker at NED ({mn:.2f},{me:.2f}) r={self.orbit_r}')
            self.enter(S.MANEUVER)

    def do_maneuver(self):                          # one swept circle, facing the pole
        mn, me = self.marker_ned()
        w = self.orbit_v / self.orbit_r             # rad/s
        now = self.now()
        if self.a0 is None:                         # entry angle frozen once, or the
            self.a0 = (self.bearing_from(mn, me, self.cur[0], self.cur[1])
                       if self.cur else 0.0)        # circle restarts under the sub
            self.a = self.a0
            self.orbit_t = self.orbit_log_t = now
            self.get_logger().info(
                f'orbit start {math.degrees(self.a0):.0f} deg, '
                f'{math.degrees(w):.0f} deg/s, {2 * math.pi / w:.1f} s expected, '
                f'face_pole={self.facing}')
        dt = min(now - self.orbit_t, MAX_DT)
        self.orbit_t = now

        # The sweep waits on ANGULAR lead, never on distance. reach_thresh hands the
        # orbit over with up to that much position error, so a distance gate can be
        # tripped before the sweep has moved at all - which deadlocked the first wet
        # run. Radius error must not stop progress; only being behind may.
        sub_a, lead = self.a, 0.0
        r_now = self.orbit_r
        if self.cur:
            sub_a = self.bearing_from(mn, me, self.cur[0], self.cur[1])
            lead = (self.a - sub_a + math.pi) % (2 * math.pi) - math.pi
            r_now = math.hypot(self.cur[0] - mn, self.cur[1] - me)
        moving = self.dry or lead < MAX_LEAD
        if moving:
            self.a += w * dt

        n = mn + self.orbit_r * math.cos(self.a)
        e = me + self.orbit_r * math.sin(self.a)
        swept = math.degrees(self.a - self.a0)
        # Feedforward velocity, decomposed at the SUB's own bearing rather than the
        # target's, as a tangential part plus a radial part.
        #
        # Tangential magnitude is r_now*w, not orbit_v: holding angular pace with the
        # sweep from radius r_now takes exactly that much speed, and feeding the nominal
        # orbit_v while sitting wide is why lead climbed 11 -> 36 deg as the radius grew
        # to 1.43 (8/5). Zero while waiting, or the setpoint says "hold still AND move".
        #
        # The RADIAL part is what actually holds the circle, and it runs even while
        # waiting, because it agrees with the position target instead of fighting it.
        # Nothing commands the centripetal acceleration needed to curve - the posvel path
        # hands pos_control a zero accel vector and there is no accel field to fill on
        # this interface - so the sub settles at whatever radius its own position error
        # happens to generate the turn, and that radius is wide. This asks for the
        # correction outright. It used to be corrected by ACCIDENT: a tangent taken at
        # the target's angle carries an INWARD component of orbit_v*sin(lead), and
        # removing that grew the bulge from 1.26 to 1.43.
        v_tan = r_now * w if moving else 0.0
        v_rad = max(-self.orbit_v,
                    min(self.orbit_v, RADIAL_P * (self.orbit_r - r_now)))
        self.stream_posvel(
            n, e, self.depth,
            -v_tan * math.sin(sub_a) + v_rad * math.cos(sub_a),
            v_tan * math.cos(sub_a) + v_rad * math.sin(sub_a),
            # Yaw as a RATE, and the rate that keeps the nose on the pole is exactly
            # the orbital angular rate - circling at w while facing the centre means
            # turning at w. Zero holds heading. A yaw ANGLE re-sent at 4 Hz made the
            # sub step through discrete turns instead of rotating (seen 8/5).
            (w if moving else 0.0) if self.facing else 0.0,
            f'orbit {swept:.0f}/360 deg')

        if not self.dry and now - self.orbit_log_t > LOG_S:
            self.orbit_log_t = now                  # the only evidence a wet orbit leaves
            self.get_logger().info(
                f'orbit {swept:.0f}/360 deg lead={math.degrees(lead):+.0f} deg '
                f'r={r_now:.2f} m vr={v_rad:+.2f} '
                f'z={self.cur[2] if self.cur else 0.0:.2f}/{self.depth:.2f} m'
                f'{"" if moving else "  WAITING"}')

        if self.a - self.a0 >= 2 * math.pi:
            self.enter(S.RETURN_GATE)


def main():
    run(PrequalCV)
