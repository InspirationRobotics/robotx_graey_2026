"""Where the sub is, for the breadcrumb map - from the VN-100 and the DVL only.

This is the same sum nav_ekf_bridge does (DVL body velocity, turned into the
world by the VN-100 attitude, added up over time), kept separately so the map
never depends on the Cube or its EKF. It is kept in the MAP's frame:

  (0, 0)   where the sub was when reset() was called
  y        forward, along the heading the sub had at that moment
  x        to the right of that
  heading  degrees clockwise from +y, so 90 = facing +x
  roll     degrees, + = right side down       } straight from the VN-100,
  pitch    degrees, + = nose up               } measured from level

Only heading CHANGES since reset() matter, so a constant heading offset (a
mis-set yaw_offset_deg, magnetic declination) cancels out.

Everything is stamped with time.monotonic() as it arrives - the same clock the
sonar uses for its pings - so pose_at(t) can say where the sub was for any ping.
While the DVL has no bottom lock the position is not added to, and every pose
says valid=False, so the map can refuse to place crumbs with it.

Nothing here imports rclpy: the sonar tools run outside the ROS container.
UdpPose at the bottom gets the readings from tools/pose_relay.py, which runs
inside it.
"""
import bisect
import json
import math
import socket
import threading
import time
from collections import namedtuple

RELAY_PORT = 14660      # where tools/pose_relay.py sends to
HISTORY_S = 120.0       # how far back pose_at() can look
STALE_S = 1.0           # no DVL velocity for this long = not valid
MAX_GAP_S = 0.5         # pose_at() won't guess further than this past the data
ALT_STALE_S = 2.0       # an altitude older than this is not used
RECORD_S = 0.05         # attitude comes fast; keep at most one pose per this

# alt_m: the DVL's height above the bottom, -1 when it has none (no lock, or not heard lately)
Pose = namedtuple("Pose", "t x y heading_deg valid roll_deg pitch_deg alt_m",
                  defaults=(0.0, 0.0, -1.0))


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def body_to_world(q, v):
    """Body (forward, right, down) to world (north, east, down). q is (w, x, y, z).
    Same maths as nav_ekf_bridge.rotate_body_to_world."""
    qc = (q[0], -q[1], -q[2], -q[3])
    w = _quat_mul(_quat_mul(q, (0.0, v[0], v[1], v[2])), qc)
    return w[1], w[2], w[3]


def heading_of(q):
    """Compass-style heading in degrees, 0-360, from a (w, x, y, z) quaternion."""
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) % 360.0


def roll_pitch_of(q):
    """(roll, pitch) in degrees from a (w, x, y, z) quaternion, the same
    yaw-pitch-roll order vn100_node builds it in."""
    w, x, y, z = q
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    return roll, pitch


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


class DeadReckoner:
    """Feed it attitude, DVL velocity and DVL valid; ask it where the sub is."""

    def __init__(self):
        self._lock = threading.Lock()
        self._q = None
        self._dvl_ok = False
        self._last_vel_t = None
        self._ne = [0.0, 0.0]       # north, east since reset, metres
        self._yaw0 = None           # heading at reset
        self._hist = []             # Pose, oldest first
        self._alt = (-1.0, None)    # DVL altitude and when it came
        self._times = []            # their .t, for bisect
        self.reset_needed = True    # reset() as soon as there is an attitude

    # ---- inputs -----------------------------------------------------------
    def on_attitude(self, q, t=None):
        t = time.monotonic() if t is None else t
        with self._lock:
            self._q = q
            if self.reset_needed:
                self._reset_locked()
            if not self._times or t - self._times[-1] >= RECORD_S:
                self._record(t)

    def on_dvl_valid(self, ok, t=None):
        with self._lock:
            self._dvl_ok = bool(ok)

    def on_altitude(self, alt, t=None):
        t = time.monotonic() if t is None else t
        with self._lock:
            self._alt = (float(alt), t)

    def on_velocity(self, v_body, t=None):
        t = time.monotonic() if t is None else t
        with self._lock:
            if self._last_vel_t is not None and self._dvl_ok and self._q is not None:
                dt = t - self._last_vel_t
                if 0.0 < dt < STALE_S:              # ignore stalls, as the bridge does
                    vn, ve, _ = body_to_world(self._q, v_body)
                    self._ne[0] += vn * dt
                    self._ne[1] += ve * dt
            self._last_vel_t = t
            if self._q is not None:
                self._record(t)

    # ---- outputs ----------------------------------------------------------
    def reset(self):
        """Make where the sub is now (0, 0), facing +y. Forgets all history."""
        with self._lock:
            if self._q is None:
                self.reset_needed = True        # do it when the first attitude arrives
            else:
                self._reset_locked()

    def now(self):
        with self._lock:
            if self._q is None:
                return None
            return self._pose(time.monotonic())

    def pose_at(self, t):
        """Where the sub was at monotonic time t, or None if there is no data
        that close. Between two samples it draws a straight line."""
        with self._lock:
            h = self._hist
            if not h or t < h[0].t - MAX_GAP_S or t > h[-1].t + MAX_GAP_S:
                return None
            i = bisect.bisect_left(self._times, t)
            if i == 0:
                return h[0]._replace(t=t)
            if i == len(h):
                return h[-1]._replace(t=t)
            a, b = h[i - 1], h[i]
            f = 0.0 if b.t == a.t else (t - a.t) / (b.t - a.t)
            return Pose(t, a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
                        (a.heading_deg + f * wrap180(b.heading_deg - a.heading_deg)) % 360.0,
                        a.valid and b.valid,
                        a.roll_deg + f * (b.roll_deg - a.roll_deg),
                        a.pitch_deg + f * (b.pitch_deg - a.pitch_deg),
                        a.alt_m + f * (b.alt_m - a.alt_m) if min(a.alt_m, b.alt_m) > 0 else -1.0)

    def turn_rate(self, t, window=0.3):
        """How fast the sub was turning just before t, degrees per second
        (+ = turning right), or None if there is no data to tell."""
        a, b = self.pose_at(t - window), self.pose_at(t)
        if a is None or b is None:
            return None
        return wrap180(b.heading_deg - a.heading_deg) / window

    def track(self):
        """Every recorded pose since reset, for drawing the sub's path."""
        with self._lock:
            return list(self._hist)

    # ---- inside -----------------------------------------------------------
    def _reset_locked(self):
        self._ne = [0.0, 0.0]
        self._yaw0 = heading_of(self._q)
        self._hist = []
        self._times = []
        self.reset_needed = False

    def _pose(self, t):
        # north/east turned so that +y is the heading at reset
        a = math.radians(self._yaw0)
        n, e = self._ne
        y = n * math.cos(a) + e * math.sin(a)
        x = -n * math.sin(a) + e * math.cos(a)
        fresh = self._last_vel_t is not None and t - self._last_vel_t < STALE_S
        alt, alt_t = self._alt
        alt = alt if alt > 0 and alt_t is not None and t - alt_t < ALT_STALE_S else -1.0
        return Pose(t, x, y, (heading_of(self._q) - self._yaw0) % 360.0,
                    self._dvl_ok and fresh, *roll_pitch_of(self._q), alt)

    def _record(self, t):
        if self._times and t <= self._times[-1]:
            # same moment as the last one (attitude and velocity together):
            # keep only the newest, which has the movement added
            self._hist[-1] = self._pose(self._times[-1])
            return
        self._hist.append(self._pose(t))
        self._times.append(t)
        while self._times[0] < t - HISTORY_S:
            self._hist.pop(0)
            self._times.pop(0)


class UdpPose(DeadReckoner):
    """DeadReckoner fed by tools/pose_relay.py, which runs inside the ROS
    container. Each reading is stamped the moment it arrives here."""

    def __init__(self, port=RELAY_PORT):
        super().__init__()
        self.log = None             # log(t, packet): every reading as it arrives, for recordings
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", port))
        threading.Thread(target=self._listen, daemon=True).start()

    def _listen(self):
        while True:
            try:
                d = json.loads(self._sock.recv(512))
            except ValueError:
                continue                    # not one of ours; ignore it
            log = self.log
            if log is not None:
                log(time.monotonic(), d)
            if "att" in d:
                self.on_attitude(tuple(d["att"]))
            elif "vel" in d:
                self.on_velocity(tuple(d["vel"]))
            elif "ok" in d:
                self.on_dvl_valid(d["ok"])
            elif "alt" in d:
                self.on_altitude(d["alt"])
