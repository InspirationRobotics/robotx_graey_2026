"""Move forward, dive, turn a full circle, come back up, disarm, and ask for the kill.

Chris, 12 Nov 2026, for camera and sonar data over the pipeline: drive Graey there by hand, press
Start on the radio page's Missions tab. It can be run again and again. How deep it goes is one of:

    above the floor   altitude_m above the floor by the DVL (3.5 m at the pipeline), never deeper
                      than max_depth_m below where it started. No bottom lock all the way down to
                      max_depth_m: it comes back up WITHOUT the turn (Chris's call, Nov 12).
    pool test         depth_m below where it started, typed; the floor is not looked at.

    WAIT_SURFACE   until the Cube's telemetry and the kill switch's permission are both seen
    GUIDED, ARM    take the vehicle; the spot, depth and heading it has now are the start
    FORWARD        Chris Simple Dive only: 1 m along the captured starting heading
    DESCEND        straight down to the target above, holding that spot and heading
    SPIN           eight 45 degree turns, HOLD_S at each, back to the start heading
    SURFACE        back up to SURFACED_M below the start depth, then straight to
    DISARM         and kill_requested: waypoint_mission asks kill_switch for the killed state

radio_route.py runs the same dive at the end of a GPS route (begin() and surfaced() are the two
places it takes over). Pure decisions, like simple_sequence: waypoint_mission turns the actions
into MAVLink.

ONCE IT HAS THE VEHICLE, EVERY ENDING BUT ONE IS DISARM AND KILL. Graey floats, so thrusters off
means it comes up. A stage that runs out of time, a lost kill-switch permission, lost telemetry,
an abort from the page: all disarm and ask for the kill. The one exception is a pilot who changes
the mode or disarms: they have the vehicle, and the mission lets go without touching it. A run
that fails before the Cube has confirmed GUIDED never had the vehicle, and leaves it as it was.

There is no radio under water. An abort from the page cannot arrive during the dive, and neither
can SA: the stage time limits are what ends a dive that has gone wrong.

A descent that does not reach its target in time (the pool floor is nearer than the number typed)
skips the turn and surfaces: turning while pushing into the floor would grind the hull on it. So
does a floor too near to be altitude_m above it at all.

The surface is not held. Exactly at the surface the vertical thrusters are half in air and the
depth controller winds up (demo_mission.py tells that story), so SURFACED_M below the start depth
counts as up and the very next thing is the disarm.
"""
import math
import secrets
import statistics
from collections import deque

from .guided_surface_mission import GuidedSurfaceRun

MIN_DEPTH_M, MAX_DEPTH_M = 0.2, 5.0     # typed depth, and the deepest max_depth_m allowed
MIN_ALTITUDE_M, MAX_ALTITUDE_M = 0.5, 10.0
STEPS = 8                           # 45 degrees each: a yaw target under 180 ahead leaves the Cube one way round
HOLD_S = 3.0                        # at each heading
STEP_LIMIT_S = 20.0                 # a heading not reached by then is skipped, the circle goes on
YAW_REACHED = math.radians(8.0)
DEPTH_REACHED_M = 0.10
SURFACED_M = 0.25
FLOOR_SAMPLES = 10                  # a second of DVL altitudes: the median ignores a wild one
START_LIMIT_S = 10.0                # GUIDED, ARM and DISARM each
TELEMETRY_LIMIT_S = 3.0             # silence once it is moving
WAIT_LIMIT_S = 15.0                 # silence before it has started: waypoint_mission's own topics take a moment
DIVE_STAGES = {'DESCEND', 'SPIN', 'SURFACE'}
CHRIS_FORWARD_M = 1.0
FORWARD_REACHED_M = 0.25
FORWARD_LIMIT_S = 30.0
RESOURCE_COLORS = {1: 'RED', 2: 'GREEN', 3: 'BLUE'}

# Why a run ended early, small enough for the radio's status frame (status_codec.py carries it).
NOTE_NONE, NOTE_OPERATOR, NOTE_PILOT, NOTE_SAFETY, NOTE_TIMEOUT, NOTE_TELEMETRY = range(6)
NOTE_NO_NAV = 9                     # a route that never started: navigation was not running (radio_route.py)
# Not aborts: the turn was skipped, the run carries on and still ends COMPLETE.
NOTE_DEPTH_NOT_REACHED, NOTE_NO_BOTTOM, NOTE_FLOOR_TOO_NEAR = 6, 7, 8


def _number(value, low, high, what):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
            or not low <= value <= high):
        raise ValueError('%s must be %.1f to %.1f m' % (what, low, high))
    return round(float(value), 2)


def dive_target(depth_m=None, altitude_m=None, max_depth_m=None):
    """{'depth_m'} for a pool test, or {'altitude_m', 'max_depth_m'} above the floor. ValueError if not allowed."""
    if depth_m is not None:
        if altitude_m is not None or max_depth_m is not None:
            raise ValueError('Dive: a pool depth, or a height above the floor, not both')
        return dict(depth_m=_number(depth_m, MIN_DEPTH_M, MAX_DEPTH_M, 'Dive depth'))
    return dict(altitude_m=_number(altitude_m, MIN_ALTITUDE_M, MAX_ALTITUDE_M, 'Dive height above the floor'),
                max_depth_m=_number(max_depth_m, MIN_DEPTH_M, MAX_DEPTH_M, 'Dive max depth'))


def dive_spin_plan(dive):
    """The plan waypoint_mission is started with, for a dive where the sub is now."""
    return dict(kind='dive_spin', mode='surface', map='local', dive=dive_target(**dive))


def random_resource_request(run_id, randbelow=None):
    """Build the simulated RobotX two-colour request for Chris Simple Dive.

    The sequence is resource colour first and delivery-circle colour second.
    Both are independently RED, GREEN or BLUE.  A stable observation ID lets
    the competition outbox deduplicate repeated terminal publications.
    """
    if not isinstance(run_id, str) or not run_id:
        raise ValueError('Mission run ID required for the resource request')
    choose = secrets.randbelow if randbelow is None else randbelow
    colors = [choose(3) + 1, choose(3) + 1]
    if any(type(value) is not int or value not in RESOURCE_COLORS for value in colors):
        raise ValueError('Random colour source returned an invalid value')
    return dict(observation_id=('chris-simple-dive:' + run_id)[:128],
                resource_color=colors[0], delivery_circle_color=colors[1],
                sequence=[RESOURCE_COLORS[value] for value in colors])


def turn(a, b):
    """The signed angle from heading b to heading a, -pi to pi."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class DiveSpinRun(GuidedSurfaceRun):
    moving = set(DIVE_STAGES) | {'FORWARD'}

    def __init__(self, plan, run_id, now):
        dive = dive_target(**plan['dive'])
        super().__init__(dict(lat=0., lon=0., map='geographic', mode='surface'), run_id, now)
        self.plan = dict(plan, dive=dive)
        self.dive = dive
        self.forward_m = CHRIS_FORWARD_M if plan.get('operator_kind') == 'dive_here' else 0.0
        self.resource_request = (random_resource_request(run_id)
                                 if plan.get('operator_kind') == 'dive_here' else None)
        self.deepest = dive.get('depth_m', dive.get('max_depth_m'))     # below the start, at most
        self.limits = dict(GUIDED=START_LIMIT_S, ARM=START_LIMIT_S, DISARM=START_LIMIT_S,
                           FORWARD=FORWARD_LIMIT_S,
                           DESCEND=15. + 15. * self.deepest, SURFACE=20. + 15. * self.deepest)
        self.has_vehicle = False        # the Cube confirmed GUIDED for this run
        self.hold = None                # (north, east, down) where the dive starts: the spot it keeps
        self.floors = deque(maxlen=FLOOR_SAMPLES)   # the floor's "down", from DVL altitudes on this dive
        self.spin_step = 0              # 1..STEPS while turning
        self.step_since = now
        self.reached_at = None          # when this heading was first reached
        self.skipped = 0                # headings given up on
        self.silent_since = None
        self.note = NOTE_NONE
        self.kill_requested = False
        self.forward_target = None
        self.reason = 'Waiting for the Cube and the kill switch'

    @property
    def phase(self):
        return 'UNDERWATER' if self.stage in DIVE_STAGES else 'SURFACE_NAV'

    def fail(self, now, o, reason, note):
        """Anything but a pilot taking over: stop, and once the vehicle was ours, disarm and want the kill."""
        self.note = note
        self.kill_requested = self.has_vehicle
        self.abort(now, reason)
        return self.abort_actions(now, o)

    def abort_actions(self, now, o):
        if self.pilot_takeover:
            return []
        if self.has_vehicle and o.get('armed'):
            return self.command(now, 'disarm', interval=.5)
        if self.takeover_requested and o.get('mode') in (None, 4):
            return self.command(now, 'manual')
        return []

    def step(self, now, o):
        self.last = now
        if self.stage == 'COMPLETE':
            return []
        if self.stage == 'ABORTED':
            return self.abort_actions(now, o)
        if o.get('abort'):
            return self.fail(now, o, 'Stopped from the page', NOTE_OPERATOR)
        if o.get('failure'):
            return self.fail(now, o, o['failure'], NOTE_SAFETY)
        if self.stage in self.moving and (o.get('armed') is False or o.get('mode') not in (None, 4)):
            self.note = NOTE_PILOT
            self.abort(now, 'Pilot or failsafe took control', pilot=True)
            return []
        if not o.get('fresh'):
            self.silent_since = now if self.silent_since is None else self.silent_since
            limit = WAIT_LIMIT_S if self.stage == 'WAIT_SURFACE' else TELEMETRY_LIMIT_S
            if now - self.silent_since > limit:
                return self.fail(now, o, 'Cube telemetry or navigation status silent for %.0f s' % limit,
                                 NOTE_TELEMETRY)
            return []
        self.silent_since = None
        limit = self.limits.get(self.stage)
        return self.act(now, o, tuple(o['local']), o['yaw'], limit is not None and now - self.since > limit)

    def act(self, now, o, local, yaw, late):
        """One stage's decision on fresh telemetry. radio_route.py adds its own stages around these."""
        if self.stage == 'WAIT_SURFACE':
            self.enter('GUIDED', now, 'Taking GUIDED')
        elif self.stage == 'GUIDED':
            if o.get('mode') == 4:
                self.has_vehicle = True
                self.enter('ARM', now, 'Arming')
            elif late:
                return self.fail(now, o, 'The Cube did not take GUIDED', NOTE_TIMEOUT)
            else:
                return self.command(now, 'guided')
        elif self.stage == 'ARM':
            if o.get('mode') != 4:
                self.note = NOTE_PILOT
                self.abort(now, 'Pilot changed mode', pilot=True)
                return []
            if not o.get('armed'):
                if late:
                    return self.fail(now, o, 'The Cube did not arm', NOTE_TIMEOUT)
                return self.command(now, 'arm')
            self.start, self.surface_down = local, local[2]
            return self.begin(now, o, local, yaw)
        elif self.stage == 'FORWARD':
            self.target = self.forward_target
            if self.settled(now, math.dist(local, self.target) <= FORWARD_REACHED_M
                            and o.get('speed', math.inf) <= .15):
                return self.start_dive(now, self.forward_target, self.heading)
            if late:
                return self.fail(now, o, 'The 1 m forward leg did not finish in time', NOTE_TIMEOUT)
            return self.command(now, 'guided_orient', (*self.target, self.heading), interval=.1)
        elif self.stage == 'DESCEND':
            down = self.descent_target(local, o.get('altitude'))
            if down is None:
                self.note = NOTE_FLOOR_TOO_NEAR
                self.target = self.hold
                self.enter('SURFACE', now, 'The floor is nearer than %.1f m: surfacing without the turn'
                           % self.dive['altitude_m'])
                return self.command(now, 'guided_orient', (*self.hold, self.heading), interval=.1)
            self.target = (self.hold[0], self.hold[1], down)
            if self.settled(now, abs(local[2] - down) <= DEPTH_REACHED_M):
                if 'altitude_m' in self.dive and not self.floors:
                    self.note = NOTE_NO_BOTTOM
                    self.enter('SURFACE', now, 'No bottom lock down to %.1f m: surfacing without the turn'
                               % self.deepest)
                else:
                    self.next_heading(now)
            elif late:
                self.note = NOTE_DEPTH_NOT_REACHED
                self.enter('SURFACE', now, 'Depth not reached in time (the floor?): surfacing without the turn')
            return self.command(now, 'guided_orient', (*self.target, self.heading), interval=.1)
        elif self.stage == 'SPIN':
            wanted = self.heading + self.spin_step * 2 * math.pi / STEPS
            if self.reached_at is None and abs(turn(wanted, yaw)) <= YAW_REACHED:
                self.reached_at = now
            if self.reached_at is not None and now - self.reached_at >= HOLD_S:
                self.next_heading(now)
            elif self.reached_at is None and now - self.step_since > STEP_LIMIT_S:
                self.skipped += 1
                self.next_heading(now)
            return self.command(now, 'guided_orient', (*self.target, wanted), interval=.1)
        elif self.stage == 'SURFACE':
            self.target = self.hold
            if local[2] <= self.hold[2] + SURFACED_M or late:
                return self.surfaced(now, o, local, yaw)
            return self.command(now, 'guided_orient', (*self.target, self.heading), interval=.1)
        elif self.stage == 'DISARM':
            if o.get('armed') and not late:
                return self.command(now, 'disarm')
            self.kill_requested = True      # also what stops it if the disarm never showed
            self.enter('COMPLETE', now, 'Disarmed; thrusters killed until SA KILL, SB DISARM, release SA')
            return [('manual', None)]
        return []

    def begin(self, now, o, local, yaw):
        """Armed in GUIDED: Chris moves 1 m first; other dive variants dive here."""
        if self.forward_m:
            self.heading = yaw
            self.forward_target = (local[0] + self.forward_m * math.cos(yaw),
                                   local[1] + self.forward_m * math.sin(yaw), local[2])
            self.target = self.forward_target
            self.enter('FORWARD', now, 'Moving 1 m along the starting heading')
            return self.command(now, 'guided_orient', (*self.target, self.heading), interval=.1)
        return self.start_dive(now, local, yaw)

    def start_dive(self, now, local, yaw):
        self.hold, self.heading = local, yaw
        self.floors.clear()
        self.spin_step = 0
        self.enter('DESCEND', now, 'Descending')
        return []

    def surfaced(self, now, o, local, yaw):
        """Back up from the dive: disarm, and the kill."""
        self.enter('DISARM', now, 'Up: disarming')
        return self.command(now, 'disarm')

    def descent_target(self, local, altitude):
        """The "down" to hold: the typed depth, or altitude_m above the floor the DVL sees, never below
        max_depth_m. Without a floor yet it heads for max_depth_m looking for one. None: the floor is
        too near to be altitude_m above it."""
        if 'depth_m' in self.dive:
            return self.hold[2] + self.dive['depth_m']
        if altitude is not None:
            self.floors.append(local[2] + altitude)
        deepest = self.hold[2] + self.deepest
        if not self.floors:
            return deepest
        down = min(deepest, statistics.median(self.floors) - self.dive['altitude_m'])
        return None if down < self.hold[2] + MIN_DEPTH_M else down

    def next_heading(self, now):
        """The next 45 degrees, or up once the circle is closed."""
        if self.spin_step == STEPS:
            self.enter('SURFACE', now, 'Circle done: surfacing')
            return
        self.spin_step += 1
        self.step_since, self.reached_at = now, None
        self.enter('SPIN', now, 'Turning: heading %d of %d' % (self.spin_step, STEPS))

    def snapshot(self):
        s = super().snapshot()
        s.update(note=self.note, spin_step=self.spin_step, spin_steps=STEPS, headings_skipped=self.skipped,
                 kill_requested=self.kill_requested, dive=self.dive,
                 forward_m=self.forward_m, forward_target=self.forward_target)
        return s

