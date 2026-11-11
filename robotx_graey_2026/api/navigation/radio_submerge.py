"""Radio submerge test: how far under water the RFD antenna can go before the boat goes quiet.

Dive 1 ft below the starting surface level and come back, 3 times, at 2 cm/s, then stop and
disarm. Receive-only: the boat broadcasts numbered text lines (radio_test/boat_broadcast.py,
TUNNEL payload_type 33022 from the boat's rfd_link_node, system 42) and every line the sub
hears is logged with the depth it was heard at. 2 cm/s against the boat's default 2 lines/s
gives one line per centimetre.

Pure decisions and log rows, like the 5-2-5 SimpleSequenceRun it follows; the
radio_submerge_mission runner (built on the unchanged WaypointMission) does the MAVLink and the
file. A timeout 180 s after arming disarms, then switches to MANUAL.
"""
import csv
import math
import re
from datetime import datetime, timezone
from pathlib import Path

from .guided_surface_mission import GuidedSurfaceRun

DIVE_M = .3048              # 1 ft below the starting surface level
CYCLES = 3
RATE_MPS = .02              # 2 cm/s, down and up
DONE_TOLERANCE_M = .03      # a leg is done when Cube depth is this close to its end for 1 s
TIMEOUT_S = 180.            # from arming; expected run is about 100 s
PRESSURE_MAX_AGE_S = .5
MODE_GUIDED, MODE_MANUAL = 4, 19
GRAVITY = 9.80665

BOAT_SYSID = 42             # rfd_link_node source_system on the boat
BOAT_TEXT_TYPE = 33022      # crusader_link PAYLOAD_TEST: a text line nobody acts on
BOAT_TEXT = 'the boat is sending this msg'

LEGS = tuple('%s_%d' % (d, i) for i in range(1, CYCLES+1) for d in ('DOWN', 'UP'))
LEG_TEXT = {leg: ('Down 1 ft' if leg.startswith('DOWN') else 'Up to the starting level')
            + ' (%s of %d)' % (leg[-1], CYCLES) for leg in LEGS}


class RadioSubmergeRun(GuidedSurfaceRun):
    moving = set(LEGS)

    def __init__(self, plan, run_id, now):
        super().__init__(dict(lat=0., lon=0., map='geographic', mode='surface'), run_id, now)
        self.plan = dict(kind='radio_submerge', mode='surface', map='local', dive_m=DIVE_M,
                         cycles=CYCLES, rate_mps=RATE_MPS, timeout_s=TIMEOUT_S)
        self.enter('GUIDED', now, 'Starting radio submerge test')
        self.armed_at = None
        self.start_pressure_hpa = None
        self.radio_status = None        # log summary, set by the runner each tick
        self.timed_out = self.disarm_confirmed = False

    @property
    def phase(self):
        return 'UNDERWATER' if self.stage in LEGS else 'SURFACE_NAV'

    def leg_target(self, now):
        """(target z, end z, ramp finished) for the current leg; NED z is positive down."""
        top, bottom = self.surface_down, self.surface_down+DIVE_M
        start, end = (top, bottom) if self.stage.startswith('DOWN') else (bottom, top)
        moved = RATE_MPS*max(0., now-self.since)
        if moved >= DIVE_M:
            return end, end, True
        return start+math.copysign(moved, end-start), end, False

    def step(self, now, o):
        self.last = now
        if self.stage == 'COMPLETE':
            return []
        if self.stage == 'TIMEOUT':
            return self.timeout_actions(now, o)
        if self.stage == 'ABORTED':
            return self.abort_actions(now, o)
        # Checked first: stale telemetry or a stuck leg must not hold off the timeout.
        if self.armed_at is not None and now-self.armed_at >= TIMEOUT_S:
            self.timed_out = True
            self.enter('TIMEOUT', now, 'Timed out %g s after arming: disarming, then MANUAL' % TIMEOUT_S)
            return self.timeout_actions(now, o)
        if o.get('abort') or o.get('failure'):
            self.abort(now, o.get('failure') or 'Operator aborted')
            return self.abort_actions(now, o)
        if self.stage in self.moving and (o.get('armed') is False or o.get('mode') not in (None, MODE_GUIDED)):
            self.abort(now, 'Pilot/failsafe took control', pilot=True)
            return []
        if not o.get('fresh'):
            self.reason = 'Waiting for Cube control telemetry'
            return []
        local = tuple(o['local'])
        if self.stage == 'GUIDED':
            # Relative depth is measured from the pressure at arming; no reading, no test.
            if o.get('pressure_hpa') is None:
                self.reason = 'Waiting for a fresh pressure reading before arming'
                return []
            if o.get('mode') == MODE_GUIDED:
                self.enter('ARM', now, 'Arming')
            else:
                return self.command(now, 'guided')
        elif self.stage == 'ARM':
            if o.get('mode') != MODE_GUIDED:
                self.abort(now, 'Pilot changed mode', pilot=True)
                return []
            if not o.get('armed'):
                if self.armed_at is not None:
                    self.abort(now, 'Disarmed before the first dive', pilot=True)
                    return []
                return self.command(now, 'arm')
            if self.armed_at is None:
                # The timeout clock and the starting surface level both start at arming.
                self.armed_at = now
                self.heading = o['yaw']
                self.start = local
                self.surface_down = local[2]
            if o.get('pressure_hpa') is None:
                self.reason = 'Armed; holding for a fresh pressure reading to start from'
                return self.command(now, 'guided_orient', (*self.start, self.heading), interval=.1)
            self.start_pressure_hpa = o['pressure_hpa']
            self.enter(LEGS[0], now, LEG_TEXT[LEGS[0]])
        elif self.stage in LEGS:
            z, end, ramped = self.leg_target(now)
            self.target = (self.start[0], self.start[1], z)
            if self.settled(now, ramped and abs(local[2]-end) <= DONE_TOLERANCE_M):
                i = LEGS.index(self.stage)
                if i+1 < len(LEGS):
                    self.enter(LEGS[i+1], now, LEG_TEXT[LEGS[i+1]])
                else:
                    self.enter('DISARM', now, 'Three dives done; disarming')
            return self.command(now, 'guided_orient', (*self.target, self.heading), interval=.1)
        elif self.stage == 'DISARM':
            if o.get('armed'):
                return self.command(now, 'disarm')
            self.enter('COMPLETE', now, 'Three dives done; disarmed')
            return [('manual', None)]
        return []

    def timeout_actions(self, now, o):
        """Disarm until the Cube says disarmed, then MANUAL until it says MANUAL."""
        if o.get('armed') is not False:
            return self.command(now, 'disarm', interval=.5)
        if not self.disarm_confirmed:
            # Ask for MANUAL straight away, not on the disarm's resend timer.
            self.disarm_confirmed, self.last_command = True, -math.inf
            self.reason = 'Timed out: disarmed; switching to MANUAL'
        if o.get('mode') != MODE_MANUAL:
            return self.command(now, 'manual')
        self.pilot_takeover = self.takeover_requested = False
        self.enter('ABORTED', now, 'Timed out %g s after arming: disarmed and in MANUAL' % TIMEOUT_S)
        return []

    def snapshot(self):
        s = super().snapshot()
        s.update(timed_out=self.timed_out, start_pressure_hpa=self.start_pressure_hpa, radio=self.radio_status,
                 seconds_since_arming=None if self.armed_at is None else self.last-self.armed_at)
        return s


class RadioLog:
    """Rows of the radio test CSV. Pure: the adapter feeds samples and writes what rows() returns.

    One row per boat line heard (kind=heard), and a depth-only row every 0.5 s (kind=depth) so
    the silent stretches still have depths. Values that are not known are left blank.
    """
    FIELDS = ('time_utc', 't_s', 'kind', 'stage', 'msg_number', 'msg_text', 'raw_depth_m',
              'relative_depth_m', 'cube_z_m', 'cube_relative_m', 'press_abs_hpa')
    DEPTH_PERIOD_S = .5

    def __init__(self, start, density=1000., wall=None):
        self.t0 = start
        self.density = density
        self.wall = wall or (lambda: datetime.now(timezone.utc))
        self.pressure = None            # (monotonic, hPa)
        self.pending = []
        self.received = 0
        self.last_number = None
        self.last_heard_at = None
        self.deepest_heard_m = None     # deepest relative depth a line was heard at
        self.next_depth_row = start

    def metres(self, hpa):
        return hpa*100./(self.density*GRAVITY)

    def pressure_sample(self, hpa, now):
        if isinstance(hpa, (int, float)) and math.isfinite(hpa) and hpa > 0:
            self.pressure = (now, float(hpa))

    def pressure_hpa(self, now):
        if self.pressure is None or not 0 <= now-self.pressure[0] < PRESSURE_MAX_AGE_S:
            return None
        return self.pressure[1]

    def _row(self, now, kind, run, cube_z, number=None, text=None):
        hpa = self.pressure_hpa(now)
        start = run.start_pressure_hpa
        relative = None if hpa is None or start is None else self.metres(hpa-start)
        cube_relative = None if cube_z is None or run.surface_down is None else cube_z-run.surface_down
        return dict(time_utc=self.wall().isoformat(timespec='milliseconds'), t_s=round(now-self.t0, 3),
                    kind=kind, stage=run.stage, msg_number=number, msg_text=text,
                    raw_depth_m=None if hpa is None else round(self.metres(hpa), 4),
                    relative_depth_m=None if relative is None else round(relative, 4),
                    cube_z_m=None if cube_z is None else round(cube_z, 4),
                    cube_relative_m=None if cube_relative is None else round(cube_relative, 4),
                    press_abs_hpa=hpa)

    def boat_line(self, payload, now, run, cube_z):
        """A TUNNEL payload from the boat; ignored unless it is one of the test's lines."""
        text = bytes(payload).decode('ascii', 'replace').rstrip('\x00')
        head, _, number = text.rpartition(' ')
        if head != BOAT_TEXT or not number.isdigit():
            return None
        row = self._row(now, 'heard', run, cube_z, int(number), text)
        self.received += 1
        self.last_number, self.last_heard_at = int(number), now
        if row['relative_depth_m'] is not None:
            self.deepest_heard_m = max(row['relative_depth_m'], self.deepest_heard_m or -math.inf)
        self.pending.append(row)
        return row

    def tick(self, now, run, cube_z):
        if now >= self.next_depth_row:
            self.next_depth_row = now+self.DEPTH_PERIOD_S
            self.pending.append(self._row(now, 'depth', run, cube_z))

    def rows(self):
        out, self.pending = self.pending, []
        return out

    def summary(self, now):
        return dict(received=self.received, last_number=self.last_number,
                    last_heard_s_ago=None if self.last_heard_at is None else round(now-self.last_heard_at, 1),
                    deepest_heard_relative_m=self.deepest_heard_m)


def tap_drain(link, tap, on_error):
    """Show `tap` every frame the mission's own handler is given, then pass it on unchanged.
    A tap failure goes to `on_error`; it can never stop the mission seeing a frame."""
    drain = link.drain

    def tapped(handler):
        def both(kind, message):
            try:
                tap(kind, message)
            except Exception as exc:  # noqa: BLE001 - logging must not take down the mission
                on_error(exc)
            handler(kind, message)
        return drain(both)

    link.drain = tapped


def radio_log_path(folder, run_id):
    """The run's CSV beside its waypoint-<run_id>.json result. Run IDs are uuid4 hex, so a
    query string can never name a file outside the folder."""
    if not isinstance(run_id, str) or not re.fullmatch('[0-9a-f]{32}', run_id):
        raise ValueError('Radio test run ID must be 32 lowercase hex characters')
    return Path(folder)/('waypoint-%s-radio.csv' % run_id)


def write_radio_header(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        csv.writer(f).writerow(RadioLog.FIELDS)


def append_radio_rows(path, rows):
    with path.open('a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=RadioLog.FIELDS)
        for row in rows:
            writer.writerow({k: '' if v is None else v for k, v in row.items()})


def copy_radio_rows(src, dst, cursor=None, since_wall=None):
    """Append the rows of a radio test CSV after `cursor` to `dst` (header first if new).

    For the Record page's RFD data source. On the first copy (cursor None) rows logged before
    `since_wall` (epoch seconds, when the recording session started) are skipped: a session
    keeps only what was logged while it was recording. Returns (new cursor, rows copied).
    """
    start = cursor or 0
    rows, _ = read_radio_rows(src, start, limit=1 << 30)
    skip = 0
    if cursor is None and since_wall is not None:
        for row in rows:
            try:
                if datetime.fromisoformat(row['time_utc']).timestamp() >= since_wall:
                    break
            except (KeyError, TypeError, ValueError):
                break               # an unreadable time is kept, never silently dropped
            skip += 1
    keep = rows[skip:]
    if keep:
        if not Path(dst).exists():
            write_radio_header(Path(dst))
        append_radio_rows(Path(dst), keep)
    return start+len(rows), len(keep)


def read_radio_rows(path, after=0, limit=500):
    """(rows after the first `after`, at most `limit` of them; total rows in the file).
    The GUI asks again from where it got to, so rows it missed while offline still arrive."""
    if isinstance(after, bool) or not isinstance(after, int) or after < 0:
        raise ValueError('after must be a row count')
    with Path(path).open(newline='', encoding='utf-8') as f:
        text = f.read()
    # The mission may be mid-append: leave a final line without its newline for next time.
    text = text[:text.rfind('\n')+1]
    rows = list(csv.DictReader(text.splitlines()))
    return rows[after:after+limit], len(rows)
