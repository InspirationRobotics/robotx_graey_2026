"""Read-only actual telemetry -> competition fields. Missing data suppresses reports."""
import math

from ..gui.navigation_view import NavigationView
from . import competition_codec as rx


class CompetitionTelemetry:
    def __init__(self, clock):
        self.clock = clock
        self.view = NavigationView(clock=clock)
        self.hae = self.hae_at = self.gps_stamp = None
        self.task = rx.TASK_NONE
        self.task_at = None
        self.reason = 'waiting for measured telemetry'

    def consume(self, kind, message):
        session = self.view.session
        self.view.consume(kind, message)
        if self.view.session != session:
            self.hae = self.hae_at = self.gps_stamp = None
        if kind == 'GPS_RAW_INT' and message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
            # The MAVLink 2 extension defaults to 0 when omitted: do not invent HAE.
            alt = getattr(message, 'alt_ellipsoid', 0)
            stamp = message.time_usec
            if message.fix_type < 3 or not alt or not stamp:
                self.hae = self.hae_at = None
            if stamp != self.gps_stamp:
                self.gps_stamp = stamp
                self.hae = alt/1000 if message.fix_type >= 3 and alt != 0 and stamp else None
                self.hae_at = self.clock() if self.hae is not None else None

    def put(self, name, data):
        if not isinstance(data, dict):
            raise ValueError('status must be a JSON object')
        if name == 'bridge':
            stamp = data.get('published_monotonic')
            if type(stamp) not in (int,float) or not 0 <= self.clock()-stamp < .5:
                raise ValueError('stale navigation bridge status')
        self.view.put(name, data)

    def set_task(self, name):
        values = {v:k for k,v in rx.RX_TASK.items() if k != 0}
        name = str(name).strip().upper()
        if name not in values:
            raise ValueError('current_task must be a known TASK_* enum name')
        self.task = values[name]
        self.task_at = self.clock()

    def update(self, sender, now):
        snapshot = self.view.snapshot()
        streams = snapshot['streams']
        bridge = streams.get('bridge', {})
        # Refresh from the original sample timestamp, never from a stale cached phase.
        if bridge.get('age', math.inf) < .5:
            surface = bridge['data'].get('surfaced')
            if type(surface) is bool:
                sender.set_surface(surface, now-bridge['age'])
            else:
                sender.surface = None
        else:
            sender.surface = None
        try:
            if bridge.get('age', math.inf) >= .5:
                raise ValueError('navigation bridge stale')
            b = bridge['data']
            if b.get('navigation_valid') is not True:
                raise ValueError('navigation invalid')
            cube = snapshot['cube_estimate']
            if not cube['valid'] or cube['heading_deg'] is None:
                raise ValueError('geographic position/heading unavailable')
            control = streams.get('control', {})
            if control.get('age', math.inf) >= 1.5:
                raise ValueError('control status stale')
            c = control['data']
            if c.get('vehicle_fresh') is not True:
                raise ValueError('Cube control telemetry stale')
            if any(type(c.get(k)) is not bool for k in ('sa_active','esc_commanded_on','autonomous','armed')):
                raise ValueError('control state incomplete')
            state = (rx.STATE_KILLED if c['sa_active'] or not c['esc_commanded_on']
                     else rx.STATE_AUTO if c['armed'] and (c['autonomous'] or c.get('mission_claimed') is True) else rx.STATE_MANUAL)
            if self.hae_at is None or not 0 <= now-self.hae_at < 1.5:
                raise ValueError('fresh GPS ellipsoid altitude unavailable')
            attitude = streams.get('attitude', {})
            local = streams.get('local', {})
            if attitude.get('age', math.inf) >= .5 or local.get('age', math.inf) >= .5:
                raise ValueError('attitude/velocity stale')
            roll,pitch,_ = attitude['data']['rpy']
            velocity = local['data']['velocity']
            sender.update_status(now=now,depth_m=b['depth_m'],robot_state=state,
                                 lat=cube['lat'],lon=cube['lon'],hdg_deg=cube['heading_deg'],
                                 spd_mps=math.hypot(*velocity[:2]),roll_deg=roll,pitch_deg=pitch,
                                 alt_m=self.hae,task=self.task if self.task_at is not None
                                     and 0<=now-self.task_at<2 and c['armed'] and (c['autonomous'] or c.get('mission_claimed') is True) else rx.TASK_NONE)
            self.reason = None
        except (ValueError,TypeError,KeyError,IndexError) as exc:
            sender.status_at = None  # stale valid heartbeat must not survive a fresh fault
            self.reason = str(exc)

