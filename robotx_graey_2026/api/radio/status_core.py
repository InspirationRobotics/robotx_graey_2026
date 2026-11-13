"""What Graey says about itself: every reading with its own age, blank once it is old.

The radio_status node feeds this from ROS; the laptop's simulator feeds it from a fake sub.
No ROS and no sockets here, so the same code runs on both and is tested on a laptop.

A value older than its limit is None - never the last one it had. On the page a blank reads
as "not arriving", which is the truth; a stale number looks exactly like a fresh one.
"""
import math
import statistics
import time
from collections import deque

from robotx_graey_2026.api.vision.stills import read_json, write_json

from . import status_codec as codec

CUBE_S = 3.0        # the Cube's HEARTBEAT is once a second
SENSOR_S = 2.0      # DVL and VN-100 readings
CONTROL_S = 1.5     # kill_switch says how it is five times a second
MISSION_S = 3.0     # waypoint_mission says its stage ten times a second while it runs
ENDED = ('COMPLETE', 'ABORTED')
RADIO_KINDS = ('dive_spin', 'radio_route')    # the plans the radio page starts; any other is "another mission"
MIN_BATTERY_V = 5.0     # below this nothing is measuring the pack (bench, Nov 12: the Cube reported 0.66 V)
# DEPTH is the water pressure sensor against the lowest pressure seen since power-on - the air on the
# bench, or the surface. Not the Cube's altitude: on the bench on Nov 12 its GLOBAL_POSITION_INT said
# +20 m above home (its EKF origin's height), which as a depth is -20 m.
HPA_PER_M = 100.5       # a metre of sea water: 1025 kg/m3 x 9.80665 m/s2 = 10052 Pa (fresh water: 2.5 % less)
PRESSURE_OK = (800.0, 2500.0)       # hPa a real reading lies between; anything else is a glitch
SURFACE = '/dev/shm/graey_surface_hpa.json'     # so a restart of this node under water keeps the surface it knew


class StatusCore:
    def __init__(self, record, clock=time.monotonic, surface_file=None, missions=None):
        self.record, self.clock = record, clock
        self.missions = missions        # mission_relay.MissionRelay, or None where no mission can be started
        self.inputs = {}                # name -> (when, value)
        self.seq = 0
        self.cmd_seq, self.cmd_result = 0, codec.RESULT_NONE
        self.ended = None               # (stage, note, point, points) of the last radio mission, until the next start
        self.refused = codec.NOTE_NONE  # why the last start or stop was refused
        self.recent_hpa = deque(maxlen=10)      # a second of readings: their median ignores one wild sample
        self.surface_file = surface_file
        self.surface_hpa = (read_json(surface_file) or {}).get('hpa') if surface_file else None

    def put(self, name, value):
        self.inputs[name] = (self.clock(), value)

    def age(self, name):
        got = self.inputs.get(name)
        return None if got is None else self.clock() - got[0]

    def get(self, name, limit):
        got = self.inputs.get(name)
        return got[1] if got is not None and self.clock() - got[0] <= limit else None

    # ------------------------------------------------------------ the Cube's own messages
    def on_cube(self, kind, m):
        """A MAVLink message from the autopilot itself (system 1, component 1)."""
        if kind == 'HEARTBEAT':
            self.put('heartbeat', (bool(m.base_mode & 128), int(m.custom_mode)))
        elif kind == 'ATTITUDE':
            self.put('attitude', (m.roll, m.pitch, m.yaw))
        elif kind == 'GLOBAL_POSITION_INT':
            fix = bool(m.lat or m.lon)                  # 0, 0 is "no position", not the Gulf of Guinea
            self.put('position', (m.lat / 1e7 if fix else None, m.lon / 1e7 if fix else None,
                                  m.relative_alt / 1000.0))
        elif kind == 'GPS_RAW_INT':
            self.put('gps', (int(m.fix_type), None if m.satellites_visible == 255 else int(m.satellites_visible)))
        elif kind == 'SYS_STATUS':
            if MIN_BATTERY_V * 1000 <= m.voltage_battery < 65535:
                self.put('battery', m.voltage_battery / 1000.0)
        elif kind == 'SCALED_PRESSURE3':                # the water pressure sensor (docs/depth-pressure-source)
            self.on_pressure(float(m.press_abs))

    def on_pressure(self, hpa):
        if not PRESSURE_OK[0] < hpa < PRESSURE_OK[1]:
            return
        self.put('pressure', hpa)
        self.recent_hpa.append(hpa)
        if len(self.recent_hpa) == self.recent_hpa.maxlen:
            steady = statistics.median(self.recent_hpa)
            if self.surface_hpa is None or steady < self.surface_hpa - 0.2:    # 0.2 hPa is 2 mm of water
                self.surface_hpa = steady
                if self.surface_file:
                    try:
                        write_json(self.surface_file, dict(hpa=steady))
                    except OSError:
                        pass                    # only the memory of it across a restart is lost

    def on_command(self, payload):
        """A COMMAND frame off the radio. The laptop repeats it until STATUS echoes its seq."""
        cmd = codec.unpack_command(payload)
        if cmd is None or cmd[0] == self.cmd_seq:
            return False
        seq, command, argument = cmd
        if command == codec.CMD_RECORD:
            self.record.set(*codec.record_wanted(argument))
            ok = True
        elif command == codec.CMD_MISSION_START:
            ok = self.start_mission(codec.unpack_mission_start(payload))
        elif command == codec.CMD_MISSION_STOP:
            ok = self.stop_mission()
        elif command == codec.CMD_NAV_START:
            ok = self.start_navigation()
        else:
            ok = command == codec.CMD_PING
        self.cmd_seq, self.cmd_result = seq, codec.RESULT_OK if ok else codec.RESULT_REFUSED
        return True

    # ------------------------------------------------------------ the mission (radio_route.py)
    def start_mission(self, mission):
        """Ask gui_node to start a dive, route or full run. True if it did; if not, the note says why."""
        self.ended = None               # the last run's ending gives way to this one's answer
        if self.missions is None or not isinstance(mission, dict):
            self.refused = codec.NOTE_REFUSED
            return False
        ok, self.refused, _ = self.missions.start(mission)
        return ok

    def start_navigation(self):
        self.ended = None
        if self.missions is None:
            self.refused = codec.NOTE_REFUSED
            return False
        ok, self.refused = self.missions.start_navigation()
        return ok

    def stop_mission(self):
        running = self.get('mission', MISSION_S) or {}
        if self.missions is None or not running.get('run_id') or running.get('stage') in ENDED:
            self.refused = codec.NOTE_REFUSED       # nothing is running to stop
            return False
        ok, self.refused = self.missions.stop(running['run_id'])
        return ok

    def mission(self):
        """(stage, note, point, points) for the frame. A radio mission's ending is remembered until the
        next start: the page was most likely out of reach, under water, at the moment it ended."""
        m = self.get('mission', MISSION_S)
        if m is not None:
            if (m.get('plan') or {}).get('kind') not in RADIO_KINDS:
                return (None if m.get('stage') in ENDED else codec.MISSION_OTHER), codec.NOTE_NONE, None, None
            now = (m.get('stage'), int(m.get('note') or 0), m.get('route_leg'), m.get('route_legs'))
            if now[0] in ENDED:
                self.ended = now
            return now
        return self.ended or (None, self.refused, None, None)

    # ------------------------------------------------------------ what is true now
    def status(self):
        hb, att = self.get('heartbeat', CUBE_S), self.get('attitude', CUBE_S)
        pos, gps = self.get('position', CUBE_S), self.get('gps', CUBE_S)
        ctl = self.get('control', CONTROL_S)
        vel = self.get('dvl_velocity', SENSOR_S)
        nav = self.get('nav', SENSOR_S)
        out = dict(
            nav_fresh=nav is not None, nav_on=bool(nav and nav.get('enabled') and nav.get('configuration_verified')),
            nav_geo=bool(nav and (nav.get('map_transform') or {}).get('ready')),
            cube_fresh=hb is not None, armed=bool(hb and hb[0]), mode=hb[1] if hb else None,
            ks_fresh=ctl is not None, inhibit_known=ctl is not None,
            inhibit=ctl.get('inhibit') if ctl else None,
            esc_power=bool(ctl and ctl.get('esc_commanded_on')), sa_kill=bool(ctl and ctl.get('sa_active')),
            rf_alive=bool(ctl and ctl.get('rf_alive')),
            source_tether=bool(ctl and ctl.get('source') == 'TETHER'),
            autonomous=bool(ctl and (ctl.get('autonomous') or ctl.get('mission_claimed'))),
            depth_m=self.depth(), cube_rel_alt_m=pos[2] if pos else None,
            lat=pos[0] if pos else None, lon=pos[1] if pos else None,
            cube_heading_deg=math.degrees(att[2]) % 360 if att else None,
            roll_deg=math.degrees(att[0]) if att else None, pitch_deg=math.degrees(att[1]) if att else None,
            gps_fix=gps[0] if gps else None, gps_sats=gps[1] if gps else None,
            battery_v=self.get('battery', CUBE_S),
            dvl_fresh=vel is not None, dvl_valid=self.get('dvl_valid', SENSOR_S) is True,
            dvl_velocity=vel, alt_m=self.get('dvl_altitude', SENSOR_S),
            vn_fresh=self.get('vn_heading', SENSOR_S) is not None,
            vn_heading_deg=self.get('vn_heading', SENSOR_S),
            dvl_age_s=self.age('dvl_velocity'), vn_age_s=self.age('vn_heading'),
            cube_age_s=self.age('heartbeat'),
            pressure_hpa=self.get('pressure', CUBE_S),
            cmd_seq=self.cmd_seq, cmd_result=self.cmd_result)
        out['mission_stage'], out['mission_note'], out['mission_point'], out['mission_points'] = self.mission()
        out.update(self.record.snapshot())
        return out

    def depth(self):
        hpa = self.get('pressure', CUBE_S)
        if hpa is None or self.surface_hpa is None:
            return None
        return max(0.0, (hpa - self.surface_hpa) / HPA_PER_M)

    def frame(self):
        """The next STATUS payload for the radio."""
        self.seq = (self.seq + 1) & 255
        return codec.pack_status(self.status(), self.seq)

    def nav(self):
        """Where the sub is, for a still's line in frames.jsonl and for nav.jsonl. Blanks stay blank."""
        s = self.status()

        def r(value, places):
            return None if value is None else round(value, places)
        return dict(depth_m=r(s['depth_m'], 2), alt_m=r(s['alt_m'], 2), pressure_hpa=r(s['pressure_hpa'], 1),
                    surface_hpa=r(self.surface_hpa, 1), cube_rel_alt_m=r(s['cube_rel_alt_m'], 2),
                    heading_deg=r(s['vn_heading_deg'], 1), cube_heading_deg=r(s['cube_heading_deg'], 1),
                    roll_deg=r(s['roll_deg'], 1), pitch_deg=r(s['pitch_deg'], 1),
                    lat=r(s['lat'], 7), lon=r(s['lon'], 7),
                    dvl_velocity=[round(v, 3) for v in s['dvl_velocity']] if s['dvl_velocity'] else None,
                    dvl_valid=s['dvl_valid'] if s['dvl_fresh'] else None,
                    mode=s['mode'], armed=s['armed'] if s['cube_fresh'] else None)
