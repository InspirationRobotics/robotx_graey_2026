"""Graey's status over the RFD900, in one small frame, and the laptop's record command.

Chris, 12 Nov 2026, after the marina test: WiFi did not reach the sub, and the GUI, QGC
and ssh all rode on WiFi, so the dock was blind. Everything the operator needs now fits
in ONE MAVLink TUNNEL that the Cube forwards to the radio, twice a second (about
1 kbit/s). One frame, so the page works whatever the Cube's own stream rates are.

    STATUS   33280  Graey -> everyone   50 bytes (46, then the mission's stage, note, point, points)
    COMMAND  33281  laptop -> Graey     4 bytes, repeated until STATUS echoes its seq; a mission
                                        start adds its dive and up to 15 points (MISSION_START)

Pure: no ROS, no sockets, no pymavlink. The laptop page (tools/station) imports this
same file, so the two ends cannot drift.

BLANKS OVER GUESSES. Every value has an "unknown" code and decodes to None; a field
that is not known is never sent as zero.
"""
import struct

PAYLOAD_STATUS = 33280          # 0x8200: clear of boat_link (0x8000-0x8008), RXL (0x80xx) and TEST (0x80FE)
PAYLOAD_COMMAND = 33281
VERSION = 1

# version, seq, flags, mode, inhibit, depth cm, DVL altitude cm, VN heading cdeg, Cube heading cdeg,
# roll deg, pitch deg, DVL forward/right/down cm/s, DVL age ds, VN age ds, Cube age ds, battery cV,
# lat E7, lon E7, GPS fix, GPS sats, stills saved, sonar sweeps saved, disk free GiB, command seq, result
_STATUS = struct.Struct('<BBIBBhhHHbbhhhBBBHiiBBHHBBB')
_MISSION = struct.Struct('<BB')     # after _STATUS since the dive-and-spin mission (Nov 12): stage, note
_ROUTE = struct.Struct('<BB')       # after _MISSION since the radio routes (Nov 12): point now (1..), points
_COMMAND = struct.Struct('<BBBB')   # version, seq (1-255), command, argument
# CMD_MISSION_START after _COMMAND: depth or height above the floor in dm (0 = none), max depth dm,
# point count; then each point as lat, lon E7. 4 + 4 + 15 x 8 = 128, a whole TUNNEL.
_START = struct.Struct('<BBBB')     # depth dm, altitude dm, max depth dm, points
_POINT = struct.Struct('<ii')
MAX_POINTS = 15
STATUS_SIZE, COMMAND_SIZE = _STATUS.size, _COMMAND.size     # a 46- or 48-byte STATUS still decodes

FLAGS = ('armed', 'esc_power', 'sa_kill', 'rf_alive', 'ks_fresh', 'cube_fresh', 'dvl_valid', 'dvl_fresh',
         'vn_fresh', 'cam_service', 'cam_down', 'cam_front', 'sonar_service', 'sonar_pinging',
         'rec_cam_want', 'rec_cam_on', 'rec_sonar_want', 'rec_sonar_on', 'rec_cam_composite',
         'source_tether', 'autonomous', 'disk_low',
         'nav_fresh', 'nav_on', 'nav_geo')     # the nav bridge reporting; its output on; lat/lon map ready
INHIBITS = {None: 0, 'STARTUP': 1, 'SOURCE_CHANGED': 2, 'RF_LOST': 3, 'FC_LOST': 4, 'TETHER_LOST': 5,
            'SA_KILL': 6, 'MISSION_DONE': 7}    # kill_switch.inhibit; anything else is sent as unknown
_INHIBIT_NAMES = {v: k for k, v in INHIBITS.items()}

CMD_RECORD, CMD_PING, CMD_MISSION_START, CMD_MISSION_STOP, CMD_NAV_START = 1, 2, 3, 4, 5
REC_CAMERA, REC_SONAR, REC_SET_CAMERA, REC_SET_SONAR = 1, 2, 16, 32   # CMD_RECORD argument bits
KINDS = ('dive', 'route', 'full')   # CMD_MISSION_START's argument is 1 + the index (radio_route.py)
DM_PER_M = 10                       # depths and heights in the start command: tenths of a metre
RESULT_NONE, RESULT_OK, RESULT_REFUSED = 0, 1, 2

# The mission's stage as waypoint_mission reports it (dive_spin.py), 1-9 in the frame; 0 is "none".
MISSION_STAGES = ('WAIT_SURFACE', 'GUIDED', 'ARM', 'DESCEND', 'SPIN', 'SURFACE', 'DISARM', 'COMPLETE', 'ABORTED',
                  'ORIENT', 'TRANSIT')      # the last two: radio_route.py's surface legs
MISSION_OTHER = 'OTHER'             # a mission is running that the radio page did not start (the GUI's own)
_STAGE_CODES = {None: 0, MISSION_OTHER: len(MISSION_STAGES) + 1,
                **{name: i + 1 for i, name in enumerate(MISSION_STAGES)}}
_STAGE_NAMES = {v: k for k, v in _STAGE_CODES.items()}
# The mission's note: 0 nothing to say, 1-9 why a run ended early (dive_spin.NOTE_*), and from 10 why
# a start or a stop asked for over the radio was refused.
NOTE_NONE = 0
NOTE_REFUSED_KILLED, NOTE_REFUSED_BUSY, NOTE_REFUSED_DEPTH, NOTE_REFUSED_NO_GUI, NOTE_REFUSED = 10, 11, 12, 13, 14
NOTE_REFUSED_NAV, NOTE_REFUSED_ROUTE = 15, 16   # navigation not running; a route point not allowed
NOTE_REFUSED_MODE, NOTE_REFUSED_JOB = 17, 18    # Start navigation: not in MANUAL/STABILIZE; already starting

_U8, _I8, _U16, _I16, _I32 = 255, -128, 65535, -32768, 2147483647   # the "unknown" codes
ALT_NO_LOCK = -1                # DVL altitude: it answers but has no bottom lock (dvl_node's own -1)


def _fit(value, scale, low, high, unknown):
    """A number as a scaled integer clamped to its field, or the unknown code for None/NaN."""
    if value is None or value != value:
        return unknown
    return int(max(low, min(high, round(value * scale))))


def _known(raw, unknown, scale):
    return None if raw == unknown else raw / scale


def _flag_bits(status):
    return sum(1 << i for i, name in enumerate(FLAGS) if status.get(name))


def pack_status(status, seq):
    """status: a dict of the names unpack_status returns. Missing or None = unknown."""
    g = status.get
    alt = g('alt_m')
    return _STATUS.pack(
        VERSION, seq & 255, _flag_bits(status),
        _fit(g('mode'), 1, 0, 254, _U8),
        INHIBITS.get(g('inhibit'), _U8) if g('inhibit_known', True) else _U8,
        _fit(g('depth_m'), 100, -32767, 32767, _I16),
        ALT_NO_LOCK if alt is not None and alt < 0 else _fit(alt, 100, 0, 32767, _I16),
        _fit(None if g('vn_heading_deg') is None else g('vn_heading_deg') % 360, 100, 0, 35999, _U16),
        _fit(None if g('cube_heading_deg') is None else g('cube_heading_deg') % 360, 100, 0, 35999, _U16),
        _fit(g('roll_deg'), 1, -127, 127, _I8), _fit(g('pitch_deg'), 1, -127, 127, _I8),
        *(_fit(v, 100, -32767, 32767, _I16) for v in (g('dvl_velocity') or (None, None, None))),
        _fit(g('dvl_age_s'), 10, 0, 254, _U8), _fit(g('vn_age_s'), 10, 0, 254, _U8),
        _fit(g('cube_age_s'), 10, 0, 254, _U8),
        _fit(g('battery_v'), 100, 0, 65534, _U16),
        _fit(g('lat'), 1e7, -900000000, 900000000, _I32), _fit(g('lon'), 1e7, -1800000000, 1800000000, _I32),
        _fit(g('gps_fix'), 1, 0, 254, _U8), _fit(g('gps_sats'), 1, 0, 254, _U8),
        (g('rec_cam_count') or 0) & 65535, (g('rec_sonar_count') or 0) & 65535,
        _fit(g('disk_free_gib'), 1, 0, 254, _U8),
        (g('cmd_seq') or 0) & 255, (g('cmd_result') or 0) & 255
    ) + _MISSION.pack(_STAGE_CODES.get(g('mission_stage'), _U8), (g('mission_note') or 0) & 255
    ) + _ROUTE.pack((g('mission_point') or 0) & 255, (g('mission_points') or 0) & 255)


def unpack_status(payload):
    """The dict pack_status takes, or None for a frame that is not a version-1 STATUS."""
    if len(payload) < STATUS_SIZE or payload[0] != VERSION:
        return None
    (_, seq, flags, mode, inhibit, depth, alt, vn, cube, roll, pitch, vx, vy, vz, dvl_age, vn_age, cube_age,
     batt, lat, lon, fix, sats, stills, sweeps, disk, cmd_seq, cmd_result) = _STATUS.unpack_from(payload)
    out = {name: bool(flags >> i & 1) for i, name in enumerate(FLAGS)}
    velocity = tuple(_known(v, _I16, 100) for v in (vx, vy, vz))
    out.update(
        seq=seq, mode=None if mode == _U8 else mode,
        inhibit=_INHIBIT_NAMES.get(inhibit), inhibit_known=inhibit in _INHIBIT_NAMES,
        depth_m=_known(depth, _I16, 100),
        alt_m=None if alt == _I16 else (-1.0 if alt == ALT_NO_LOCK else alt / 100),
        vn_heading_deg=_known(vn, _U16, 100), cube_heading_deg=_known(cube, _U16, 100),
        roll_deg=_known(roll, _I8, 1), pitch_deg=_known(pitch, _I8, 1),
        dvl_velocity=None if None in velocity else velocity,
        dvl_age_s=_known(dvl_age, _U8, 10), vn_age_s=_known(vn_age, _U8, 10),
        cube_age_s=_known(cube_age, _U8, 10),
        battery_v=_known(batt, _U16, 100),
        lat=_known(lat, _I32, 1e7), lon=_known(lon, _I32, 1e7),
        gps_fix=None if fix == _U8 else fix, gps_sats=None if sats == _U8 else sats,
        rec_cam_count=stills, rec_sonar_count=sweeps,
        disk_free_gib=None if disk == _U8 else disk,
        cmd_seq=cmd_seq, cmd_result=cmd_result)
    known = len(payload) >= STATUS_SIZE + _MISSION.size     # a sub from before the mission says nothing of one
    stage, note = _MISSION.unpack_from(payload, STATUS_SIZE) if known else (_U8, 0)
    out.update(mission_known=known and stage in _STAGE_NAMES, mission_stage=_STAGE_NAMES.get(stage),
               mission_note=note)
    at = STATUS_SIZE + _MISSION.size
    point, points = _ROUTE.unpack_from(payload, at) if len(payload) >= at + _ROUTE.size else (0, 0)
    out.update(mission_point=point or None, mission_points=points or None)
    return out


def pack_command(seq, command, argument=0):
    if not 1 <= seq <= 255:
        raise ValueError('command seq must be 1-255 (0 means "no command yet" in STATUS)')
    return _COMMAND.pack(VERSION, seq, command, argument)


def unpack_command(payload):
    """(seq, command, argument), or None."""
    if len(payload) < COMMAND_SIZE or payload[0] != VERSION or payload[1] == 0:
        return None
    return tuple(_COMMAND.unpack_from(payload)[1:])


def _dm(metres):
    return 0 if metres is None else max(1, min(255, round(metres * DM_PER_M)))


def pack_mission_start(seq, mission):
    """A CMD_MISSION_START frame for radio_route.mission_plan's mission dict. Values are only
    squeezed into their bytes here; Graey checks them (gui_node, through radio_route.py)."""
    kind, dive, points = mission['kind'], mission.get('dive') or {}, mission.get('points') or []
    if kind not in KINDS or len(points) > MAX_POINTS:
        raise ValueError('a mission is dive, route or full, with up to %d points' % MAX_POINTS)
    body = _START.pack(_dm(dive.get('depth_m')), _dm(dive.get('altitude_m')), _dm(dive.get('max_depth_m')),
                       len(points))
    body += b''.join(_POINT.pack(round(lat * 1e7), round(lon * 1e7)) for lat, lon in points)
    return pack_command(seq, CMD_MISSION_START, KINDS.index(kind) + 1) + body


def unpack_mission_start(payload):
    """The mission dict pack_mission_start was given (depths to 0.1 m), or None for a short frame."""
    cmd = unpack_command(payload)
    if cmd is None or cmd[1] != CMD_MISSION_START or not 1 <= cmd[2] <= len(KINDS)             or len(payload) < COMMAND_SIZE + _START.size:
        return None
    depth, alt, deepest, n = _START.unpack_from(payload, COMMAND_SIZE)
    at = COMMAND_SIZE + _START.size
    if len(payload) < at + n * _POINT.size:
        return None
    points = [[v / 1e7 for v in _POINT.unpack_from(payload, at + i * _POINT.size)] for i in range(n)]
    dive = {k: v / DM_PER_M for k, v in (('depth_m', depth), ('altitude_m', alt), ('max_depth_m', deepest)) if v}
    mission = dict(kind=KINDS[cmd[2] - 1], dive=dive)
    if mission['kind'] != 'dive':
        mission['points'] = points
    return mission


def record_argument(camera=None, sonar=None):
    """CMD_RECORD's argument: None leaves that recorder as it is."""
    arg = 0
    if camera is not None:
        arg |= REC_SET_CAMERA | (REC_CAMERA if camera else 0)
    if sonar is not None:
        arg |= REC_SET_SONAR | (REC_SONAR if sonar else 0)
    return arg


def record_wanted(argument):
    """(camera, sonar) from a CMD_RECORD argument: True, False, or None for "leave it"."""
    return (bool(argument & REC_CAMERA) if argument & REC_SET_CAMERA else None,
            bool(argument & REC_SONAR) if argument & REC_SET_SONAR else None)


def tunnel_body(message):
    """The payload bytes of a pymavlink TUNNEL message (its array is padded to 128)."""
    return bytes(message.payload[:message.payload_length])
