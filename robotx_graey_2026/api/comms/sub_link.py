"""GRAEY's pure submarine codec, following the team's boat_link pack/unpack split.

Canonical copy. Byte-identical copies live in Ekko/uav_common and OCS/fleet.
No ROS, telemetry reads, vehicle commands, network access, or synthetic data.
Sender optionally journals actual mission products for process-restart recovery.
"""
import math
import json
import os
from pathlib import Path
import secrets
import struct
import tempfile

PAYLOAD_SUB = 32775
PAYLOAD_PIPELINE_SURVEY = 32776
PAYLOAD_RESOURCE_REQUEST = 32777
SUB_TYPES = (PAYLOAD_SUB, PAYLOAD_PIPELINE_SURVEY, PAYLOAD_RESOURCE_REQUEST)
VERSION = 1
MAX_PAYLOAD = 128
_HEADER = struct.Struct('<BIII')  # version, run, sender session, sequence
_SUB = struct.Struct('<ii6fBB')  # lat/lon E7; speed, heading, roll, pitch, HAE, depth; state, task
_SURVEY = struct.Struct('<iiB')  # active buoy E7, count; then ordered segment bytes
_RESOURCE = struct.Struct('<BB')  # tin color, destination-circle color
MAX_SEGMENTS = MAX_PAYLOAD - _HEADER.size - _SURVEY.size
STATES = {'STATE_KILLED': 1, 'STATE_MANUAL': 2, 'STATE_AUTO': 3}
TASKS = {'TASK_NONE': 1, 'TASK_SAFE_PASSAGE': 2, 'TASK_INFRA_SURVEY_REPAIR': 3,
         'TASK_COORDINATED_LOGISTICS': 4, 'TASK_DYNAMIC_INCIDENT': 5}
COLORS = {'COLOR_RED': 1, 'COLOR_GREEN': 2, 'COLOR_BLUE': 3, 'COLOR_ANY': 4}
SEGMENTS = {'PIPELINE_SEGMENT_INTACT': 1, 'PIPELINE_SEGMENT_DAMAGED': 2}


def _uint(value, name):
    if type(value) is not int or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(f'{name} must be uint32')
    return value


def _number(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return value


def _coordinate(position):
    lat = _number(position['latitude'], 'latitude')
    lon = _number(position['longitude'], 'longitude')
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError('coordinate out of range')
    return round(lat * 1e7), round(lon * 1e7)


def _enum(value, mapping):
    if value not in mapping:
        raise ValueError(f'expected one of {tuple(mapping)}; got {value!r}')
    return mapping[value]


def _name(value, mapping):
    for name, code in mapping.items():
        if code == value:
            return name
    raise ValueError(f'unknown enum value {value}')


def _header(run_id, session_id, sequence):
    return _HEADER.pack(VERSION, _uint(run_id, 'run_id'),
                        _uint(session_id, 'session_id'), _uint(sequence, 'sequence'))


def _read(payload, minimum):
    raw = bytes(payload)
    if not minimum <= len(raw) <= MAX_PAYLOAD:
        raise ValueError('invalid submarine payload length')
    version, run_id, session_id, sequence = _HEADER.unpack_from(raw)
    if version != VERSION:
        raise ValueError(f'unsupported submarine version {version}')
    return raw[_HEADER.size:], dict(run_id=run_id, session_id=session_id, sequence=sequence)


def pack_sub(heartbeat, *, run_id, session_id, sequence):
    """Encode actual UUV heartbeat fields; task changes travel in current_task."""
    lat, lon = _coordinate(heartbeat['position'])
    values = [_number(heartbeat[k], k) for k in
              ('spd_mps', 'heading_deg', 'altitude_hae_m', 'depth_m')]
    if values[0] < 0 or values[3] < 0 or not 0 <= values[1] < 360:
        raise ValueError('invalid speed, heading, or depth')
    angles = [float('nan') if heartbeat.get(k) is None else _number(heartbeat[k], k)
              for k in ('roll_deg', 'pitch_deg')]
    return _header(run_id, session_id, sequence) + _SUB.pack(
        lat, lon, values[0], values[1], *angles, values[2], values[3],
        _enum(heartbeat['state'], STATES), _enum(heartbeat['current_task'], TASKS))


def unpack_sub(payload):
    body, out = _read(payload, _HEADER.size + _SUB.size)
    if len(body) != _SUB.size:
        raise ValueError('invalid SUB body size')
    lat, lon, speed, heading, roll, pitch, hae, depth, state, task = _SUB.unpack(body)
    heartbeat = dict(position=dict(latitude=lat/1e7, longitude=lon/1e7),
                     spd_mps=speed, heading_deg=heading, altitude_hae_m=hae,
                     depth_m=depth, state=_name(state, STATES), current_task=_name(task, TASKS),
                     vehicle_type='TYPE_UUV', flight_phase='FLIGHT_PHASE_UNKNOWN')
    for key, angle in (('roll_deg', roll), ('pitch_deg', pitch)):
        if not math.isnan(angle):
            heartbeat[key] = angle
    # Apply the same finite/range validation to received fields.
    pack_sub(heartbeat, **out)
    out.update(kind='sub', heartbeat=heartbeat)
    return out


def pack_pipeline_survey(active_buoy_position, segments, *, run_id, session_id, sequence):
    """Ordered ORIGINAL observations, nearest active buoy first; never post-repair edits."""
    if not run_id:
        raise ValueError('survey requires a run_id')
    segments = list(segments)
    if not 1 <= len(segments) <= MAX_SEGMENTS:
        raise ValueError(f'survey requires 1..{MAX_SEGMENTS} observed segments')
    lat, lon = _coordinate(active_buoy_position)
    return (_header(run_id, session_id, sequence) + _SURVEY.pack(lat, lon, len(segments))
            + bytes(_enum(s, SEGMENTS) for s in segments))


def unpack_pipeline_survey(payload):
    body, out = _read(payload, _HEADER.size + _SURVEY.size + 1)
    lat, lon, count = _SURVEY.unpack_from(body)
    if len(body) != _SURVEY.size + count:
        raise ValueError('survey segment count disagrees with payload')
    survey = dict(active_buoy_position=dict(latitude=lat/1e7, longitude=lon/1e7),
                  segments=[_name(s, SEGMENTS) for s in body[_SURVEY.size:]])
    pack_pipeline_survey(**survey, **out)
    out.update(kind='pipeline_survey', pipeline_survey=survey)
    return out


def pack_resource_request(resource_color, delivery_circle_color, *, run_id, session_id, sequence):
    """Advanced: resource COLOR_ANY. Disruptive: actual tin color, then circle color."""
    if not run_id:
        raise ValueError('resource request requires a run_id')
    tin = _enum(resource_color, COLORS)
    circle = _enum(delivery_circle_color, COLORS)
    if circle == COLORS['COLOR_ANY']:
        raise ValueError('delivery circle must be RED, GREEN, or BLUE')
    return _header(run_id, session_id, sequence) + _RESOURCE.pack(tin, circle)


def unpack_resource_request(payload):
    body, out = _read(payload, _HEADER.size + _RESOURCE.size)
    if len(body) != _RESOURCE.size:
        raise ValueError('invalid resource body size')
    tin, circle = _RESOURCE.unpack(body)
    request = dict(resource_color=_name(tin, COLORS), delivery_circle_color=_name(circle, COLORS))
    pack_resource_request(**request, **out)
    request['task'] = 'TASK_INFRA_SURVEY_REPAIR'
    out.update(kind='resource_delivery', resource_delivery=request)
    return out


def decode(payload_type, payload):
    decoder = {PAYLOAD_SUB: unpack_sub, PAYLOAD_PIPELINE_SURVEY: unpack_pipeline_survey,
               PAYLOAD_RESOURCE_REQUEST: unpack_resource_request}.get(payload_type)
    return None if decoder is None else decoder(payload)


def ocs_envelope(packet, vehicle_id='UUV1'):
    """Local OCS ingress wrapper; only its report is official RoboCommand JSON."""
    body = 'heartbeat' if packet['kind'] == 'sub' else packet['kind']
    return {'sub_event': {k: packet[k] for k in ('run_id', 'session_id', 'sequence')},
            'report': {'vehicle_id': vehicle_id, body: packet[body]}}


class Receiver:
    """Decoded state for Ekko and ground; receiving colors does not begin delivery."""
    def __init__(self):
        self.status = self.survey = self.resource = None

    def hear(self, payload_type, payload):
        packet = decode(payload_type, payload)
        if packet is None:
            return None
        if packet['kind'] == 'sub':
            self.status = packet
        elif packet['kind'] == 'pipeline_survey':
            self.survey = packet
        else:
            self.resource = packet
        return packet

    def delivery_begun(self, *, run_id, session_id, sequence, vehicle_id='UAV1'):
        """Called ONLY by actual UAV delivery software, referencing the received request."""
        packet = self.resource
        if packet is None or any(packet[k] != v for k, v in
                                 dict(run_id=run_id, session_id=session_id, sequence=sequence).items()):
            raise ValueError('delivery start must reference the received resource request')
        return ocs_envelope(packet, vehicle_id)


class Sender:
    """Real producers fill slots; empty slots never manufacture messages.

    Fresh status at 2 Hz; actual survey/resource events repeat every 3 s to
    bridge radio gaps. Session/run/sequence identify copies for OCS deduplication.
    """
    def __init__(self, run_id=None, session_id=None, outbox_path=None):
        self.session_id = secrets.randbits(32) if session_id is None else session_id
        self.sequence = 0
        self.run_id = 0 if run_id is None else _uint(run_id, 'run_id')
        self._packets = {}
        self._sent = {}
        self._status_at = None
        self.submerged = False
        self.outbox_path = Path(outbox_path) if outbox_path else None
        self._restore()
        if run_id is not None:
            self.start_run(run_id)

    def _restore(self):
        if self.outbox_path is None or not self.outbox_path.exists():
            return
        saved = json.loads(self.outbox_path.read_text(encoding='utf-8'))
        if saved.get('version') != 1 or type(saved.get('submerged')) is not bool:
            raise ValueError('unsupported or invalid submarine outbox')
        run_id = _uint(saved['run_id'], 'outbox run_id')
        packets = {}
        for key, encoded in saved['packets'].items():
            ptype = int(key)
            if ptype not in (PAYLOAD_PIPELINE_SURVEY, PAYLOAD_RESOURCE_REQUEST):
                raise ValueError('outbox must contain only survey/resource products')
            payload = bytes.fromhex(encoded)
            if decode(ptype, payload)['run_id'] != run_id:
                raise ValueError('outbox contains a report from another run')
            packets[ptype] = payload
        self.run_id, self.submerged, self._packets = run_id, saved['submerged'], packets
        # No old status or monotonic sample time is restored. Stored event
        # bytes retain their original session/sequence; new status uses ours.

    def _save(self, packets, run_id, submerged):
        if self.outbox_path is None:
            return
        saved = dict(version=1, run_id=run_id, submerged=submerged,
                     packets={str(k): p.hex() for k, p in packets.items() if k != PAYLOAD_SUB})
        path = self.outbox_path
        path.parent.mkdir(parents=True, exist_ok=True)
        pending = None
        try:
            with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=path.parent,
                                             prefix=path.name+'.', delete=False) as stream:
                pending = Path(stream.name)
                json.dump(saved, stream, sort_keys=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(pending, path)
        finally:
            if pending is not None and pending.exists():
                pending.unlink()

    def set_submerged(self, submerged):
        """Mission-supplied communications phase; never inferred from an RF gap."""
        if type(submerged) is not bool:
            raise ValueError('submerged must be bool')
        if submerged != self.submerged:
            self._save(self._packets, self.run_id, submerged)
            self.submerged = submerged
            self._sent.clear()  # First surface poll offers retained products immediately.

    def start_run(self, run_id):
        _uint(run_id, 'run_id')
        if run_id != self.run_id:
            self._save({}, run_id, self.submerged)
            self.run_id = run_id
            self._packets.clear()
            self._sent.clear()
            self._status_at = None

    def _set(self, ptype, packer, *args, **kwargs):
        sequence = (self.sequence + 1) & 0xFFFFFFFF
        payload = packer(*args, **kwargs, run_id=self.run_id,
                         session_id=self.session_id, sequence=sequence)
        if ptype != PAYLOAD_SUB:
            candidate = dict(self._packets)
            candidate[ptype] = payload
            self._save(candidate, self.run_id, self.submerged)
        self.sequence = sequence
        self._packets[ptype] = payload
        if ptype != PAYLOAD_SUB:
            self._sent.pop(ptype, None)

    def update_status(self, heartbeat, now):
        self._set(PAYLOAD_SUB, pack_sub, heartbeat)
        self._status_at = now

    def survey_complete(self, active_buoy_position, segments):
        self._set(PAYLOAD_PIPELINE_SURVEY, pack_pipeline_survey, active_buoy_position, segments)

    def resource_observed(self, resource_color, delivery_circle_color):
        self._set(PAYLOAD_RESOURCE_REQUEST, pack_resource_request, resource_color, delivery_circle_color)

    def due(self, now):
        if self.submerged:
            return []
        out = []
        for ptype, payload in self._packets.items():
            if ptype == PAYLOAD_SUB and (self._status_at is None or now-self._status_at > 1):
                continue
            period = .5 if ptype == PAYLOAD_SUB else 3
            if ptype not in self._sent or now-self._sent[ptype] >= period:
                if ptype == PAYLOAD_SUB:
                    # A fresh cached observation may support successive 2 Hz
                    # heartbeats. Each publication needs its own transport ID
                    # so direct/relay dedup does not suppress the next beat.
                    self.sequence = (self.sequence + 1) & 0xFFFFFFFF
                    payload = pack_sub(unpack_sub(payload)['heartbeat'], run_id=self.run_id,
                                       session_id=self.session_id, sequence=self.sequence)
                out.append((ptype, payload))
                self._sent[ptype] = now
        return out
