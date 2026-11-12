"""RFD mission endpoint on GRAEY's Jetson, routed through the Cube.

The node never opens serial or UDP.  It uses telemetry_gateway's ROS transport as
companion sysid 43/component 201, receives plans from the laptop, and invokes the
same checked GUI mission API used over WiFi/tether.
"""
import json
import math
import os
import time
import urllib.error
import urllib.request
from collections import OrderedDict
from pathlib import Path

from . import competition_codec as rx
from .competition_telemetry import CompetitionTelemetry
from .mission_radio import (Assembler, KIND_REQUEST, KIND_RESPONSE, PAYLOAD_TYPE,
                            encode, revision)
from ..gui.navigation_view import NAVIGATION_VIEW_KINDS
from ..pixhawk.identity import COMPANION_COMPONENT, COMPANION_SYSTEM

OCS_SYSTEM = 250
OCS_COMPONENT = 190
LAPTOP_SYSTEM = 254
LAPTOP_COMPONENT = 191
def accept_run_start(source_system, source_component, target_system, payload, sequence, active, now):
    """Return (confirmed-event, ACK-payload) or (None, None).

    An early RunStart is deliberately not ACKed: fleet_link will retry it until a
    prepared competition mission exists. A different run is published (so the
    old gate revokes) but is not ACKed as if the old mission accepted it.
    """
    if (source_system,source_component)!=(OCS_SYSTEM,OCS_COMPONENT) or target_system not in (0,COMPANION_SYSTEM):
        return None,None
    try:data=rx.unpack_run_start(payload)
    except ValueError:return None,None
    if not active or active.get('competition') is not True:return None,None
    event=dict(run_id=data['run_id'],remote_run_id=data['run_id'],remote_state='RUNNING',
        run_confirmed=True,source='rfd',received_monotonic=now,published_monotonic=now)
    if active.get('robocommand_run_id') not in (None,data['run_id']):return event,None
    return event,rx.pack_ack(msg_id=rx.RUN_START,seq=sequence,src=rx.NODE_UUV)


def validate_mission(value):
    if not isinstance(value, dict):
        raise ValueError('mission must be an object')
    kind = value.get('kind')
    if kind not in ('waypoints', 'pipeline', 'simple_sequence', 'survey',
                    'dive_here', 'pool_dive', 'surface_route',
                    'dock_pipeline_dock'):
        raise ValueError('unknown GRAEY mission kind')
    if type(value.get('competition', False)) is not bool:
        raise ValueError('competition must be a boolean')
    points = value.get('points', [])
    if kind in ('simple_sequence','dive_here','pool_dive'):
        if points not in (None, []):
            raise ValueError(kind+' does not accept geographic points')
        points = []
    else:
        if not isinstance(points, list) or not 1 <= len(points) <= 256:
            raise ValueError('mission requires 1-256 points')
        if kind in ('pipeline', 'survey') and len(points) < 2:
            raise ValueError(kind+' requires at least a start and end point')
        if kind == 'pipeline' and len(points) != 2:
            raise ValueError('pipeline requires exactly a start and end point')
        if kind in ('surface_route','dock_pipeline_dock') and len(points)>15:
            raise ValueError(kind+' accepts at most 15 points')
        clean = []
        for item in points:
            if not isinstance(item, dict):
                raise ValueError('mission point must be an object')
            lat, lon = item.get('lat'), item.get('lon')
            if (isinstance(lat, bool) or isinstance(lon, bool)
                    or not isinstance(lat, (int, float)) or not isinstance(lon, (int, float))
                    or not math.isfinite(lat) or not math.isfinite(lon)
                    or abs(lat) > 85 or abs(lon) > 180):
                raise ValueError('invalid mission coordinates')
            clean.append(dict(item, lat=float(lat), lon=float(lon)))
        points = clean
    clean=dict(value,kind=kind,competition=value.get('competition',False),points=points)
    dive=value.get('dive',{})
    if kind in ('dive_here','pool_dive','dock_pipeline_dock'):
        if not isinstance(dive,dict) or set(dive)-{'depth_m','altitude_m','max_depth_m'}:
            raise ValueError('invalid dive settings')
        numeric={}
        for key,item in dive.items():
            if isinstance(item,bool) or not isinstance(item,(int,float)) or not math.isfinite(item) or item<=0:
                raise ValueError('invalid '+key)
            numeric[key]=float(item)
        if kind=='pool_dive':
            if set(numeric)!={'depth_m'}:raise ValueError('pool dive requires depth_m only')
        else:
            numeric.setdefault('altitude_m',3.5)
            if 'depth_m' in numeric:raise ValueError(kind+' uses DVL altitude and max depth, not depth_m')
            if 'max_depth_m' not in numeric:raise ValueError(kind+' requires max_depth_m')
        clean['dive']=numeric
    return clean


class MissionBackend:
    def __init__(self, directory=None, gui_port=8090, opener=urllib.request.urlopen,
                 clock=time.time):
        self.directory = Path(directory or os.environ.get(
            'GRAEY_RADIO_MISSION_DIR', str(Path.home()/'robotx_ws/missions/radio')))
        self.gui_port, self.opener, self.clock = gui_port, opener, clock
        self.selected = None
        self.active = None

    def _request(self, path, body=None):
        if path not in ('/api/waypoint','/api/calibration','/api/recording'):
            raise ValueError('unsupported local GRAEY operation')
        req = urllib.request.Request(f'http://127.0.0.1:{self.gui_port}'+path,
            data=None if body is None else json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'}, method='GET' if body is None else 'POST')
        try:
            with self.opener(req, timeout=8) as reply:
                result = json.loads(reply.read().decode())
        except urllib.error.HTTPError as exc:
            try: detail = json.loads(exc.read().decode()).get('msg')
            except Exception: detail = None
            raise ValueError(detail or f'GRAEY mission API refused HTTP {exc.code}') from None
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError('GRAEY mission API unavailable: '+str(exc)) from None
        if not isinstance(result, dict) or result.get('ok') is False:
            raise ValueError((result or {}).get('msg', 'GRAEY mission API refused request'))
        return result

    def _post(self, body):return self._request('/api/waypoint',body)

    def _radio_mission(self, mission):
        kind=mission['kind'];dive=mission.get('dive',{});points=mission.get('points',[])
        if kind=='dive_here':return dict(kind='dive',operator_kind=kind,dive=dive)
        if kind=='pool_dive':return dict(kind='dive',operator_kind=kind,dive=dive)
        if kind=='surface_route':return dict(kind='route',operator_kind=kind,points=[[p['lat'],p['lon']] for p in points])
        if kind=='dock_pipeline_dock':
            return dict(kind='full',operator_kind=kind,dive=dive,points=[[p['lat'],p['lon']] for p in points])
        raise ValueError('not a Chris-derived radio mission')

    def store(self, mission):
        mission = validate_mission(mission)
        rev = revision(mission)
        self.directory.mkdir(parents=True, exist_ok=True)
        final = self.directory/(rev+'.json')
        temporary = self.directory/(rev+'.tmp')
        temporary.write_text(json.dumps(mission, indent=2, sort_keys=True), encoding='utf-8')
        os.replace(temporary, final)
        self.selected = dict(mission=mission, revision=rev, stored_at=self.clock())
        return dict(revision=rev, kind=mission['kind'], point_count=len(mission['points']))

    def prepare(self, mission):
        stored = self.store(mission)
        m = self.selected['mission']
        if m['kind'] == 'simple_sequence':
            result = self._post(dict(action='simple_start', competition=m['competition']))
        elif m['kind'] in ('dive_here','pool_dive','surface_route','dock_pipeline_dock'):
            result=self._post(dict(action='radio_mission_start',mission=self._radio_mission(m),
                                   competition=m['competition']))
        elif m['kind'] == 'survey':
            result = self._post(dict(action='survey_start', points=m['points'], dry=False,
                                     competition=m['competition']))
        else:
            preview = self._post(dict(action='preview', plan=dict(
                points=m['points'], recording_mode='manual')))
            result = self._post(dict(action='start', plan_id=preview['plan_id'],
                                     competition=m['competition']))
        self.active = dict(revision=stored['revision'], run_id=result['run_id'],
                           kind=m['kind'], competition=m['competition'], prepared_at=self.clock())
        return dict(stored, **self.active, state='PREPARING')

    def abort(self, run_id=None):
        active = self.active
        if not active:
            raise ValueError('no radio-prepared mission is active')
        if run_id is not None and run_id != active['run_id']:
            raise ValueError('active mission run ID does not match')
        result = self._post(dict(action='survey_stop' if active['kind'] == 'survey' else 'abort',
                                 run_id=active['run_id']))
        active['state']='STOP_REQUESTED'
        return dict(result, run_id=active['run_id'], state='STOP_REQUESTED')

    def note_waypoint(self, status):
        if (self.active is None and isinstance(status,dict)
                and status.get('competition') is True
                and isinstance(status.get('run_id'),str) and status['run_id']
                and status.get('stage') not in ('COMPLETE','ABORTED')):
            plan=status.get('plan') if isinstance(status.get('plan'),dict) else {}
            self.active=dict(revision=None,run_id=status['run_id'],
                kind=plan.get('operator_kind') or plan.get('kind') or 'waypoints',
                competition=True,prepared_at=self.clock(),source='ip',state=status.get('stage'))
        active=self.active
        if (active and active['kind']!='survey' and isinstance(status,dict)
                and status.get('run_id')==active['run_id']):
            active['state']=status.get('stage',active.get('state'))
            if status.get('stage') in ('COMPLETE','ABORTED'):self.active=None

    def note_survey(self, status):
        if (self.active is None and isinstance(status,dict)
                and status.get('competition') is True and status.get('running') is True
                and isinstance(status.get('run_id'),str) and status['run_id']):
            self.active=dict(revision=None,run_id=status['run_id'],kind='survey',
                competition=True,prepared_at=self.clock(),source='ip',state='PREPARING')
        active=self.active
        if (active and active['kind']=='survey' and isinstance(status,dict)
                and status.get('run_id')==active['run_id']):
            active['state']='RUNNING' if status.get('running') else status.get('result')
            if status.get('running') is False and status.get('result') is not None:self.active=None

    def status(self):
        return dict(selected=None if self.selected is None else {
            'revision': self.selected['revision'], 'kind': self.selected['mission']['kind'],
            'point_count': len(self.selected['mission']['points'])}, active=self.active)

    def handle(self, request):
        if not isinstance(request, dict):
            raise ValueError('request must be an object')
        op = request.get('op')
        if op == 'load':
            return self.store(request.get('mission'))
        if op == 'prepare':
            return self.prepare(request.get('mission'))
        if op == 'abort':
            return self.abort(request.get('run_id'))
        if op == 'status':
            return self.status()
        if op == 'navigation_prepare':
            return self._request('/api/calibration',dict(action='prepare',confirmed=True,
                surface_confirmed=request.get('surface_confirmed') is True,alignment_mode='gps'))
        if op == 'record':
            command=request.get('command')
            if not isinstance(command,dict):raise ValueError('record command required')
            return self._request('/api/recording',command)
        if op == 'recording_status':
            status=self._request('/api/recording')
            # Keep normal RFD status bounded. Topic discovery is an IP/tether setup
            # operation; start/stop and the active-session result remain available.
            status.pop('topics',None)
            return status
        if op == 'abort_run':
            run_id=request.get('run_id');kind=request.get('kind')
            if not isinstance(run_id,str) or not run_id:raise ValueError('run_id required')
            return self._post(dict(action='survey_stop' if kind=='survey' else 'abort',run_id=run_id))
        raise ValueError('unknown radio mission operation')


class HeartbeatCapture:
    def __init__(self): self.value = None
    def update_status(self, **value): self.value = value


class BeaconGate:
    """Change-driven private status; standard RXL heartbeat carries normal liveness."""
    def __init__(self):
        self.wire = None
        self.sent_at = -math.inf

    def due(self, event, now, heartbeat_available):
        wire = json.dumps(event, sort_keys=True, separators=(',', ':'))
        period = 5.0 if heartbeat_available else 1.0
        if wire == self.wire and now-self.sent_at < period:
            return False
        self.wire, self.sent_at = wire, now
        return True


def build_heartbeat(value):
    """Build one truthful UUV RXL heartbeat or explain why it cannot be sent."""
    if not isinstance(value, dict):
        raise ValueError('competition telemetry is unavailable')
    numeric = ('lat','lon','spd_mps','hdg_deg','roll_deg','pitch_deg','alt_m')
    bad = [key for key in numeric if isinstance(value.get(key), bool)
           or not isinstance(value.get(key), (int, float)) or not math.isfinite(value[key])]
    if bad:
        raise ValueError('competition heartbeat missing finite '+', '.join(bad))
    return rx.pack_heartbeat(vehicle=rx.NODE_UUV, robot_state=value['robot_state'],
        flight_phase=0, pos_valid=True, att_valid=True, alt_valid=True,
        lat=value['lat'], lon=value['lon'], spd_mps=value['spd_mps'],
        hdg_deg=value['hdg_deg'], roll_deg=value['roll_deg'], pitch_deg=value['pitch_deg'],
        alt_m=value['alt_m'], task=value['task'])


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from ..pixhawk.mavlink import Link

    class RadioMissionNode(Node):
        def __init__(self):
            super().__init__('radio_mission_node')
            self.declare_parameter('gui_port', 8090)
            self.declare_parameter('mission_dir', '/root/robotx_ws/missions/radio')
            self.backend = MissionBackend(self.get_parameter('mission_dir').value,
                                          self.get_parameter('gui_port').value)
            self.assembler = Assembler()
            self.cache = OrderedDict()
            self.telemetry = CompetitionTelemetry(time.monotonic)
            self.capture = HeartbeatCapture()
            self.link = Link('ros://graey/fcu', COMPANION_COMPONENT, self.get_logger(),
                system=COMPANION_SYSTEM, node=self, kinds=set(NAVIGATION_VIEW_KINDS)|{'TUNNEL'})
            self.run_pub = self.create_publisher(String, '/graey/comms/radio_run', 10)
            self.status_pub = self.create_publisher(String, '/graey/comms/radio_mission_status', 10)
            self.create_subscription(String, '/graey/navigation/bridge_status',
                                     lambda m:self._status('bridge', m.data), 10)
            self.create_subscription(String, '/graey/control_watchdog/status',
                                     lambda m:self._status('control', m.data), 10)
            self.create_subscription(String, '/graey/comms/current_task',
                                     lambda m:self.telemetry.set_task(m.data), 10)
            self.create_subscription(String, '/graey/waypoint/status',
                                     lambda m:self._mission_status('waypoint',m.data), 10)
            self.create_subscription(String, '/graey/survey/status',
                                     lambda m:self._mission_status('survey',m.data), 10)
            self.error = None
            self.heartbeat_reason = 'waiting for measured competition telemetry'
            self.last_run = None
            self.beacon_id = 0xfffffffe
            self.beacon = BeaconGate()
            self.create_timer(.02, self._pump)
            self.create_timer(.5, self._heartbeat)
            self.create_timer(1., self._publish)

        def _status(self, name, raw):
            try:self.telemetry.put(name, json.loads(raw))
            except (ValueError, TypeError, json.JSONDecodeError) as exc:self.error = str(exc)

        def _mission_status(self, kind, raw):
            try:
                value=json.loads(raw)
                (self.backend.note_survey if kind=='survey' else self.backend.note_waypoint)(value)
            except (ValueError,TypeError,json.JSONDecodeError):pass

        def _send_tunnel(self, target_system, target_component, payload_type, payload):
            wire = bytes(payload)
            if len(wire) > 128: raise ValueError('TUNNEL payload exceeds 128 bytes')
            self.link.send('tunnel_send', target_system, target_component, payload_type,
                           len(wire), wire + bytes(128-len(wire)))

        def _reply(self, request_id, body):
            for chunk in encode(body, request_id, KIND_RESPONSE):
                self._send_tunnel(LAPTOP_SYSTEM, LAPTOP_COMPONENT, PAYLOAD_TYPE, chunk)

        def _mission_request(self, msg, payload):
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (LAPTOP_SYSTEM, LAPTOP_COMPONENT):
                return
            if getattr(msg, 'target_system', 0) not in (0, COMPANION_SYSTEM) or \
                    getattr(msg, 'target_component', 0) not in (0, COMPANION_COMPONENT):
                return
            complete = self.assembler.add(payload)
            if complete is None:return
            kind, request_id, request = complete
            if kind != KIND_REQUEST:return
            fingerprint = revision(request)
            cached = self.cache.get(request_id)
            if cached is not None:
                if cached[0] != fingerprint:
                    self._reply(request_id, dict(ok=False, msg='request ID reused with different data'))
                else:self._reply(request_id, cached[1])
                return
            try:
                result = dict(ok=True, result=self.backend.handle(request))
                self.error = None
            except (ValueError, OSError, KeyError) as exc:
                result = dict(ok=False, msg=str(exc))
                self.error = str(exc)
            self.cache[request_id] = (fingerprint, result)
            while len(self.cache) > 64:self.cache.popitem(last=False)
            self._reply(request_id, result)

        def _run_start(self, msg, payload):
            active=self.backend.active
            now = time.monotonic()
            event,ack=accept_run_start(msg.get_srcSystem(),msg.get_srcComponent(),
                getattr(msg,'target_system',0),payload,msg.get_seq(),active,now)
            if event is None:return
            self.last_run=event
            self.run_pub.publish(String(data=json.dumps(event)))
            if ack is None:return
            active.update(robocommand_run_id=event['run_id'],state='RUNNING_ADMITTED')
            self._send_tunnel(OCS_SYSTEM, OCS_COMPONENT, rx.payload_type(rx.ACK), ack)

        def _consume(self, kind, msg):
            if kind != 'TUNNEL':
                self.telemetry.consume(kind, msg);return
            length = int(getattr(msg, 'payload_length', 0))
            payload = bytes(getattr(msg, 'payload', []))[:length]
            ptype = int(getattr(msg, 'payload_type', -1))
            try:
                if ptype == PAYLOAD_TYPE:self._mission_request(msg, payload)
                elif ptype == rx.payload_type(rx.RUN_START):self._run_start(msg, payload)
            except (ValueError, TypeError, KeyError) as exc:
                self.error = str(exc)

        def _pump(self): self.link.drain(self._consume)

        def _heartbeat(self):
            now = time.monotonic()
            self.capture.value = None
            self.telemetry.update(self.capture, now)
            try:
                payload = build_heartbeat(self.capture.value)
                self._send_tunnel(0, 0, rx.payload_type(rx.HEARTBEAT), payload)
                self.heartbeat_reason = None
            except (ValueError, TypeError, KeyError) as exc:
                self.heartbeat_reason = self.telemetry.reason or str(exc)
            # A valid private status beacon establishes the companion route even while
            # competition telemetry is incomplete. It never fabricates a heartbeat.
            event = dict(event='radio_status', ok=self.error is None,
                         mission=self.backend.status(), error=self.error,
                         heartbeat_reason=self.heartbeat_reason,last_run=self.last_run)
            # The standard RXL heartbeat is the normal 2 Hz liveness source, as it
            # is for Crusader's OCS path.  Private application status is sent on
            # change and as a slow keepalive, not duplicated continuously.
            if self.beacon.due(event, now, self.heartbeat_reason is None):
                for chunk in encode(event, self.beacon_id, KIND_RESPONSE):
                    self._send_tunnel(0, 0, PAYLOAD_TYPE, chunk)

        def _publish(self):
            self.status_pub.publish(String(data=json.dumps(dict(
                online=True, companion_system=COMPANION_SYSTEM, component=COMPANION_COMPONENT,
                selected=self.backend.status()['selected'], active=self.backend.status()['active'],
                last_run=self.last_run, error=self.error, heartbeat_reason=self.heartbeat_reason,
                published_monotonic=time.monotonic()))))

        def destroy_node(self):
            self.link.mav.close()
            return super().destroy_node()

    rclpy.init(args=args)
    node = RadioMissionNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
