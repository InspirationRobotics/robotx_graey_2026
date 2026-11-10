"""Native SD-OCS Wi-Fi reporting, durable observations and nonblocking reconnects.

No radio/serial connections, MQTT client, arming, mission-start or kill commands.
"""
import json
import math
import select
import socket
import sqlite3
import struct
import time
from datetime import datetime, timezone
from pathlib import Path

from . import competition_codec as rx
from .ocs_discovery import BeaconListener, TargetChooser, check_fallback_hosts, on_course_network


def uint(value):
    if type(value) is not int or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError('run/sequence must be uint32')
    return value


def timestamp(wall):
    return datetime.fromtimestamp(wall,timezone.utc).isoformat(timespec='microseconds').replace('+00:00','Z')


class CompetitionSender:
    def __init__(self, path, wall=time.time, team_id='ASTA'):
        if str(path) != ':memory:':Path(path).parent.mkdir(parents=True,exist_ok=True)
        self.db = sqlite3.connect(str(path))
        self.db.execute('CREATE TABLE IF NOT EXISTS settings (key TEXT PRIMARY KEY,value TEXT)')
        self.db.execute('''CREATE TABLE IF NOT EXISTS events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,run INTEGER,observation_id TEXT,
            kind TEXT,report TEXT,verdict TEXT DEFAULT 'pending',detail TEXT,
            UNIQUE(run,observation_id))''')
        self.db.commit()
        row = self.db.execute("SELECT value FROM settings WHERE key='run'").fetchone()
        self.run = None if row is None else json.loads(row[0])
        self.wall,self.team_id = wall,team_id
        self.surface = self.surface_at = self.status_at = self.heartbeat_body = None
        self.remote_run = self.remote_at = None
        self.remote_state = None
        self.course = None
        self.native_run = False
        self.run_snapshot = {}
        self.report_envelope = False
        self.directive = None
        self.tiers_reply = None
        self.next_hb = self.next_event = 0

    def start_run(self, run):
        uint(run)
        with self.db:self.db.execute("INSERT OR REPLACE INTO settings VALUES('run',?)", (json.dumps(run),))
        self.run = run
        self.next_event = 0

    def set_surface(self,surface,now):
        if type(surface) is not bool:raise ValueError('surface state must be boolean')
        self.surface,self.surface_at = surface,now

    def update_status(self, *, now,depth_m,**fields):
        if depth_m is not None and (type(depth_m) not in (int,float) or not math.isfinite(depth_m) or depth_m < 0):
            raise ValueError('measured nonnegative depth is required')
        altitude = fields.get('alt_m')
        fields['alt_m'] = 0. if altitude is None else altitude
        body = rx.unpack_heartbeat(rx.pack_heartbeat(vehicle=rx.NODE_UUV,flight_phase=0,
            pos_valid=True,att_valid=True,alt_valid=altitude is not None,**fields))
        if body['robot_state'] == 0 or body['task'] not in rx.RX_TASK or body['task'] == 0 or body['hdg_deg'] is None:
            raise ValueError('heartbeat state, heading and task must be known')
        self.heartbeat_body = dict(vehicle_type='TYPE_UUV',state=rx.ROBOT_STATE[body['robot_state']],
            current_task=rx.RX_TASK[body['task']],position=dict(latitude=body['lat'],longitude=body['lon']),
            spd_mps=body['spd_mps'],heading_deg=body['hdg_deg'],roll_deg=body['roll_deg'],pitch_deg=body['pitch_deg'],
            altitude_hae_m=body['alt_m'],depth_m=depth_m)
        if depth_m is None:self.heartbeat_body.pop('depth_m')
        if altitude is None:self.heartbeat_body.pop('altitude_hae_m')
        self.status_at = now

    def _event(self, observation_id, kind, body):
        if self.run is None:raise ValueError('confirmed RoboCommand run ID required before recording observations')
        if not isinstance(observation_id,str) or not 1 <= len(observation_id) <= 128:
            raise ValueError('observation_id must be a stable string of 1..128 characters')
        previous = self.db.execute('SELECT id,kind,report FROM events WHERE run=? AND observation_id=?',
                                   (self.run,observation_id)).fetchone()
        if previous:
            if previous[1] != kind or json.loads(previous[2])[kind] != body:
                raise ValueError('observation_id reused with different contents')
            return previous[0]
        if self.db.execute("SELECT count(*) FROM events WHERE verdict='pending'").fetchone()[0] >= 2048:
            raise ValueError('outbox full; existing observations retained')
        report = dict(team_id=self.team_id,vehicle_id='UUV1',sent_at=timestamp(self.wall()),**{kind:body})
        with self.db:
            cursor = self.db.execute('INSERT INTO events(run,observation_id,kind,report) VALUES(?,?,?,?)',
                                     (self.run,observation_id,kind,json.dumps(report)))
            report['seq'] = uint(cursor.lastrowid)
            self.db.execute('UPDATE events SET report=? WHERE id=?',(json.dumps(report),cursor.lastrowid))
        return cursor.lastrowid

    def survey(self,observation_id,lat,lon,segments):
        packed = rx.pack_pipeline_survey(lat=lat,lon=lon,segments=segments)
        if not segments or any(type(s) is not int or s not in (1,2) for s in segments):
            raise ValueError('survey requires 1..8 ordered intact(1)/damaged(2) segments')
        p = rx.unpack_pipeline_survey(packed)
        return self._event(observation_id,'pipeline_survey',dict(
            active_buoy_position=dict(latitude=p['lat'],longitude=p['lon']),
            segments=[rx.PIPELINE_SEGMENT[s] for s in p['segments']]))

    def resource(self,observation_id,resource_color,delivery_circle_color):
        if (type(resource_color) is not int or resource_color not in (1,2,3)
                or type(delivery_circle_color) is not int or delivery_circle_color not in (1,2,3,4)):
            raise ValueError('resource color must be red/green/blue; circle may also be ANY')
        return self._event(observation_id,'resource_delivery',dict(task='TASK_INFRA_SURVEY_REPAIR',
            resource_color=rx.COLOR[resource_color],delivery_circle_color=rx.COLOR[delivery_circle_color]))

    def disconnected(self):
        self.native_run = False
        self.remote_at = None  # reconnect must learn current OCS run before sending retained reports
        self.report_envelope = False
        self.run_snapshot = {}
        self.directive = None
        self.tiers_reply = None

    def receive(self,obj,now):
        if not isinstance(obj,dict):raise ValueError('OCS frame must be an object')
        if isinstance(obj.get('ocs_info'),dict) and 'course' in obj['ocs_info']:
            self.course=obj['ocs_info']['course']
        native = obj.get('ocs_run')
        info = native if native is not None else obj.get('ocs_info',{}).get('graey_run')
        if info is not None:
            if not isinstance(info,dict):raise ValueError('invalid OCS run snapshot')
            if info.get('state') not in ('DISCONNECTED','CONNECTED','COURSE_RX','DECLARED','RUNNING','ENDED'):
                raise ValueError('invalid OCS run state')
            run = info.get('run_id')
            if run is not None:uint(run)
            self.remote_run,self.remote_state,self.remote_at = run,info.get('state'),now
            self.native_run = native is not None
            self.run_snapshot = dict(info)
            self.report_envelope = (native is None or info.get('graey_report_v1') is True)
            if 'reply' in info:self.tiers_reply = dict(info['reply'],received_monotonic=now)
            self.directive = None
            if self.remote_state == 'RUNNING' and run is not None and run != self.run:
                self.start_run(run)  # metadata only; no arming or mission execution
        directive = obj.get('ocs_directive')
        if isinstance(directive,dict) and directive.get('action') in ('standby_auto','run_end'):
            self.directive = dict(directive)
            if directive['action'] == 'run_end' and directive.get('run_id') == self.remote_run:
                self.remote_state,self.remote_at = 'ENDED',now
            elif directive['action'] == 'standby_auto':
                self.remote_state,self.remote_at = 'DECLARED',now
        receipt = obj.get('ocs_receipt')
        if receipt is None:return
        if not isinstance(receipt,dict):raise ValueError('invalid OCS receipt')
        seq = uint(receipt.get('vehicle_seq'))
        row = self.db.execute('SELECT kind,verdict FROM events WHERE id=?',(seq,)).fetchone()
        if row is None or receipt.get('kind') != row[0]:return
        verdict = receipt.get('verdict')
        if verdict not in ('published','acknowledged','held','dropped'):raise ValueError('unknown receipt verdict')
        if row[1] == 'pending' and verdict != 'published':
            with self.db:self.db.execute('UPDATE events SET verdict=?,detail=? WHERE id=?',
                                        (verdict,str(receipt.get('detail','')),seq))

    def due(self,now):
        reports = []
        if self.status_at is not None and now-self.status_at < 1.5 and now >= self.next_hb:
            # Preserve cadence through ordinary executor jitter, skip missed
            # periods after an outage rather than accumulating a heartbeat queue.
            self.next_hb = now+.5 if self.next_hb == 0 or now-self.next_hb >= .5 else self.next_hb+.5
            reports.append(dict(team_id=self.team_id,vehicle_id='UUV1',sent_at=timestamp(self.wall()),
                                heartbeat=self.heartbeat_body))
        # Routine IP heartbeat follows Crusader; surfaced task observations keep their gate.
        if self.surface is not True or self.surface_at is None or now-self.surface_at >= 2:return reports
        if (self.remote_at is not None and (self.native_run or 0 <= now-self.remote_at < 5) and self.remote_state == 'RUNNING'
                and (self.native_run or self.report_envelope) and self.run is not None and self.run == self.remote_run and now >= self.next_event):
            self.next_event = now+2
            row = self.db.execute("SELECT report FROM events WHERE run=? AND verdict='pending' ORDER BY id LIMIT 1",
                                  (self.run,)).fetchone()
            if row:
                report=json.loads(row[0])
                # Native team OCS takes the same standard RxReport as Crusader.
                # Retain the old envelope only for legacy SD fixtures/clients.
                reports.append(report if self.native_run else dict(graey_report=dict(run_id=self.run,report=report)))
        return reports

    def status(self,now):
        counts = dict(self.db.execute('SELECT verdict,count(*) FROM events GROUP BY verdict'))
        reports=[dict(zip(('sequence','run_id','observation_id','kind','verdict','detail'),row))
                 for row in self.db.execute('SELECT id,run,observation_id,kind,verdict,detail FROM events ORDER BY id DESC LIMIT 30')]
        return dict(run_id=self.run,remote_run_id=self.remote_run,remote_state=self.remote_state,
            run_confirmed=bool(self.remote_at is not None and (self.native_run or 0 <= now-self.remote_at < 5) and self.remote_state=='RUNNING'),
            surface=self.surface,transmission_enabled=bool(self.status_at is not None and now-self.status_at < 1.5),
            telemetry_fresh=bool(self.status_at is not None and now-self.status_at < 1.5),pending=counts.get('pending',0),
            broker_acknowledged=counts.get('acknowledged',0),rejected=counts.get('held',0)+counts.get('dropped',0),
            reports=reports,course=self.course,ocs_run=self.run_snapshot,ocs_directive=self.directive,tiers_reply=self.tiers_reply,
            report_transport_ready=self.native_run or self.report_envelope,
            report_hold_reason=None if (self.native_run or self.report_envelope) else 'Waiting for the OCS run snapshot',
            previous_run_pending=self.db.execute("SELECT count(*) FROM events WHERE verdict='pending' AND run!=?",(self.run,)).fetchone()[0])

    def close(self):self.db.close()


class WifiLink:
    """One TCP socket; bounded frames, immediate disconnect on stalled writes.

    tick() is called on the ROS executor. Connection attempts run on a helper
    thread so DNS/TCP timeouts cannot stall telemetry or status publication.
    """
    MAX_FRAME = 1 << 20  # same SD framing limit

    def __init__(self,host,port=37564,clock=time.monotonic,*,team_id='ASTA',fallback_hosts=(),discovery_port=37565):
        import concurrent.futures
        if not isinstance(host,str) or not host.strip():raise ValueError('ocs_host must be auto or the laptop Wi-Fi IP/hostname')
        if on_course_network(host):raise ValueError('OCS must be on the team network')
        fallback,problems = check_fallback_hosts(fallback_hosts)
        if problems:raise ValueError('; '.join(problems))
        self.chooser = TargetChooser(fallback,port,clock=clock) if host == 'auto' else None
        self.listener = BeaconListener(discovery_port,team_id,self.chooser) if self.chooser else None
        self.next_listen = 0
        self.target = None
        if type(port) is not int or not 1 <= port <= 65535:raise ValueError('invalid OCS TCP port')
        self.host,self.port,self.clock = host,port,clock
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=1,thread_name_prefix='ocs-wifi-connect')
        self.future = self.sock = None
        self.buffer = bytearray()
        self.next_connect = self.next_sync = 0
        self.connections = 0
        self.connected_at = None
        self.error = None

    @staticmethod
    def _connect(host,port):
        # Resolve on the worker, rejecting course addresses even through hostnames.
        addresses=socket.getaddrinfo(host,port,type=socket.SOCK_STREAM)
        if any(on_course_network(a[4][0]) for a in addresses):
            raise OSError('OCS address resolves to a course network')
        sock=socket.create_connection((addresses[0][4][0],port),1.)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_KEEPALIVE,1)
        if hasattr(socket,'TCP_KEEPIDLE'):
            sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_KEEPIDLE,5)
            sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_KEEPINTVL,2)
            sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_KEEPCNT,3)
        return sock

    def send_tiers(self,tiers,mission=''):
        if not isinstance(tiers,dict) or set(tiers) != {'task1','task2','task3','task4'} or any(
                v not in ('TIER_NONE','TIER_CORE','TIER_ADVANCED','TIER_DISRUPTIVE') for v in tiers.values()):
            raise ValueError('all four valid task tiers are required')
        if not isinstance(mission,str) or len(mission)>128:raise ValueError('invalid mission name')
        # A control request is sent once, never queued or replayed on reconnect.
        if not self.send(dict(ocs_tiers=dict(vehicle_id='UUV1',tiers=tiers,mission=mission,sent_at=timestamp(time.time())))):
            raise ValueError('OCS disconnected; tier request was not queued')

    @staticmethod
    def frame(obj):
        data = json.dumps(obj,allow_nan=False,separators=(',',':')).encode('utf-8')
        if not 0 < len(data) <= WifiLink.MAX_FRAME:raise ValueError('invalid TCP frame size')
        return struct.pack('!I',len(data))+data

    def send(self,obj):
        if self.sock is None:return False
        try:self.sock.sendall(self.frame(obj))
        except OSError as exc:
            self.drop(str(exc))
            return False
        return True

    def drop(self,error=None):
        if self.sock is not None:self.sock.close()
        self.sock = None
        self.buffer.clear()
        self.error = error
        self.next_connect = self.clock()+1

    def tick(self,sender):
        now = self.clock()
        if self.listener and not self.listener.listening and now >= self.next_listen:
            self.next_listen = now+5
            self.listener.start()
        if self.sock is None:
            sender.disconnected()
            if self.future is None and now >= self.next_connect:
                target = self.chooser.choose() if self.chooser else (self.host,self.port,'configured')
                if target:
                    self.target = target
                    self.future = self.pool.submit(self._connect,target[0],target[1])
            if self.future is not None and self.future.done():
                future,self.future = self.future,None
                try:
                    self.sock = future.result()
                    self.sock.setblocking(False)
                    if self.chooser:self.chooser.note_connected(self.target[0])
                    self.connections += 1
                    self.connected_at = now
                    self.error = None
                    self.send(dict(ocs_hello=dict(vehicle_id='UUV1',link='wifi',agent='graey_competition',synthesized=[])))
                    self.next_sync = now
                except OSError as exc:self.drop(str(exc))
            if self.sock is None:return
        try:
            # Bound each tick's work, including a peer sending oversized/invalid frames.
            for _ in range(8):
                if not select.select([self.sock],[],[],0)[0]:break
                chunk = self.sock.recv(65536)
                if not chunk:raise ConnectionError('OCS closed Wi-Fi connection')
                self.buffer.extend(chunk)
                while len(self.buffer) >= 4:
                    length = struct.unpack_from('!I',self.buffer)[0]
                    if not 0 < length <= self.MAX_FRAME:raise ValueError('invalid OCS frame length')
                    if len(self.buffer) < length+4:break
                    data = bytes(self.buffer[4:4+length]);del self.buffer[:4+length]
                    sender.receive(json.loads(data.decode('utf-8')),now)
            if sender.report_envelope and not sender.native_run and now >= self.next_sync:
                self.next_sync = now+2
                self.send({'graey_sync':{}})
            if not sender.native_run and now-(sender.remote_at if sender.remote_at is not None else self.connected_at) >= 7:
                raise ConnectionError('OCS run snapshot timeout')
            if self.sock is not None:
                for report in sender.due(now):
                    if not self.send(report):break
        except (OSError,ValueError,TypeError,AttributeError) as exc:
            self.drop(str(exc))
            sender.disconnected()

    def status(self):
        return dict(transport='wifi',host=self.target[0] if self.target else self.host,port=self.target[1] if self.target else self.port,
                    discovery=bool(self.listener),beacon_listening=bool(self.listener and self.listener.listening),connected=self.sock is not None,
                    connections=self.connections,last_error=self.error)

    def close(self):
        if self.listener:self.listener.stop()
        self.drop()
        if self.future is not None:
            try:self.future.result(timeout=2).close()
            except (OSError,TimeoutError):pass
        self.pool.shutdown(wait=False,cancel_futures=True)
