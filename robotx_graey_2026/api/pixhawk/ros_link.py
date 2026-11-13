"""Crusader-style semantic ROS transport for the single FCU gateway.

The gateway is the only process that decodes inbound MAVLink. Consumers name
the message kinds they need and subscribe only to those compact field streams;
there is deliberately no shared raw-frame topic to copy, parse and decode in
every Python process.
"""
import collections
import json
import re
import threading
import time
from types import SimpleNamespace
from .identity import FCU_COMPONENT, FCU_SYSTEM


FCU_RX_KINDS = frozenset({
    'ATTITUDE', 'EKF_STATUS_REPORT', 'GLOBAL_POSITION_INT',
    'GPS_GLOBAL_ORIGIN', 'GPS_RAW_INT', 'HEARTBEAT',
    'LOCAL_POSITION_NED', 'MISSION_COUNT', 'MISSION_ITEM_INT',
    'PARAM_VALUE', 'RC_CHANNELS',
    'SCALED_PRESSURE', 'SCALED_PRESSURE2', 'SCALED_PRESSURE3',
    'STATUSTEXT', 'TIMESYNC', 'TUNNEL',
})

# Crusader's telemetry bridge keeps only the newest sample for sensor/state
# streams.  Ordered protocol traffic must remain lossless: dropping one TUNNEL
# chunk, PARAM_VALUE reply or mission-download item corrupts the transaction
# rather than merely skipping an old display sample.  MISSION_COUNT and
# MISSION_ITEM_INT answer tools/survey_run.py reading the QGC plan off the Cube.
ORDERED_RX_KINDS = frozenset({'MISSION_COUNT', 'MISSION_ITEM_INT', 'PARAM_VALUE',
                              'STATUSTEXT', 'TUNNEL'})


def mav_topic(kind):
    if kind not in FCU_RX_KINDS or not re.fullmatch(r'[A-Z][A-Z0-9_]*', kind):
        raise ValueError(f'unsupported FCU receive kind: {kind!r}')
    return '/graey/fcu/mav/' + kind.lower()


class SemanticMessage:
    """Field-compatible view of one already-decoded MAVLink message."""
    def __init__(self, kind, source_system, source_component, sequence, fields):
        self.__dict__.update(fields)
        self._kind = kind
        self._source_system = source_system
        self._source_component = source_component
        self._sequence = sequence

    def get_type(self): return self._kind
    def get_srcSystem(self): return self._source_system
    def get_srcComponent(self): return self._source_component
    def get_seq(self): return self._sequence


class RosTransport:
    def __init__(self,node,component,system,rx_depth=20,kinds=None):
        from std_msgs.msg import String
        from .mavlink import mavutil
        if kinds is None:
            raise ValueError('ROS FCU consumers must declare their receive kinds')
        requested=frozenset(kinds)|{'HEARTBEAT'}
        unsupported=requested-FCU_RX_KINDS
        if unsupported:
            raise ValueError(f'unsupported FCU receive kinds: {sorted(unsupported)}')
        self.node,self.component,self.system=node,component,system
        self.String=String
        self.encoder=mavutil.mavlink.MAVLink(None,srcSystem=system,srcComponent=component)
        self.frames=collections.deque(maxlen=128)
        self.pending_latest={}
        self.pending_ordered=collections.deque(maxlen=128)
        self.receive_lock=threading.Lock()
        self.send_lock=threading.Lock()
        self.mav=SimpleNamespace(flightmode='UNKNOWN',close=self.close)
        self.kinds=requested
        self.publisher=node.create_publisher(String,'/graey/fcu/tx',100)
        self.subscriptions=[node.create_subscription(
            String,mav_topic(kind),lambda msg,kind=kind:self.receive_kind(kind,msg),
            min(max(int(rx_depth),1),20) if kind in ORDERED_RX_KINDS else 1)
            for kind in sorted(requested)]

    def receive_kind(self,kind,msg):
        with self.receive_lock:
            if kind in ORDERED_RX_KINDS:
                self.pending_ordered.append((kind,msg.data))
            else:
                self.pending_latest[kind]=msg.data

    def decode(self,raw,expected_kind=None):
        try:
            data=json.loads(raw)
            kind=data['kind']
            if expected_kind is not None and kind!=expected_kind:return None
            if kind not in self.kinds:return
            stamp=float(data['received_monotonic'])
            if not 0<=time.monotonic()-stamp<.25:return
            fields=dict(data['fields'])
            fields.pop('mavpackettype',None)
            frame=SemanticMessage(kind,int(data['source_system']),
                int(data['source_component']),int(data.get('sequence',0)),fields)
            if (kind=='HEARTBEAT' and frame.get_srcSystem()==FCU_SYSTEM
                    and frame.get_srcComponent()==FCU_COMPONENT):
                self.mav.flightmode=str(data.get('flightmode','UNKNOWN'))
            return stamp,frame
        except (ValueError,TypeError,KeyError):return

    def receive(self,msg):
        """Direct fixture/compatibility entry; production callbacks use receive_kind."""
        decoded=self.decode(msg.data)
        if decoded is not None:self.frames.append(decoded)

    def send(self,name,*args):
        if name=='heartbeat_send':return
        with self.send_lock:
            message=getattr(self.encoder,name.removesuffix('_send')+'_encode')(*args)
            raw=bytes(message.pack(self.encoder))
            self.encoder.seq=(self.encoder.seq+1)%256
        self.publisher.publish(self.String(data=json.dumps(dict(component=self.component,
            sent_monotonic=time.monotonic(),frame=raw.hex()))))

    def drain(self,handler):
        with self.receive_lock:
            pending=list(self.pending_ordered)+list(self.pending_latest.items())
            self.pending_ordered.clear();self.pending_latest.clear()
        for kind,raw in pending:
            decoded=self.decode(raw,kind)
            if decoded is not None:self.frames.append(decoded)
        deadline=time.monotonic()+.02
        while self.frames and time.monotonic()<deadline:
            stamp,message=self.frames.popleft()
            if 0<=time.monotonic()-stamp<.25:handler(message.get_type(),message)

    def close(self):
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.node.destroy_publisher(self.publisher)
