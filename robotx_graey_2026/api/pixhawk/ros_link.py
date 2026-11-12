"""Compatibility API over ROS FCU topics; consumers open no UDP or serial socket."""
import collections
import json
import threading
import time
from types import SimpleNamespace
from .identity import FCU_COMPONENT, FCU_SYSTEM


class RosTransport:
    def __init__(self,node,component,system,rx_depth=200,kinds=None):
        from std_msgs.msg import String
        from .mavlink import mavutil
        self.node,self.component,self.system=node,component,system
        self.String=String
        self.encoder=mavutil.mavlink.MAVLink(None,srcSystem=system,srcComponent=component)
        self.decoder=mavutil.mavlink.MAVLink(None)
        self.frames=collections.deque(maxlen=512)
        self.send_lock=threading.Lock()
        self.mav=SimpleNamespace(flightmode='UNKNOWN',close=self.close)
        # kinds: the message types this node reads (HEARTBEAT always, for flightmode). Others are skipped
        # before decoding. A frame without a 'kind' label (an older gateway) is decoded as before.
        self.kinds=None if kinds is None else frozenset(kinds)|{'HEARTBEAT'}
        self.publisher=node.create_publisher(String,'/graey/fcu/tx',100)
        # rx_depth: a reader that can fall behind keeps only the newest frames (Crusader's short queues).
        self.subscription=node.create_subscription(String,'/graey/fcu/rx',self.receive,rx_depth)

    def receive(self,msg):
        from .mavlink import mavutil
        try:
            data=json.loads(msg.data)
            kind=data.get('kind')
            if self.kinds is not None and kind is not None and kind not in self.kinds:return
            stamp=data['received_monotonic']
            if not 0<=time.monotonic()-stamp<.25:return
            frame=self.decoder.parse_char(bytes.fromhex(data['frame']))
            if frame is None:return
            if (frame.get_type()=='HEARTBEAT' and frame.get_srcSystem()==FCU_SYSTEM
                    and frame.get_srcComponent()==FCU_COMPONENT):
                self.mav.flightmode=mavutil.mode_string_v10(frame)
            self.frames.append((stamp,frame))
        except (ValueError,TypeError,KeyError,mavutil.mavlink.MAVError):return

    def send(self,name,*args):
        if name=='heartbeat_send':return
        with self.send_lock:
            message=getattr(self.encoder,name.removesuffix('_send')+'_encode')(*args)
            raw=bytes(message.pack(self.encoder))
            self.encoder.seq=(self.encoder.seq+1)%256
        self.publisher.publish(self.String(data=json.dumps(dict(component=self.component,
            sent_monotonic=time.monotonic(),frame=raw.hex()))))

    def drain(self,handler):
        deadline=time.monotonic()+.02
        while self.frames and time.monotonic()<deadline:
            stamp,message=self.frames.popleft()
            if 0<=time.monotonic()-stamp<.25:handler(message.get_type(),message)

    def close(self):
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_publisher(self.publisher)
