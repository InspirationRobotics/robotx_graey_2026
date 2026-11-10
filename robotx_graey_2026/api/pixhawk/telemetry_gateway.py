"""Crusader-style single ROS FCU gateway; MAVProxy remains the serial owner."""
import json
import time
from .gateway_core import GatewayPolicy


def main():
    from robotx_graey_2026.api.steady_node import SteadyNode as Node
    from robotx_graey_2026.api.node_util import run
    from std_msgs.msg import String
    from rcl_interfaces.msg import ParameterDescriptor
    from .mavlink import Link,mavutil

    class TelemetryGateway(Node):
        def __init__(self):
            super().__init__('telemetry_gateway')
            self.declare_parameter('mavlink','udpout:127.0.0.1:14551',ParameterDescriptor(read_only=True))
            self.declare_parameter('stream_ids',[24,29,30,32,33,49,65,137,143,193],ParameterDescriptor(read_only=True))
            self.declare_parameter('stream_hz',[10.,10.,20.,20.,10.,1.,10.,10.,10.,5.],ParameterDescriptor(read_only=True))
            self.rates=list(zip(self.get_parameter('stream_ids').value,self.get_parameter('stream_hz').value))
            if (len(self.get_parameter('stream_ids').value)!=len(self.get_parameter('stream_hz').value)
                    or len({mid for mid,_ in self.rates})!=len(self.rates)
                    or any(type(mid) is not int or not 0<=mid<=0xFFFFFF or not 0<hz<=50 for mid,hz in self.rates)):
                raise ValueError('stream IDs/rates must have equal lengths and rates within 0..50 Hz')
            # Match Crusader's default GCS identity; forwarded client frames
            # retain their own component IDs (including mission arbitration).
            self.link=Link(self.get_parameter('mavlink').value,0,self.get_logger(),system=255)
            self.policy=GatewayPolicy()
            self.decoder=mavutil.mavlink.MAVLink(None)
            self.received=self.transmitted=self.refused=0
            self.last_reason=None
            self.rx=self.create_publisher(String,'/graey/fcu/rx',200)
            self.status=self.create_publisher(String,'/graey/fcu/gateway_status',10)
            self.semantic={name:self.create_publisher(String,'/graey/fcu/'+name,30)
                           for name in ('pose','attitude','status','rc_channels')}
            self.create_subscription(String,'/graey/fcu/tx',self.tx,100)
            self.create_subscription(String,'/graey/control_watchdog/status',self.control,10)
            self.create_timer(.005,lambda:self.link.drain(self.receive))
            self.create_timer(1.,self.link.heartbeat)
            self.create_timer(.5,self.publish_status)
            self.create_timer(5.,self.request_streams)
            self.request_streams()

        def request_streams(self):
            for mid,hz in self.rates:self.link.command(511,mid,1e6/hz)

        def control(self,msg):
            try:self.policy.update_control(json.loads(msg.data),time.monotonic())
            except (ValueError,TypeError):pass

        def receive(self,kind,message):
            if kind=='BAD_DATA':return
            now=time.monotonic()
            data=dict(received_monotonic=now,frame=bytes(message.get_msgbuf()).hex())
            self.rx.publish(String(data=json.dumps(data)))
            self.received+=1
            name={'GLOBAL_POSITION_INT':'pose','ATTITUDE':'attitude','HEARTBEAT':'status','RC_CHANNELS':'rc_channels'}.get(kind)
            if name and message.get_srcSystem()==1 and message.get_srcComponent()==1:
                self.semantic[name].publish(String(data=json.dumps(dict(received_monotonic=now,fields=message.to_dict()),allow_nan=True)))

        def tx(self,msg):
            try:
                obj=json.loads(msg.data);now=time.monotonic()
                if not 0<=now-obj['sent_monotonic']<.2:raise ValueError('stale TX request')
                raw=bytes.fromhex(obj['frame'])
                if not 12<=len(raw)<=280:raise ValueError('invalid frame size')
                message=self.decoder.decode(bytearray(raw))
                if message is None:raise ValueError('incomplete frame')
                allowed,reason=self.policy.admit(message,obj['component'],now)
                if not allowed:raise ValueError(reason)
                if message.get_msgId()==76 and message.command==511:
                    return  # this gateway owns rates; consumers cannot fight over the FCU stream
                self.link.mav.write(raw)
                self.transmitted+=1
            except (ValueError,TypeError,KeyError,OSError,mavutil.mavlink.MAVError) as exc:
                self.refused+=1;self.last_reason=str(exc)

        def publish_status(self):
            self.status.publish(String(data=json.dumps(dict(received=self.received,transmitted=self.transmitted,
                refused=self.refused,last_refusal=self.last_reason,published_monotonic=time.monotonic(),
                serial_owner='MAVProxy',ros_owner='telemetry_gateway'))))

        def destroy_node(self):
            self.link.mav.close()
            return super().destroy_node()
    run(TelemetryGateway,ownership='telemetry_gateway')
