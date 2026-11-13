"""Crusader-style single FCU decode gateway; MAVProxy owns Cube serial."""
import json
import time
from .gateway_core import GatewayPolicy
from .ros_link import FCU_RX_KINDS, mav_topic
from .identity import FCU_COMPONENT, FCU_SYSTEM


# The GUI needs current vehicle state, not every intermediate sensor frame.
# Crusader's bridge has the same latest-value boundary before its GCS.
GUI_SNAPSHOT_KINDS=frozenset({
    'ATTITUDE','EKF_STATUS_REPORT','GLOBAL_POSITION_INT','GPS_GLOBAL_ORIGIN',
    'GPS_RAW_INT','LOCAL_POSITION_NED',
})


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
            self.link=Link(self.get_parameter('mavlink').value,0,self.get_logger(),system=255)
            self.policy=GatewayPolicy()
            self.decoder=mavutil.mavlink.MAVLink(None)
            self.received=self.transmitted=self.refused=self.ignored=0
            self.last_reason=None
            # ``Node.publishers`` is a read-only rclpy graph-introspection
            # property.  Keep our per-message routing table under a private
            # name so the gateway can run on the vehicle's real ROS runtime.
            self._mav_publishers={kind:self.create_publisher(String,mav_topic(kind),20)
                                  for kind in FCU_RX_KINDS}
            self.gui_latest={}
            self.gui_snapshot=self.create_publisher(String,'/graey/fcu/gui_snapshot',1)
            self.status=self.create_publisher(String,'/graey/fcu/gateway_status',10)
            self.create_subscription(String,'/graey/fcu/tx',self.tx,100)
            self.create_subscription(String,'/graey/control_watchdog/status',self.control,10)
            # 100 Hz is above the configured aggregate receive cadence while
            # avoiding the needless 200 Hz empty polling loop.
            self.create_timer(.01,lambda:self.link.drain(self.receive))
            self.create_timer(1.,self.link.heartbeat)
            self.create_timer(.5,self.publish_status)
            # Crusader's ground-station state is projected at 2 Hz.  Five Hz
            # keeps the GRAEY pose pointer smooth without making the Python GUI
            # parse ten copies per second of the same latest FCU state.
            self.create_timer(.2,self.publish_gui_snapshot)
            self.create_timer(5.,self.request_streams)
            self.request_streams()

        def request_streams(self):
            for mid,hz in self.rates:self.link.command(511,mid,1e6/hz)

        def control(self,msg):
            try:self.policy.update_control(json.loads(msg.data),time.monotonic())
            except (ValueError,TypeError):pass

        def receive(self,kind,message):
            if kind=='BAD_DATA':return
            publisher=self._mav_publishers.get(kind)
            if publisher is None:
                self.ignored+=1
                return
            now=time.monotonic()
            data=dict(
                received_monotonic=now,kind=kind,
                source_system=message.get_srcSystem(),
                source_component=message.get_srcComponent(),
                sequence=message.get_seq(),
                flightmode=mavutil.mode_string_v10(message) if kind=='HEARTBEAT' else None,
                fields=message.to_dict())
            if (kind in GUI_SNAPSHOT_KINDS and message.get_srcSystem()==FCU_SYSTEM
                    and message.get_srcComponent()==FCU_COMPONENT):
                self.gui_latest[kind]=data
            # Do not serialize and fan out high-rate topics nobody currently
            # consumes. The GUI uses the bounded snapshot below.
            if publisher.get_subscription_count():
                publisher.publish(String(data=json.dumps(data,allow_nan=True,default=list)))
            self.received+=1

        def publish_gui_snapshot(self):
            now=time.monotonic()
            messages={kind:data for kind,data in self.gui_latest.items()
                      if 0<=now-data['received_monotonic']<3.}
            self.gui_snapshot.publish(String(data=json.dumps(dict(
                published_monotonic=now,messages=messages),allow_nan=True,default=list)))

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
                    return
                self.link.mav.write(raw)
                self.transmitted+=1
            except (ValueError,TypeError,KeyError,OSError,mavutil.mavlink.MAVError) as exc:
                self.refused+=1;self.last_reason=str(exc)

        def publish_status(self):
            self.status.publish(String(data=json.dumps(dict(received=self.received,ignored=self.ignored,
                transmitted=self.transmitted,refused=self.refused,last_refusal=self.last_reason,
                published_monotonic=time.monotonic(),serial_owner='MAVProxy',
                ros_owner='telemetry_gateway',receive_format='semantic-per-kind'))))

        def destroy_node(self):
            self.link.mav.close()
            return super().destroy_node()
    run(TelemetryGateway,ownership='telemetry_gateway')
