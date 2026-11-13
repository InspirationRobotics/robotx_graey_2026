"""SD competition reporting through shared ROS FCU telemetry and native OCS WiFi."""
import json
import sqlite3
import time

from .competition_wifi import CompetitionSender, WifiLink
from .competition_telemetry import CompetitionTelemetry
from ..pixhawk.mavlink import Link

COMPETITION_RX_KINDS=('GLOBAL_POSITION_INT','GPS_RAW_INT','ATTITUDE','HEARTBEAT')


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import ParameterDescriptor
    from std_msgs.msg import String
    from ..process_ownership import ProcessOwnership, executable_pids

    class CompetitionLinkNode(Node):
        def __init__(self):
            super().__init__('competition_link_node')
            for key,value in dict(ocs_host='auto',ocs_port=37564,team_id='ASTA',
                                  ocs_discovery_port=37565,
                                  outbox_path='/root/robotx_ws/logs/competition-outbox.sqlite3').items():
                self.declare_parameter(key,value,ParameterDescriptor(read_only=True,description='OCS session/identity/outbox configuration; restart required'))
            self.declare_parameter('ocs_fallback_hosts',rclpy.Parameter.Type.STRING_ARRAY,ParameterDescriptor(read_only=True,description='OCS dial targets; restart required'))
            if executable_pids('sub_link_node'):
                raise RuntimeError('stop the older sub_link_node before starting competition_link_node')
            param = lambda key:self.get_parameter_or(key).value
            self.wifi = WifiLink(param('ocs_host'),param('ocs_port'),team_id=param('team_id'),
                                 fallback_hosts=param('ocs_fallback_hosts'),discovery_port=param('ocs_discovery_port'))
            self.sender = CompetitionSender(param('outbox_path'),team_id=param('team_id'))
            self.telemetry = CompetitionTelemetry(time.monotonic)
            # CompetitionTelemetry -> NavigationView.consume reads only these.
            self.fcu = Link('ros://graey/fcu',191,self.get_logger(),node=self,
                            kinds=COMPETITION_RX_KINDS)
            self.error = None
            self.radio_run = None
            self.tiers_request = None
            self.publisher = self.create_publisher(String,'/graey/comms/status',10)
            self.create_subscription(String,'/graey/navigation/bridge_status',
                                     lambda m:self._status('bridge',m.data),10)
            self.create_subscription(String,'/graey/control_watchdog/status',
                                     lambda m:self._status('control',m.data),10)
            self.create_subscription(String,'/graey/comms/tiers_request',self._tiers,10)
            self.create_subscription(String,'/graey/comms/current_task',self._task,10)
            self.create_subscription(String,'/graey/comms/pipeline_survey',self._survey,10)
            self.create_subscription(String,'/graey/comms/resource_request',self._resource,10)
            self.create_subscription(String,'/graey/comms/radio_run',self._radio_run,10)
            self.create_timer(.05,self._tick)
            self.create_timer(.5,self._publish)

        def _guard(self, action):
            previous = self.error
            try:
                action()
                self.error = None
                if previous is not None:
                    self.get_logger().info('Competition input recovered')
            except (ValueError,TypeError,KeyError,OverflowError,OSError,sqlite3.Error) as exc:
                self.error = str(exc)
                if self.error != previous:
                    self.get_logger().error(self.error)

        def _status(self,name,raw):
            self._guard(lambda:self.telemetry.put(name,json.loads(raw)))

        def _tiers(self,msg):
            def send():
                data=json.loads(msg.data)
                stamp=data.get('requested_monotonic')
                if type(stamp) not in (int,float) or not 0<=time.monotonic()-stamp<1:
                    raise ValueError('Expired tier request; not sent')
                self.sender.tiers_reply=None
                self.wifi.send_tiers(data['tiers'],data.get('mission',''))
                self.tiers_request=dict(sent=True,error=None,requested_monotonic=stamp)
            try:send()
            except (ValueError,TypeError,KeyError,OSError) as exc:
                self.tiers_request=dict(sent=False,error=str(exc),requested_monotonic=time.monotonic())
                self.get_logger().error(str(exc))

        def _task(self,msg):
            self._guard(lambda:self.telemetry.set_task(msg.data))

        def _survey(self,msg):
            def record():
                data = json.loads(msg.data)
                p = data['active_buoy_position']
                self.sender.survey(data['observation_id'],p['latitude'],p['longitude'],data['segments'])
            self._guard(record)

        def _resource(self,msg):
            def record():
                data = json.loads(msg.data)
                self.sender.resource(data['observation_id'],data['resource_color'],data['delivery_circle_color'])
            self._guard(record)

        def _radio_run(self,msg):
            def receive():
                data=json.loads(msg.data);now=time.monotonic()
                stamp=data.get('received_monotonic')
                run=data.get('run_id')
                if (type(stamp) not in (int,float) or not 0<=now-stamp<2
                        or type(run) is not int or not 0<=run<=0xffff
                        or data.get('remote_state')!='RUNNING'
                        or data.get('run_confirmed') is not True
                        or data.get('source')!='rfd'):
                    raise ValueError('invalid or stale RFD RunStart')
                self.radio_run=data
            self._guard(receive)

        def _consume(self,kind,msg):
            self.telemetry.consume(kind,msg)

        def _tick(self):
            def tick():
                self.fcu.drain(self._consume)
                now = time.monotonic()
                self.telemetry.update(self.sender,now)
                self.wifi.tick(self.sender)
            self._guard(tick)

        def _publish(self):
            now=time.monotonic()
            data = self.sender.status(now)
            if self.radio_run is not None and 0<=now-self.radio_run['received_monotonic']<2:
                run=self.radio_run['run_id']
                data.update(run_id=run,remote_run_id=run,remote_state='RUNNING',run_confirmed=True,
                            run_transport='rfd',mission_run_id=self.radio_run.get('mission_run_id'),
                            run_started_monotonic=self.radio_run.get('run_started_monotonic',self.radio_run['received_monotonic']))
            data.update(wifi=self.wifi.status(),tiers_request=self.tiers_request,telemetry_hold_reason=self.telemetry.reason,last_error=self.error,
                        published_monotonic=now)
            self.publisher.publish(String(data=json.dumps(data,allow_nan=False)))

        def destroy_node(self):
            self.fcu.mav.close()
            self.wifi.close()
            self.sender.close()
            return super().destroy_node()

    with ProcessOwnership('competition_link_node'):
        rclpy.init(args=args)
        node = CompetitionLinkNode()
        try:rclpy.spin(node)
        except KeyboardInterrupt:pass
        finally:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
