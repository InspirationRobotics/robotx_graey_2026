"""Submarine publisher: real mission producers supply JSON; this node only reports.

No sensor acquisition or vehicle actions. Run explicitly with ros2 run until
the pipeline mission is integrated. No report exists before a producer supplies it.
"""
import json
import time

from .mesh_link import MeshLink, DEFAULT_ENDPOINT, DEFAULT_SYSTEM, DEFAULT_COMPONENT
from .sub_link import Sender


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, UInt32, Bool

    class SubLinkNode(Node):
        def __init__(self):
            super().__init__('sub_link_node')
            self.declare_parameter('endpoint', DEFAULT_ENDPOINT)
            self.declare_parameter('source_system', DEFAULT_SYSTEM)
            self.declare_parameter('source_component', DEFAULT_COMPONENT)
            self.declare_parameter('outbox_path', '/root/robotx_ws/logs/sub-comms-outbox.json')
            self.mesh = MeshLink(self.get_parameter('endpoint').value,
                                 self.get_parameter('source_system').value,
                                 self.get_parameter('source_component').value)
            self.sender = Sender(outbox_path=self.get_parameter('outbox_path').value)
            self.received = self.create_publisher(String, '/graey/comms/received', 50)
            self.create_subscription(UInt32, '/graey/comms/run_id',
                                     self._run_input, 10)
            self.create_subscription(Bool, '/graey/comms/submerged', self._submerged_input, 10)
            for topic, kind in (('status', 'status'), ('pipeline_survey', 'pipeline_survey'),
                                ('resource_request', 'resource_request')):
                self.create_subscription(String, '/graey/comms/' + topic,
                                         lambda m, k=kind: self._input(k, m.data), 10)
            self.create_timer(.1, self._tick)
            self.create_timer(1.0, self.mesh.heartbeat)

        def _run_input(self, msg):
            try:
                self.sender.start_run(msg.data)
            except (OSError, ValueError) as exc:
                self.get_logger().error(f'run/outbox update failed: {exc}')

        def _submerged_input(self, msg):
            try:
                self.sender.set_submerged(msg.data)
            except (OSError, ValueError) as exc:
                self.get_logger().error(f'submerged/outbox update failed: {exc}')

        def _input(self, kind, raw):
            try:
                data = json.loads(raw)
                if kind == 'status':
                    self.sender.update_status(data, time.monotonic())
                elif kind == 'pipeline_survey':
                    self.sender.survey_complete(data['active_buoy_position'], data['segments'])
                else:
                    self.sender.resource_observed(data['resource_color'], data['delivery_circle_color'])
            except (ValueError, TypeError, KeyError, OverflowError, OSError) as exc:
                self.get_logger().error(f'{kind} not published: {exc}')

        def _tick(self):
            for payload_type, payload in self.sender.due(time.monotonic()):
                self.mesh.send_tunnel(payload_type, payload)
            for record in self.mesh.poll():
                self.received.publish(String(data=json.dumps(record)))

        def destroy_node(self):
            self.mesh.close()
            return super().destroy_node()

    rclpy.init(args=args)
    node = SubLinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
