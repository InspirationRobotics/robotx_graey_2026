#!/usr/bin/env python3
"""Graey's status onto the RFD900, and the operator's Record switches off it.

Chris, 12 Nov 2026 (marina test: no WiFi, so no GUI, no QGC, no ssh - the dock was blind).

    DVL, VN-100, kill_switch, Cube telemetry, camera + sonar tools
        -> ONE status frame, twice a second (api/radio/status_codec.py)
        -> /graey/fcu/tx -> telemetry_gateway -> MAVProxy -> Cube -> RFD900 -> the laptop page

    the laptop's Record buttons -> RFD900 -> Cube -> ... -> /graey/fcu/rx -> here
        -> api/radio/record_control.py keeps the camera stills and sonar sweeps recording

It moves nothing: no arming, no mode, no setpoint - it reports, and it switches recording.
The Cube forwards a broadcast frame to its radio port only once it has HEARD something
there (QGC or the laptop page on the mesh), so with nothing listening nothing is keyed.

Since Nov 12 the page's Missions tab comes through here too, and this node still moves nothing:
a mission start (dive here, surface route, or dock -> pipeline dive -> dock; radio_route.py), a
stop, or Start navigation (the Sensors page's, for when WiFi cannot reach) off the radio is handed to gui_node's own start path (api/radio/mission_relay.py), which makes
its usual checks and starts waypoint_mission. The mission's stage, point and ending ride in the
status frame.

The same status and switches are on http://<graey>:8085 for when WiFi or the tether IS up:
    GET  /status            everything in the frame, plus the recorders' error text
    POST /record            {"camera": true|false, "sonar": true|false} (leave one out to keep it)
    POST /mission           {"mission": {"kind": "dive", "dive": {"depth_m": 0.8}}} starts one, {"stop": true} stops it,
                            {"navigation": true} is Start navigation
    GET  /calibration       the team GUI's calibration state; POST /calibration one of its buttons
                            (calibrate, cancel, align_heading) - gui_node only listens on localhost
"""
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Bool, Float32, String

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link
from robotx_graey_2026.api.radio import status_codec as codec
from robotx_graey_2026.api.radio.mission_relay import MissionRelay
from robotx_graey_2026.api.radio.record_control import RecordControl
from robotx_graey_2026.api.radio.status_core import SURFACE, StatusCore
from robotx_graey_2026.api.steady_node import SteadyNode

COMPONENT = 199         # this node's MAVLink component of system 1; gateway_core admits its TUNNELs
KINDS = ('HEARTBEAT', 'ATTITUDE', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'SYS_STATUS', 'SCALED_PRESSURE3',
         'TUNNEL')      # all pump() reads


class RadioStatus(SteadyNode):
    def __init__(self):
        super().__init__('radio_status')
        self.declare_parameter('record_dir', os.path.expanduser('~/robotx_ws/recordings'))
        self.declare_parameter('http_port', 8085)
        self.declare_parameter('rate_hz', 2.0)

        try:
            self.link = Link('ros://graey/fcu', COMPONENT, self.get_logger(), node=self, kinds=KINDS)
        except TypeError:               # a tree from before Nov 11: Link takes no kinds, and filters nothing
            self.link = Link('ros://graey/fcu', COMPONENT, self.get_logger(), node=self)
        self.recorder = RecordControl(self.get_parameter('record_dir').value, lambda: self.core.nav(),
                                      log=lambda text: self.get_logger().info(text))
        self.core = StatusCore(self.recorder, surface_file=SURFACE,
                               missions=MissionRelay(log=lambda text: self.get_logger().info(text)))

        put = self.core.put
        self.create_subscription(TwistWithCovarianceStamped, '/graey/dvl/velocity', lambda m: put(
            'dvl_velocity', (m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z)), 10)
        self.create_subscription(Bool, '/graey/dvl/valid', lambda m: put('dvl_valid', m.data), 10)
        self.create_subscription(Float32, '/graey/dvl/altitude', lambda m: put('dvl_altitude', m.data), 10)
        self.create_subscription(Float32, '/graey/vn100/heading', lambda m: put('vn_heading', m.data), 10)
        self.create_subscription(String, '/graey/control_watchdog/status', lambda m: self.on_json('control', m), 10)
        self.create_subscription(String, '/graey/waypoint/status', lambda m: self.on_json('mission', m), 10)
        self.create_subscription(String, '/graey/navigation/bridge_status', lambda m: self.on_json('nav', m), 10)

        self.create_timer(0.05, self.pump)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.send)
        threading.Thread(target=self.recorder.run, daemon=True, name='radio-record').start()
        self.http = self.serve_http(self.get_parameter('http_port').value)
        self.get_logger().info('status frame on the radio; record switches on the radio and on :%d'
                               % self.get_parameter('http_port').value)

    def on_json(self, name, msg):
        """kill_switch's status ('control'), waypoint_mission's ('mission'), the nav bridge's ('nav')."""
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, dict):
            self.core.put(name, data)

    def pump(self):
        def consume(kind, m):
            if kind == 'TUNNEL':
                if m.payload_type == codec.PAYLOAD_COMMAND and self.core.on_command(codec.tunnel_body(m)):
                    self.get_logger().info('command %d off the radio: result %d, record wanted = %s'
                                           % (self.core.cmd_seq, self.core.cmd_result, self.recorder.want))
            elif m.get_srcSystem() == 1 and m.get_srcComponent() == 1:
                self.core.on_cube(kind, m)
        self.link.drain(consume)

    def send(self):
        payload = self.core.frame()
        self.link.send('tunnel_send', 0, 0, codec.PAYLOAD_STATUS, len(payload), payload.ljust(128, b'\0'))

    def serve_http(self, port):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def reply(self, code, data):
                body = json.dumps(data).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == '/calibration':
                    try:
                        return self.reply(*node.core.missions.calibration())
                    except (OSError, ValueError) as ex:
                        return self.reply(502, dict(ok=False, msg='the GUI node did not answer: %s' % ex))
                if self.path != '/status':
                    return self.send_error(404)
                self.reply(200, node.core.status())

            def do_POST(self):
                if self.path not in ('/record', '/mission', '/calibration'):
                    return self.send_error(404)
                try:
                    want = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0)) or 0) or b'{}')
                    if self.path == '/calibration':
                        return self.reply(*node.core.missions.calibration(want))
                    camera, sonar = want.get('camera'), want.get('sonar')
                    if not all(v is None or isinstance(v, bool) for v in (camera, sonar)):
                        raise ValueError
                except (ValueError, AttributeError):
                    return self.reply(400, dict(ok=False, message='send {"camera": true|false, "sonar": true|false}'))
                if self.path == '/mission':
                    ok = (node.core.stop_mission() if want.get('stop') is True
                          else node.core.start_navigation() if want.get('navigation') is True
                          else node.core.start_mission(want.get('mission')))
                    return self.reply(200 if ok else 409, dict(ok=ok, note=node.core.refused))
                node.recorder.set(camera, sonar)
                self.reply(200, dict(ok=True, want=node.recorder.want))

            def log_message(self, *a):
                pass

        try:
            server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
        except OSError as ex:           # the radio is the point; the page on WiFi is a convenience
            self.get_logger().error(f'port {port} is taken, so no status over WiFi: {ex}')
            return None
        threading.Thread(target=server.serve_forever, daemon=True, name='radio-http').start()
        return server

    def destroy_node(self):
        self.recorder.close()
        if self.http:
            self.http.shutdown()
            self.http.server_close()
        return super().destroy_node()


def main():
    run(RadioStatus)
