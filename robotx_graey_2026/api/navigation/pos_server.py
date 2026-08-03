#!/usr/bin/env python3
"""Serve Graey's live position and the CV pole fix as JSON for pool_planner.html.

GET /pos -> {"ok","x","y","z","yaw","age","pole_ok","pole_n","pole_e",
             "pole_fwd","pole_right"}

Position and attitude come from MAVProxy (udpin 14554); the pole sighting comes
from pole_tracker over ROS and is converted to world NED using the position and
yaw at the moment it was seen. The planner polls http://<jetson-ip>:8081/pos
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.navigation.frames import body_to_world
from robotx_graey_2026.api.pixhawk.mavlink import Link

POLE_STALE_S = 2.0                                  # older than this reports pole_ok false

state = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 't': 0.0,
         'pole_ok': False, 'pole_n': 0.0, 'pole_e': 0.0,
         'pole_fwd': 0.0, 'pole_right': 0.0, 'pole_t': 0.0}


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/pos':
            self.send_error(404)
            return
        fresh = state['pole_ok'] and time.time() - state['pole_t'] < POLE_STALE_S
        body = json.dumps({
            'ok': state['t'] > 0,
            'x': state['x'], 'y': state['y'], 'z': state['z'], 'yaw': state['yaw'],
            'age': round(time.time() - state['t'], 2) if state['t'] else -1,
            'pole_ok': fresh, 'pole_n': state['pole_n'], 'pole_e': state['pole_e'],
            'pole_fwd': state['pole_fwd'], 'pole_right': state['pole_right'],
        }).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')   # planner loads from file://
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


class PosServer(Node):
    def __init__(self):
        super().__init__('pos_server')
        self.declare_parameter('mavlink', 'udpout:127.0.0.1:14554')
        self.declare_parameter('port', 8081)
        self.link = Link(self.get_parameter('mavlink').value, 191, self.get_logger())

        self.create_subscription(Bool, '/graey/pole/visible', self.on_vis, 10)
        self.create_subscription(PointStamped, '/graey/pole', self.on_pole, 10)
        self.create_timer(0.5, self.link.heartbeat)
        self.create_timer(0.05, self.pump)

        port = self.get_parameter('port').value
        threading.Thread(target=lambda: ThreadingHTTPServer(
            ('0.0.0.0', port), Handler).serve_forever(), daemon=True).start()
        self.get_logger().info(f'pos server on http://0.0.0.0:{port}/pos')

    def on_vis(self, msg):
        state['pole_ok'] = bool(msg.data)

    def on_pole(self, msg):
        f, r = msg.point.x, msg.point.y
        state['pole_fwd'], state['pole_right'] = f, r
        state['pole_n'], state['pole_e'] = body_to_world(
            state['x'], state['y'], state['yaw'], f, r)
        state['pole_t'] = time.time()

    def pump(self):
        def handle(kind, m):
            if kind == 'LOCAL_POSITION_NED':
                state['x'], state['y'], state['z'] = m.x, m.y, m.z
                state['t'] = time.time()
            elif kind == 'ATTITUDE':
                state['yaw'] = m.yaw
                state['t'] = time.time()
        self.link.drain(handle)


def main():
    run(PosServer)
