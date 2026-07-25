#!/usr/bin/env python3
"""Serve Graey's live position AND the CV pole fix as JSON for pool_planner.html.

GET /pos -> {"ok":..,"x":..,"y":..,"z":..,"yaw":..,"age":..,
             "pole_ok":..,"pole_n":..,"pole_e":..,"pole_fwd":..,"pole_right":..}

Position/attitude come from MAVProxy (udpin 14554). The pole sighting comes from
pole_tracker via ROS (/graey/pole, body frame) and is converted to world NED
using the position+yaw at the moment it was seen.
Run inside the container; planner polls http://<jetson-ip>:8081/pos"""
import json
import os
import threading
import time
os.environ['MAVLINK20'] = '1'
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from pymavlink import mavutil

state = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 't': 0.0,
         'pole_ok': False, 'pole_n': 0.0, 'pole_e': 0.0,
         'pole_fwd': 0.0, 'pole_right': 0.0, 'pole_t': 0.0}


def pump():
    m = mavutil.mavlink_connection('udpout:127.0.0.1:14554',
                                   source_system=1, source_component=191)
    last_hb = 0.0
    while True:
        if time.time() - last_hb > 0.5:
            m.mav.heartbeat_send(6, 8, 0, 0, 0)
            last_hb = time.time()
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'LOCAL_POSITION_NED':
            state['x'], state['y'], state['z'] = msg.x, msg.y, msg.z
            state['t'] = time.time()
        elif t == 'ATTITUDE':
            state['yaw'] = msg.yaw
            state['t'] = time.time()


class PoleListener(Node):
    def __init__(self):
        super().__init__('pos_server_pole')
        self.create_subscription(Bool, '/graey/pole/visible', self.on_vis, 10)
        self.create_subscription(PointStamped, '/graey/pole', self.on_pole, 10)

    def on_vis(self, msg):
        state['pole_ok'] = bool(msg.data)

    def on_pole(self, msg):
        f, r = msg.point.x, msg.point.y
        c, s = math.cos(state['yaw']), math.sin(state['yaw'])
        state['pole_fwd'], state['pole_right'] = f, r
        state['pole_n'] = state['x'] + f * c - r * s
        state['pole_e'] = state['y'] + f * s + r * c
        state['pole_t'] = time.time()


class H(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/pos':
            self.send_error(404)
            return
        age = round(time.time() - state['t'], 2) if state['t'] else -1
        fresh = state['pole_ok'] and (time.time() - state['pole_t'] < 2.0)
        body = json.dumps({
            'ok': state['t'] > 0, 'x': state['x'], 'y': state['y'],
            'z': state['z'], 'yaw': state['yaw'], 'age': age,
            'pole_ok': fresh, 'pole_n': state['pole_n'], 'pole_e': state['pole_e'],
            'pole_fwd': state['pole_fwd'], 'pole_right': state['pole_right'],
        }).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


if __name__ == '__main__':
    threading.Thread(target=pump, daemon=True).start()
    threading.Thread(target=lambda: ThreadingHTTPServer(
        ('0.0.0.0', 8081), H).serve_forever(), daemon=True).start()
    print('pos server on http://0.0.0.0:8081/pos  (with CV pole)')
    rclpy.init()
    node = PoleListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
