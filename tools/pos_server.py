#!/usr/bin/env python3
"""Serve Graey's live position as JSON for pool_planner.html.

GET /pos -> {"ok":true,"x":..,"y":..,"z":..,"yaw":..,"age":..}   (NED, radians)
Reads LOCAL_POSITION_NED + ATTITUDE from MAVProxy (udpin 14554).
Run inside the container; planner polls http://<jetson-ip>:8081/pos"""
import json
import os
import threading
import time
os.environ['MAVLINK20'] = '1'
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pymavlink import mavutil

state = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 't': 0.0}


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
            state['x'] = msg.x
            state['y'] = msg.y
            state['z'] = msg.z
            state['t'] = time.time()
        elif t == 'ATTITUDE':
            state['yaw'] = msg.yaw
            state['t'] = time.time()


class H(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/pos':
            self.send_error(404)
            return
        age = round(time.time() - state['t'], 2) if state['t'] else -1
        body = json.dumps({'ok': state['t'] > 0, 'x': state['x'],
                           'y': state['y'], 'z': state['z'],
                           'yaw': state['yaw'], 'age': age}).encode()
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
    print('pos server on http://0.0.0.0:8081/pos')
    ThreadingHTTPServer(('0.0.0.0', 8081), H).serve_forever()
