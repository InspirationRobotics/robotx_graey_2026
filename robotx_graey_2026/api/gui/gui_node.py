#!/usr/bin/env python3
"""Graey control GUI -> http://<jetson-ip>:8090

Tabs: stack control, sensor health, camera, mission planner. A status strip
across the top mirrors what the vehicle is actually doing. Any laptop on the
router or the tether can open it, no install needed.

Nodes are grouped into stacks because they are only useful together - the EKF
bridge needs both the DVL and the VN-100, and the two LED nodes are one chain.

Stack state comes from scanning /proc for our own executables rather than asking
ROS: instant, needs no ros2 daemon, and sees nodes started by anything - the
launch file, systemd, or a terminal.

A stack stopped here is NOT restarted by the core.launch that owned it; starting
it again from this page runs it standalone until graey-ros restarts.
"""
import json
import os
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link, mavutil

CAMERA = 'pole_tracker'                         # the CV mission is blind without it

GROUPS = {                                      # key: (label, executables)
    'led':     ('LED stack',        ['led_node', 'pixhawk_led_node']),
    'nav':     ('Nav / EKF stack',  ['dvl_node', 'vn100_node', 'nav_ekf_bridge']),
    'camera':  ('Camera tracker',   [CAMERA]),
    'planner': ('Planner feed',     ['pos_server']),
}                                               # gui_node is deliberately absent -
                                                # stopping it would kill this page
READ_ONLY = {'mavproxy': ('MAVProxy', ['mavproxy.py'])}   # systemd owns it
FILES = {'/': 'gui.html', '/planner': 'pool_planner.html',
         '/pool.png': 'pool.png',                   # lab pool
         '/neighbor_pool.png': 'neighbor_pool.png'}  # prequal venue, scaled off satellite
TYPES = {'.html': 'text/html', '.png': 'image/png'}
LED_NAMES = {0: 'off', 1: 'RED', 2: 'YELLOW', 3: 'GREEN'}

MISSION = 'prequal_mission_cv'
MISSION_LOG = '/root/robotx_ws/logs/mission.log'
# Parameters are clamped to these ranges and rebuilt into argv here. The browser
# never supplies a command string - nothing it sends is ever executed as text.
MISSION_PARAMS = {'depth': (0.0, 10.0),
                  'gate_forward': (0.0, 50.0),
                  'marker_forward': (-50.0, 50.0),
                  'marker_right': (-50.0, 50.0),
                  'orbit_radius': (0.25, 10.0),
                  'orbit_speed': (0.05, 1.0),
                  'reach_thresh': (0.05, 2.0),
                  'cv_blind': (0.0, 60.0),
                  'state_timeout': (5.0, 300.0)}

# The GUI runs in a container and cannot shut the Jetson down. It drops a file in
# the bind mount instead; systemd .path units on the host watch for these and act.
REQUESTS = {'shutdown': '/root/robotx_ws/logs/shutdown.request',
            'reboot': '/root/robotx_ws/logs/reboot.request'}

ROS_LOG = '/root/robotx_ws/logs/graey-ros.log'
NOISY = (b'ERROR', b'WARN', b'Traceback', b'Error', b'error')

web_dir = '/root/robotx_ws/src/robotx_graey_2026/tools'
link = None                                     # set by GuiNode, used to disarm on stop

# Timestamps are absolute; the API reports them as an age so the page can grey
# out anything that has gone quiet.
tel = {'led': None, 'led_t': 0.0,
       'armed': False, 'mode': '?', 'mav_t': 0.0, 'depth': 0.0,
       'dvl_lock': False, 'altitude': -1.0, 'dvl_t': 0.0,
       'heading': None, 'hdg_t': 0.0}


def pids_for(name):
    """PIDs whose argv holds an exact executable match - 'led_node' must not
    match 'pixhawk_led_node', so compare whole path components."""
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == os.getpid():
            continue
        try:
            with open('/proc/' + entry + '/cmdline', 'rb') as f:
                parts = f.read().decode('utf-8', 'replace').split('\0')
        except OSError:
            continue                            # process exited while we looked
        if any(p == name or p.endswith('/' + name) for p in parts):
            found.append(int(entry))
    return found


def spawn(argv, out=subprocess.DEVNULL):
    """Start something detached, so it outlives the request that asked for it."""
    subprocess.Popen(argv, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)


def tail(path, nbytes=60000):
    """Last chunk of a file without reading all of it - these grow unbounded."""
    with open(path, 'rb') as f:
        f.seek(0, 2)
        f.seek(max(0, f.tell() - nbytes))
        return f.read().split(b'\n')[1:]        # drop the partial first line


def describe(key, label, execs):
    up = sum(1 for e in execs if pids_for(e))
    return {'key': key, 'label': label, 'nodes': execs, 'up': up, 'total': len(execs)}


def status():
    now = time.time()
    seen = lambda t: round(now - t, 1) if t else -1
    return {'led': LED_NAMES.get(tel['led'], '--'), 'led_age': seen(tel['led_t']),
            'armed': tel['armed'], 'mode': tel['mode'], 'mav_age': seen(tel['mav_t']),
            'depth': round(tel['depth'], 2),
            'dvl_lock': tel['dvl_lock'], 'altitude': round(tel['altitude'], 2),
            'dvl_age': seen(tel['dvl_t']),
            'heading': tel['heading'], 'hdg_age': seen(tel['hdg_t']),
            'mission': bool(pids_for(MISSION))}


def mission_start(query):
    """Build argv from clamped floats. Returns (ok, message).

    The mission is only a CV mission while pole_tracker is up. Without it the
    node never sees /graey/pole, quietly flies the planner coordinates and still
    reports success - so a wet run is refused until the camera is actually
    running. A dry run starts it and carries on, which warms the model up during
    exactly the minute you spend reading waypoints. Untick "use CV" in the planner
    and none of that applies: the mission is then deliberately dead-reckoned.
    """
    if pids_for(MISSION):
        return False, 'a mission is already running'
    dry = query.get('dry', ['true'])[0] == 'true'
    rc = query.get('rc_start', ['false'])[0] == 'true'
    cv = query.get('use_cv', ['true'])[0] == 'true'

    cv_up = bool(pids_for(CAMERA))
    if cv and not cv_up:
        spawn(['ros2', 'run', 'robotx_graey_2026', CAMERA])
        if not dry:
            return False, (CAMERA + ' was not running - started it. Wait for the Camera '
                           'tab to show video, then run again.')

    argv = ['ros2', 'run', 'robotx_graey_2026', MISSION, '--ros-args',
            '-p', 'dry_run:=' + ('true' if dry else 'false'),
            '-p', 'rc_start:=' + ('true' if rc else 'false'),
            '-p', 'use_cv:=' + ('true' if cv else 'false')]
    for name, (lo, hi) in MISSION_PARAMS.items():
        raw = query.get(name)
        if not raw:
            continue
        try:
            value = float(raw[0])
        except ValueError:
            return False, 'bad value for ' + name
        value = max(lo, min(hi, value))         # clamp, never reject silently
        argv += ['-p', name + ':=' + str(value)]   # str(float) keeps ROS's DOUBLE type
    os.makedirs(os.path.dirname(MISSION_LOG), exist_ok=True)
    spawn(argv, open(MISSION_LOG, 'wb'))
    if rc:
        return True, 'armed and waiting - press SE on the transmitter to start'
    if dry:
        return True, 'dry run started' + ('' if cv_up or not cv else ' - camera starting too')
    return True, 'MISSION STARTED'


def mission_stop():
    """Disarm FIRST, then kill.

    Killing first leaves GUIDED with no setpoint for a moment, which is the
    forward-lurch case. And SIGTERM alone is not enough: rclpy installs its own
    SIGTERM handler that calls rclpy.shutdown(), which does not reliably end a
    spinning node - so escalate to SIGKILL rather than trust it.
    """
    if link:
        link.disarm()
    for pid in pids_for(MISSION):
        os.kill(pid, signal.SIGTERM)
    time.sleep(1.0)
    survivors = pids_for(MISSION)
    for pid in survivors:
        os.kill(pid, signal.SIGKILL)
    return True, ('mission stopped and disarmed'
                  + (' (needed SIGKILL)' if survivors else ''))


class Handler(BaseHTTPRequestHandler):
    def reply(self, code, body, ctype):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)

        if u.path == '/api/status':
            self.reply(200, json.dumps(status()).encode(), 'application/json')
            return

        if u.path == '/api/groups':
            self.reply(200, json.dumps({
                'groups': [describe(k, lbl, ex) for k, (lbl, ex) in GROUPS.items()],
                'readonly': [describe(k, lbl, ex) for k, (lbl, ex) in READ_ONLY.items()],
            }).encode(), 'application/json')
            return

        if u.path == '/api/log':
            try:
                lines = tail(ROS_LOG)
            except OSError:
                lines = [b'no log yet - graey-ros writes it on start']
            if parse_qs(u.query).get('errors', ['0'])[0] == '1':
                lines = [ln for ln in lines if any(k in ln for k in NOISY)] \
                        or [b'(no warnings or errors)']
            self.reply(200, b'\n'.join(lines[-250:]), 'text/plain')
            return

        if u.path == '/api/power':
            what = parse_qs(u.query).get('do', [''])[0]
            if what not in REQUESTS:
                self.send_error(400, 'unknown power action')
                return
            os.makedirs(os.path.dirname(REQUESTS[what]), exist_ok=True)
            with open(REQUESTS[what], 'w') as f:
                f.write(str(time.time()))
            self.reply(200, json.dumps(
                {'ok': True, 'msg': what + ' requested - the Jetson acts within a few '
                                           'seconds. Wait for the lights to go out before '
                                           'disconnecting batteries.'}).encode(),
                'application/json')
            return

        if u.path == '/api/mission/log':
            try:
                with open(MISSION_LOG, 'rb') as f:
                    body = b''.join(f.readlines()[-120:])
            except OSError:
                body = b'no mission has been run yet'
            self.reply(200, body, 'text/plain')
            return

        if u.path in ('/api/mission/start', '/api/mission/stop'):
            if u.path.endswith('start'):
                ok, msg = mission_start(parse_qs(u.query))
            else:
                ok, msg = mission_stop()
            self.reply(200 if ok else 409,
                       json.dumps({'ok': ok, 'msg': msg}).encode(), 'application/json')
            return

        if u.path in ('/api/start', '/api/stop'):
            key = parse_qs(u.query).get('group', [''])[0]
            if key not in GROUPS:               # allowlist - never exec arbitrary input
                self.send_error(400, 'unknown group')
                return
            for name in GROUPS[key][1]:
                if u.path == '/api/stop':
                    for pid in pids_for(name):
                        os.kill(pid, signal.SIGTERM)
                elif not pids_for(name):        # start is a no-op if already up
                    spawn(['ros2', 'run', 'robotx_graey_2026', name])
            self.reply(200, b'{"ok":true}', 'application/json')
            return

        fname = FILES.get(u.path)
        if not fname:
            self.send_error(404)
            return
        try:
            with open(os.path.join(web_dir, fname), 'rb') as f:
                self.reply(200, f.read(), TYPES[os.path.splitext(fname)[1]])
        except OSError:
            self.send_error(404, fname + ' not found in ' + web_dir)

    def log_message(self, *a):
        pass


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        global web_dir, link
        self.declare_parameter('port', 8090)
        self.declare_parameter('web_dir', web_dir)
        self.declare_parameter('mavlink', 'udpout:127.0.0.1:14555')
        web_dir = self.get_parameter('web_dir').value
        port = self.get_parameter('port').value

        self.link = link = Link(self.get_parameter('mavlink').value, 195,
                                self.get_logger())
        self.create_subscription(Int32, '/graey/led_state', self.on_led, 10)
        self.create_subscription(Bool, '/graey/dvl/valid', self.on_dvl, 10)
        self.create_subscription(Float32, '/graey/dvl/altitude', self.on_alt, 10)
        self.create_subscription(Float32, '/graey/vn100/heading', self.on_hdg, 10)
        self.create_timer(1.0, self.link.heartbeat)
        self.create_timer(0.1, self.pump)

        threading.Thread(target=lambda: ThreadingHTTPServer(
            ('0.0.0.0', port), Handler).serve_forever(), daemon=True).start()
        self.get_logger().info('GUI on http://0.0.0.0:' + str(port))

    def on_led(self, m):
        tel['led'], tel['led_t'] = m.data, time.time()

    def on_dvl(self, m):
        tel['dvl_lock'], tel['dvl_t'] = m.data, time.time()

    def on_alt(self, m):
        tel['altitude'] = m.data

    def on_hdg(self, m):
        tel['heading'], tel['hdg_t'] = m.data, time.time()

    def pump(self):
        def handle(kind, m):
            if kind == 'HEARTBEAT' and m.get_srcComponent() == 1:
                tel['armed'] = bool(m.base_mode
                                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                tel['mode'] = self.link.mav.flightmode
                tel['mav_t'] = time.time()
            elif kind == 'LOCAL_POSITION_NED':
                tel['depth'] = m.z              # NED: positive down = depth
        self.link.drain(handle)


def main():
    run(GuiNode)
