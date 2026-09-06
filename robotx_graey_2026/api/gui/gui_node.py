#!/usr/bin/env python3
"""Graey GUI, including RF/tether failsafe selection.

The existing tools/gui.html is served with an additional control panel.
No modification to gui.html or additional ROS message package is required.

Safety requests and acknowledgements use std_msgs/String JSON messages.
Only kill_switch owns the selected watchdog and recovery state.
"""

import json
import os
import queue
import signal
import subprocess
import threading
import time
import uuid

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32, String

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link, mavutil


CAMERA = 'pole_tracker'

GROUPS = {
    'led': ('LED stack', ['led_node', 'pixhawk_led_node']),
    'nav': ('Nav / EKF stack',
            ['dvl_node', 'vn100_node', 'nav_ekf_bridge']),
    'camera': ('Camera tracker', [CAMERA]),
    'planner': ('Planner feed', ['pos_server']),
}
READ_ONLY = {
    'mavproxy': ('MAVProxy', ['mavproxy.py']),
}
FILES = {
    '/': 'gui.html',
    '/planner': 'pool_planner.html',
    '/pool.png': 'pool.png',
    '/neighbor_pool.png': 'neighbor_pool.png',
}
TYPES = {
    '.html': 'text/html',
    '.png': 'image/png',
}
LED_NAMES = {0: 'off', 1: 'RED', 2: 'YELLOW', 3: 'GREEN'}
LED_DRIVER = 'led_node'
LED_SOURCE = 'pixhawk_led_node'

MISSION = 'prequal_mission_cv'
MISSION_EXECUTABLES = (
    'prequal_mission',
    'prequal_mission_cv',
    'demo_mission',
)
MISSION_LOG = '/root/robotx_ws/logs/mission.log'
MISSION_PARAMS = {
    'depth': (0.0, 10.0),
    'gate_forward': (0.0, 50.0),
    'marker_forward': (-50.0, 50.0),
    'marker_right': (-50.0, 50.0),
    'orbit_radius': (0.25, 10.0),
    'orbit_speed': (0.05, 2.0),
    'reach_thresh': (0.05, 2.0),
    'cv_blind': (0.0, 60.0),
    'state_timeout': (5.0, 300.0),
}
# Preserve the repository's original orbit-speed limit.
MISSION_PARAMS['orbit_speed'] = (0.05, 1.0)

REQUESTS = {
    'shutdown': '/root/robotx_ws/logs/shutdown.request',
    'reboot': '/root/robotx_ws/logs/reboot.request',
}
ROS_LOG = '/root/robotx_ws/logs/graey-ros.log'
NOISY = (b'ERROR', b'WARN', b'Traceback', b'Error', b'error')

CONTROL_REQUEST_TOPIC = '/graey/control_watchdog/request'
CONTROL_REPLY_TOPIC = '/graey/control_watchdog/reply'
CONTROL_STATUS_TOPIC = '/graey/control_watchdog/status'
CONTROL_STATUS_TIMEOUT = 2.0
CONTROL_REQUEST_LIFETIME = 2.0

web_dir = '/root/robotx_ws/src/robotx_graey_2026/tools'
link = None
led_off = False
control_bridge = None

# Serializes GUI-originated mission changes and watchdog changes.
operation_lock = threading.Lock()

tel = {
    'led': None,
    'led_t': 0.0,
    'armed': False,
    'mode': '?',
    'mav_t': 0.0,
    'depth': 0.0,
    'dvl_lock': False,
    'altitude': -1.0,
    'dvl_t': 0.0,
    'heading': None,
    'hdg_t': 0.0,
}


# Added to the existing HTML response, without writing to gui.html.
CONTROL_PANEL = r"""
<section id="watchdogPanel"
 style="margin:14px 18px;padding:14px;border:1px solid #465268;
 border-radius:8px;background:#1d2129;color:#e8e8e8">
  <strong>Controller connection failsafe</strong>

  <div style="margin-top:10px;display:flex;gap:8px;flex-wrap:wrap">
    <button class="act" id="watchdogRF"
      onclick="watchdogAction('select','RF')">RF</button>
    <button class="act" id="watchdogTether"
      onclick="watchdogAction('select','TETHER')">
      Logitech via tether
    </button>
    <button class="act" id="watchdogRecover"
      onclick="watchdogAction('recover','TETHER')">
      Recover tether control
    </button>
  </div>

  <div id="watchdogState" style="margin-top:10px">
    Waiting for safety node...
  </div>
  <div id="watchdogLinks" style="margin-top:6px"></div>
  <div id="watchdogMessage"
       style="margin-top:8px;white-space:pre-wrap"></div>

  <div style="margin-top:10px;font-size:12px;color:#bbc">
    Both controllers remain accepted. The selection changes only the
    connection watchdog. Tether mode checks laptop reachability, not
    QGroundControl or Logitech health. Recovery does not arm the vehicle.
  </div>
</section>

<script>
let watchdogBusy = false;
let watchdogSnapshot = null;

function watchdogRender() {
  const s = watchdogSnapshot;
  const online = Boolean(s && s.online);
  const rf = document.getElementById('watchdogRF');
  const tether = document.getElementById('watchdogTether');
  const recover = document.getElementById('watchdogRecover');

  rf.disabled = watchdogBusy || !online || !s.can_select_rf;
  tether.disabled = watchdogBusy || !online || !s.can_select_tether;
  recover.disabled = watchdogBusy || !online || !s.can_recover_tether;

  rf.setAttribute('aria-pressed',
    String(online && s.source === 'RF'));
  tether.setAttribute('aria-pressed',
    String(online && s.source === 'TETHER'));

  rf.style.borderColor =
    online && s.source === 'RF' ? '#79c9ff' : '';
  tether.style.borderColor =
    online && s.source === 'TETHER' ? '#79c9ff' : '';

  if (!online) {
    document.getElementById('watchdogState').textContent =
      'Safety node unavailable — controls disabled';
    document.getElementById('watchdogLinks').textContent = '';
    return;
  }

  document.getElementById('watchdogState').textContent =
    'Selected: ' + s.source +
    ' | ' + (s.armed === true ? 'ARMED' : 'disarmed/unknown') +
    ' | ESC command: ' + (s.esc_commanded_on ? 'ON' : 'OFF') +
    ' | ' + (s.inhibit ? 'Inhibited: ' + s.inhibit : 'Ready');

  const age = s.ping_age === null
    ? 'no successful reply'
    : s.ping_age.toFixed(1) + ' s since reply';

  document.getElementById('watchdogLinks').textContent =
    'RF heartbeat: ' + (s.rf_alive ? 'OK' : 'LOST') +
    ' | Tether: ' + (s.tether_alive ? 'OK' : 'UNREACHABLE') +
    ' (' + s.tether_ip + ' via ' + s.tether_interface + ', ' + age + ')' +
    ' | Cube telemetry: ' + (s.vehicle_fresh ? 'OK' : 'STALE') +
    (s.sa_active ? ' | SA KILL active' : '') +
    (s.autonomous ? ' | Autonomous flight mode' : '');
}

async function watchdogPoll() {
  try {
    const r = await fetch('/api/controller', {cache: 'no-store'});
    if (!r.ok) throw new Error('HTTP ' + r.status);
    watchdogSnapshot = await r.json();
  } catch (e) {
    watchdogSnapshot = {online: false};
  }
  watchdogRender();
  setTimeout(watchdogPoll, 500);
}

async function watchdogAction(action, source) {
  if (watchdogBusy || !watchdogSnapshot?.online) return;

  const snapshot = watchdogSnapshot;
  watchdogBusy = true;
  watchdogRender();

  try {
    const r = await fetch('/api/controller', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({
        action: action,
        source: source,
        session: snapshot.session,
        revision: snapshot.revision
      })
    });
    const result = await r.json();
    document.getElementById('watchdogMessage').textContent = result.msg;

    const update = await fetch('/api/controller', {cache: 'no-store'});
    if (update.ok) watchdogSnapshot = await update.json();
  } catch (e) {
    document.getElementById('watchdogMessage').textContent =
      'Request outcome unknown. Check confirmed status before retrying.';
  } finally {
    watchdogBusy = false;
    watchdogRender();
  }
}

watchdogPoll();
</script>
"""


class ControlBridge:
    """Thread-safe boundary between HTTP handlers and the ROS executor."""

    def __init__(self):
        self.lock = threading.Lock()
        self.outgoing = queue.Queue()
        self.pending = {}
        self.latest = {}
        self.received_at = 0.0

    def snapshot(self):
        with self.lock:
            result = dict(self.latest)
            received_at = self.received_at

        result['online'] = (
            received_at > 0.0
            and time.monotonic() - received_at < CONTROL_STATUS_TIMEOUT
        )
        return result

    def on_status(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            if not isinstance(value.get('session'), str):
                return
            if not isinstance(value.get('revision'), int):
                return
        except (TypeError, ValueError):
            return

        with self.lock:
            self.latest = value
            self.received_at = time.monotonic()

    def on_reply(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
        except (TypeError, ValueError):
            return

        with self.lock:
            entry = self.pending.get(value.get('id'))
            if entry is not None:
                entry['result'] = value
                entry['event'].set()

    def request(self, payload):
        if not self.snapshot().get('online'):
            return {'ok': False, 'msg': 'Safety node is unavailable.'}

        request_id = uuid.uuid4().hex
        message = dict(payload)
        message['id'] = request_id
        # Both processes run on the same Jetson and share CLOCK_MONOTONIC.
        message['deadline'] = (
            time.monotonic() + CONTROL_REQUEST_LIFETIME
        )
        entry = {'event': threading.Event(), 'result': None}

        with self.lock:
            self.pending[request_id] = entry

        self.outgoing.put(message)
        try:
            if not entry['event'].wait(CONTROL_REQUEST_LIFETIME + 1.0):
                return {
                    'ok': False,
                    'msg': (
                        'No acknowledgement. Check confirmed status; '
                        'do not assume the request failed or succeeded.'
                    ),
                }
            return entry['result']
        finally:
            with self.lock:
                self.pending.pop(request_id, None)

    def flush(self, publisher):
        # Bounded work per ROS timer callback.
        for _ in range(8):
            try:
                message = self.outgoing.get_nowait()
            except queue.Empty:
                return

            if time.monotonic() > message['deadline']:
                with self.lock:
                    entry = self.pending.get(message['id'])
                    if entry is not None:
                        entry['result'] = {
                            'ok': False,
                            'msg': 'Request expired before it was sent.',
                        }
                        entry['event'].set()
                continue

            publisher.publish(String(data=json.dumps(message)))


def pids_for(name):
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == os.getpid():
            continue
        try:
            with open('/proc/' + entry + '/cmdline', 'rb') as f:
                parts = f.read().decode('utf-8', 'replace').split('\0')
        except OSError:
            continue
        if any(p == name or p.endswith('/' + name) for p in parts):
            found.append(int(entry))
    return found


def any_mission_running():
    return any(pids_for(name) for name in MISSION_EXECUTABLES)


def spawn(argv, out=subprocess.DEVNULL):
    subprocess.Popen(
        argv, stdout=out, stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def tail(path, nbytes=60000):
    with open(path, 'rb') as f:
        f.seek(0, 2)
        f.seek(max(0, f.tell() - nbytes))
        return f.read().split(b'\n')[1:]


def describe(key, label, execs):
    up = sum(1 for e in execs if pids_for(e))
    return {
        'key': key, 'label': label, 'nodes': execs,
        'up': up, 'total': len(execs),
    }


def status():
    now = time.time()

    def seen(timestamp):
        return round(now - timestamp, 1) if timestamp else -1

    return {
        'led': LED_NAMES.get(tel['led'], '--'),
        'led_age': seen(tel['led_t']),
        'led_off': led_off,
        'armed': tel['armed'],
        'mode': tel['mode'],
        'mav_age': seen(tel['mav_t']),
        'depth': round(tel['depth'], 2),
        'dvl_lock': tel['dvl_lock'],
        'altitude': round(tel['altitude'], 2),
        'dvl_age': seen(tel['dvl_t']),
        'heading': tel['heading'],
        'hdg_age': seen(tel['hdg_t']),
        'mission': bool(pids_for(MISSION)),
    }


def kill_exec(name):
    def signal_all(sig):
        for pid in pids_for(name):
            try:
                os.kill(pid, sig)
            except OSError:
                pass

    signal_all(signal.SIGTERM)
    for _ in range(10):
        if not pids_for(name):
            return True
        time.sleep(0.1)

    signal_all(signal.SIGKILL)
    time.sleep(0.2)
    return not pids_for(name)


def led_blackout(on):
    global led_off

    if not on:
        led_off = False
        if not pids_for(LED_SOURCE):
            spawn(['ros2', 'run', 'robotx_graey_2026', LED_SOURCE])
        return True, 'status LED back under ' + LED_SOURCE

    if not kill_exec(LED_SOURCE):
        return False, 'could not stop ' + LED_SOURCE

    if not pids_for(LED_DRIVER):
        spawn(['ros2', 'run', 'robotx_graey_2026', LED_DRIVER])

    led_off = True
    return True, 'status LED off - the panel goes dark within a second'


def mission_start(query):
    if any_mission_running():
        return False, 'a mission is already running'

    dry = query.get('dry', ['true'])[0] == 'true'
    rc = query.get('rc_start', ['false'])[0] == 'true'
    cv = query.get('use_cv', ['true'])[0] == 'true'

    if not dry:
        safety = control_bridge.snapshot()
        if not safety.get('online'):
            return False, 'safety node is unavailable'
        if safety.get('inhibit') or safety.get('sa_active'):
            return False, 'resolve the safety inhibit before starting a mission'
        if not safety.get('vehicle_fresh'):
            return False, 'Cube telemetry is stale'

    cv_up = bool(pids_for(CAMERA))
    if cv and not cv_up:
        spawn(['ros2', 'run', 'robotx_graey_2026', CAMERA])
        if not dry:
            return False, (
                CAMERA + ' was not running - started it. Wait for '
                'the Camera tab to show video, then run again.'
            )

    argv = [
        'ros2', 'run', 'robotx_graey_2026', MISSION, '--ros-args',
        '-p', 'dry_run:=' + ('true' if dry else 'false'),
        '-p', 'rc_start:=' + ('true' if rc else 'false'),
        '-p', 'use_cv:=' + ('true' if cv else 'false'),
    ]
    for name, (lo, hi) in MISSION_PARAMS.items():
        raw = query.get(name)
        if not raw:
            continue
        try:
            value = float(raw[0])
        except ValueError:
            return False, 'bad value for ' + name
        value = max(lo, min(hi, value))
        argv += ['-p', name + ':=' + str(value)]

    os.makedirs(os.path.dirname(MISSION_LOG), exist_ok=True)
    with open(MISSION_LOG, 'wb') as output:
        spawn(argv, output)

    if rc:
        return True, 'armed and waiting - press SE on the transmitter to start'
    if dry:
        return True, 'dry run started' + (
            '' if cv_up or not cv else ' - camera starting too'
        )
    return True, 'MISSION STARTED'


def mission_stop():
    if link:
        link.disarm()

    for pid in pids_for(MISSION):
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    time.sleep(1.0)
    survivors = pids_for(MISSION)
    for pid in survivors:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    return True, (
        'mission stopped and disarmed'
        + (' (needed SIGKILL)' if survivors else '')
    )


class Handler(BaseHTTPRequestHandler):
    def reply(self, code, body, ctype):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def reply_json(self, code, value):
        self.reply(
            code, json.dumps(value).encode(), 'application/json'
        )

    def do_POST(self):
        if urlparse(self.path).path != '/api/controller':
            self.send_error(404)
            return

        # New state-changing API uses same-origin JSON POST.
        origin = self.headers.get('Origin')
        if origin and urlparse(origin).netloc != self.headers.get('Host'):
            self.reply_json(403, {'ok': False, 'msg': 'Origin rejected.'})
            return

        if self.headers.get_content_type() != 'application/json':
            self.reply_json(415, {'ok': False, 'msg': 'JSON required.'})
            return

        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 2048:
                raise ValueError('invalid body size')
            self.connection.settimeout(3.0)
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError('expected object')

            action = body.get('action')
            source = body.get('source')
            if action not in ('select', 'recover'):
                raise ValueError('invalid action')
            if source not in ('RF', 'TETHER'):
                raise ValueError('invalid source')
            if not isinstance(body.get('session'), str):
                raise ValueError('missing session')
            if not isinstance(body.get('revision'), int):
                raise ValueError('missing revision')
        except (ValueError, TypeError, OSError):
            self.reply_json(400, {'ok': False, 'msg': 'Invalid request.'})
            return

        with operation_lock:
            if any_mission_running():
                result = {
                    'ok': False,
                    'msg': 'Stop the mission before changing or recovering control.',
                }
            else:
                result = control_bridge.request({
                    'action': action,
                    'source': source,
                    'session': body['session'],
                    'revision': body['revision'],
                })

        self.reply_json(200 if result.get('ok') else 409, result)

    def do_GET(self):
        u = urlparse(self.path)
        query = parse_qs(u.query)

        if u.path == '/api/controller':
            self.reply_json(200, control_bridge.snapshot())
            return

        if u.path == '/api/status':
            self.reply_json(200, status())
            return

        if u.path == '/api/groups':
            self.reply_json(200, {
                'groups': [
                    describe(k, label, executables)
                    for k, (label, executables) in GROUPS.items()
                ],
                'readonly': [
                    describe(k, label, executables)
                    for k, (label, executables) in READ_ONLY.items()
                ],
            })
            return

        if u.path == '/api/log':
            try:
                lines = tail(ROS_LOG)
            except OSError:
                lines = [b'no log yet - graey-ros writes it on start']

            if query.get('errors', ['0'])[0] == '1':
                lines = [
                    line for line in lines
                    if any(word in line for word in NOISY)
                ] or [b'(no warnings or errors)']

            self.reply(200, b'\n'.join(lines[-250:]), 'text/plain')
            return

        if u.path == '/api/power':
            what = query.get('do', [''])[0]
            if what not in REQUESTS:
                self.send_error(400, 'unknown power action')
                return

            os.makedirs(os.path.dirname(REQUESTS[what]), exist_ok=True)
            with open(REQUESTS[what], 'w') as f:
                f.write(str(time.time()))

            self.reply_json(200, {
                'ok': True,
                'msg': (
                    what + ' requested - the Jetson acts within a few seconds. '
                    'Wait for the lights to go out before disconnecting batteries.'
                ),
            })
            return

        if u.path == '/api/led':
            what = query.get('do', [''])[0]
            if what not in ('off', 'on'):
                self.send_error(400, 'unknown led action')
                return

            ok, msg = led_blackout(what == 'off')
            self.reply_json(200 if ok else 409, {'ok': ok, 'msg': msg})
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
            with operation_lock:
                if u.path.endswith('start'):
                    ok, msg = mission_start(query)
                else:
                    ok, msg = mission_stop()
            self.reply_json(200 if ok else 409, {'ok': ok, 'msg': msg})
            return

        if u.path in ('/api/start', '/api/stop'):
            key = query.get('group', [''])[0]
            if key not in GROUPS:
                self.send_error(400, 'unknown group')
                return

            for name in GROUPS[key][1]:
                if u.path == '/api/stop':
                    for pid in pids_for(name):
                        try:
                            os.kill(pid, signal.SIGTERM)
                        except ProcessLookupError:
                            pass
                elif not pids_for(name):
                    spawn(['ros2', 'run', 'robotx_graey_2026', name])

            self.reply_json(200, {'ok': True})
            return

        fname = FILES.get(u.path)
        if not fname:
            self.send_error(404)
            return

        try:
            with open(os.path.join(web_dir, fname), 'rb') as f:
                body = f.read()

            if u.path == '/':
                # Insert after the opening body tag, retaining the old GUI.
                html = body.decode('utf-8')
                start = html.lower().find('<body')
                end = html.find('>', start) if start >= 0 else -1
                if end >= 0:
                    html = html[:end + 1] + CONTROL_PANEL + html[end + 1:]
                else:
                    html = CONTROL_PANEL + html
                body = html.encode('utf-8')

            self.reply(200, body, TYPES[os.path.splitext(fname)[1]])
        except OSError:
            self.send_error(404, fname + ' not found in ' + web_dir)

    def log_message(self, *args):
        pass


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        global web_dir, link, control_bridge

        self.declare_parameter('port', 8090)
        self.declare_parameter('web_dir', web_dir)
        self.declare_parameter('mavlink', 'udpout:127.0.0.1:14555')

        web_dir = self.get_parameter('web_dir').value
        port = self.get_parameter('port').value

        self.link = link = Link(
            self.get_parameter('mavlink').value,
            195,
            self.get_logger(),
        )
        self.led_pub = self.create_publisher(
            Int32, '/graey/led_state', 10
        )
        self.led_checked = 0.0

        control_bridge = ControlBridge()
        self.control_bridge = control_bridge
        self.control_pub = self.create_publisher(
            String, CONTROL_REQUEST_TOPIC, 10
        )
        self.create_subscription(
            String, CONTROL_STATUS_TOPIC,
            self.control_bridge.on_status, 10,
        )
        self.create_subscription(
            String, CONTROL_REPLY_TOPIC,
            self.control_bridge.on_reply, 10,
        )
        self.create_timer(
            0.05,
            lambda: self.control_bridge.flush(self.control_pub),
        )

        self.create_subscription(
            Int32, '/graey/led_state', self.on_led, 10
        )
        self.create_subscription(
            Bool, '/graey/dvl/valid', self.on_dvl, 10
        )
        self.create_subscription(
            Float32, '/graey/dvl/altitude', self.on_alt, 10
        )
        self.create_subscription(
            Float32, '/graey/vn100/heading', self.on_hdg, 10
        )

        self.create_timer(1.0, self.link.heartbeat)
        self.create_timer(0.1, self.pump)
        self.create_timer(0.2, self.hold_led)

        self.http_server = ThreadingHTTPServer(
            ('0.0.0.0', port), Handler
        )
        self.http_server.daemon_threads = True
        threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True,
        ).start()

        self.get_logger().info(f'GUI on http://0.0.0.0:{port}')

    def hold_led(self):
        global led_off
        if not led_off:
            return

        now = time.time()
        if now - self.led_checked > 2.0:
            self.led_checked = now
            if pids_for(LED_SOURCE):
                led_off = False
                self.get_logger().info(
                    LED_SOURCE + ' is back - releasing the LED'
                )
                return

        self.led_pub.publish(Int32(data=0))

    def on_led(self, msg):
        tel['led'], tel['led_t'] = msg.data, time.time()

    def on_dvl(self, msg):
        tel['dvl_lock'], tel['dvl_t'] = msg.data, time.time()

    def on_alt(self, msg):
        tel['altitude'] = msg.data

    def on_hdg(self, msg):
        tel['heading'], tel['hdg_t'] = msg.data, time.time()

    def pump(self):
        def consume(kind, msg):
            if (
                kind == 'HEARTBEAT'
                and msg.get_srcSystem() == 1
                and msg.get_srcComponent() == 1
            ):
                tel['armed'] = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                tel['mode'] = self.link.mav.flightmode
                tel['mav_t'] = time.time()
            elif kind == 'LOCAL_POSITION_NED':
                tel['depth'] = msg.z

        self.link.drain(consume)

    def destroy_node(self):
        if hasattr(self, 'http_server'):
            self.http_server.shutdown()
            self.http_server.server_close()
        return super().destroy_node()


def main():
    run(GuiNode)
