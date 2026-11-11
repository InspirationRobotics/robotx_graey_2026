#!/usr/bin/env python3
"""GRAEY ground station, following the SD fleet's operator conventions.

Shared theme, transport/run/outbox snapshots, process and sensor status,
incremental logs and host measurements accompany the existing vehicle controls.

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
from pathlib import Path

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from robotx_graey_2026.api.steady_node import SteadyNode as Node
from std_msgs.msg import Int32, Bool, Float32, String
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import Log
from robotx_graey_2026.api.gui.log_buffer import LogBuffer
from robotx_graey_2026.api.gui.navigation_view import NAVIGATION_VIEW_KINDS, NavigationView
from robotx_graey_2026.api.gui.communications_view import CommunicationsView
from robotx_graey_2026.api.gui.system_info import SystemInfo
from robotx_graey_2026.api.gui.log_reader import read_log
from robotx_graey_2026.api.gui.recording_service import RecordingService
from robotx_graey_2026.api.gui.mission_store import MissionStore

communications = CommunicationsView()
tiers_requests = queue.Queue(maxsize=1)
gateway_view = CommunicationsView()
system_info = SystemInfo()
recording_service = None
parameter_bridge = None
parameter_graph = []
parameter_defaults = {}
fleet_logs=LogBuffer(1500)
mission_store = MissionStore(Path(os.environ.get('GRAEY_MISSION_DIR',str(Path.home()/'robotx_ws/missions'))))
from robotx_graey_2026.api.gui.waypoint_api import WaypointAPI
from robotx_graey_2026.api.gui.survey_launch import SurveyLauncher
from robotx_graey_2026.api.gui.calibration_jobs import CalibrationJobs, ServiceQueue

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.process_ownership import executable_pids
from robotx_graey_2026.api.pixhawk.mavlink import Link, mavutil


CAMERA = 'pole_tracker'

GROUPS = {
    'led': ('LED stack', ['led_node', 'pixhawk_led_node']),
    'nav': ('Nav / EKF stack',
            ['dvl_node', 'vn100_node', 'fused_nav_bridge']),
    'camera': ('Camera tracker', [CAMERA]),
    'planner': ('Planner feed', ['pos_server']),
}
CORE_MANAGED_GROUPS = {'nav'}
CORE_MANAGED_EXECUTABLES = frozenset(
    name for key in CORE_MANAGED_GROUPS for name in GROUPS[key][1]
) | {'navigation_supervisor', 'nav_ekf_bridge'}
READ_ONLY = {
    'mavproxy': ('MAVProxy', ['mavproxy.py']),
    'gcs': ('Laptop GCS bridge · core', ['gcs_bridge']),
    'gateway': ('FCU telemetry gateway · core', ['telemetry_gateway']),
    'watchdog': ('Control watchdog · core', ['kill_switch']),
    'communications': ('OCS client · automatic discovery / reconnect', ['competition_link_node']),
}
FILES = {
    '/missions':'missions.html',
    '/sensors':'sensors.html',
    '/record':'record.html',
    '/missions.js':'missions.js',
    '/crusader-map.js':'crusader_map.js',
    '/sensors.js':'sensors.js',
    '/record.js':'record.js',
    '/mission.css':'mission.css',
    '/ti_logo.jpg': 'ti_logo.jpg',
    '/asta_logo.png': 'asta_logo.png',
    '/station.css': 'station.css',
    '/station_theme.js': 'station_theme.js',
    '/simple-sequence': 'simple_sequence.html',
    '/vendor/leaflet/leaflet.js': 'vendor/leaflet/leaflet.js',
    '/vendor/leaflet/leaflet.css': 'vendor/leaflet/leaflet.css',
    '/navigation': 'navigation.html',
    '/navigation-supervisor': 'navigation_supervisor.html',
    '/': 'gui.html',                   # the whole GUI, every tab, lives here (Ruth, Nov 7)
    '/planner': 'missions.html',
    '/pool-map': 'missions.html',
    '/waypoint': 'missions.html',
    '/calibration': 'calibration.html',
    '/pool_reference.png': 'pool_reference.png',
    '/pool.png': 'pool.png',
    '/neighbor_pool.png': 'neighbor_pool.png',
}
TYPES = {
    '.js': 'application/javascript',
    '.css': 'text/css',
    '.html': 'text/html',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
}

def page_navigation(html, path):
    """Shared navigation across existing pages; retain every page and control."""
    theme='<link rel="stylesheet" href="/station.css"><script src="/station_theme.js"></script>'
    html=html.replace('</head>',theme+'</head>',1) if '</head>' in html else theme+html
    if path in ('/','/missions','/sensors','/record'):return html
    groups = (
        ('Missions', '/missions', ('/missions','/waypoint','/pool-map','/planner','/simple-sequence')),
        ('Sensors', '/sensors', ('/navigation','/sensors','/navigation-supervisor')),
        ('Record', '/record', ('/record',)),
        ('Prepare & Calibration','/calibration',('/calibration',)),
    )
    links = ''.join('<a href="'+url+'"'+(' aria-current="page"' if path in paths else '')+'>'+label+'</a>'
                    for label, url, paths in groups)
    extras = '<a href="/vehicle-controls">Vehicle controls</a>'
    bar = ('<style>.graey-tabs{display:flex;flex-wrap:wrap;gap:8px;padding:10px 14px;'
           'background:#15222e;border-bottom:1px solid #47647a;flex:none;position:relative;z-index:2000}'
           '.graey-tabs a{padding:10px 14px;color:#e5edf2;text-decoration:none;border-radius:6px;'
           'font:600 14px system-ui}.graey-tabs a[aria-current]{background:#216b85}'
           '.graey-tabs a:focus-visible{outline:2px solid #ffda89}</style>'
           '<nav class="graey-tabs" aria-label="Main navigation">'+links+extras+'</nav>')
    # Pages with implicit body start at their header. Explicit-body pages insert
    # immediately after its opening tag, keeping all existing scripts intact.
    start=html.lower().find('<body')
    end=html.find('>',start)+1 if start>=0 else html.lower().find('<header')
    if end<0:end=0
    return html[:end]+bar+html[end:]
# Explicit allowlist for packaged georeferenced imagery; no arbitrary file route.
for _z,_x0,_y0,_radius in ((16,11450,26416,1),(19,91604,211332,3)):
    for _x in range(_x0-_radius,_x0+_radius+1):
        for _y in range(_y0-_radius,_y0+_radius+1):
            _tile=f'vendor/miramar/{_z}-{_y}-{_x}.jpg'
            FILES['/'+_tile]=_tile
LED_NAMES = {0: 'off', 1: 'RED', 2: 'YELLOW', 3: 'GREEN'}
LED_DRIVER = 'led_node'
LED_SOURCE = 'pixhawk_led_node'

MISSION = 'prequal_mission_cv'
MISSION_EXECUTABLES = (
    'north_maneuver',
    'waypoint_mission',
    'sitl_mission',
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

# Resolve bundled GUI assets from this source/install layout. On the Jetson this
# resolves to the same tools directory as the historic absolute path; it also
# lets the local WSL SITL serve the GUI without a Jetson-specific mount path.
_source_tools = Path(__file__).resolve().parents[3] / 'tools'
if _source_tools.is_dir():
    web_dir = str(_source_tools)
else:
    from ament_index_python.packages import get_package_share_directory
    web_dir = str(Path(get_package_share_directory('robotx_graey_2026')) / 'tools')
link = None
led_off = False
control_bridge = None
navigation = NavigationView()
waypoint_api = WaypointAPI()
waypoint_aborts = queue.Queue()
# Crusader's gcs_bridge never decodes raw MAVLink: it reads telemetry_bridge's decoded summaries. Here the gateway's
# summaries (/graey/fcu/pose, attitude, status) carry these kinds; the raw stream (~235 frames/s) is left for the rest.
SUMMARY_KINDS = {'pose': 'GLOBAL_POSITION_INT', 'attitude': 'ATTITUDE', 'status': 'HEARTBEAT'}
# Raw frames older than 0.25 s are dropped (ros_link). At ~235 frames/s a 200-deep queue holds ~0.85 s, so once the GUI
# fell behind, every frame it reached was too old and its autopilot data went stale for good (Graey, Nov 11: load 13
# with the sonar map and cameras on). 50 keeps whatever is queued inside the 0.25 s window.
RAW_RX_DEPTH = 50


class SummaryFrame:
    """A gateway summary, read like the pymavlink message it came from (the Cube: system 1, component 1)."""
    def __init__(self, kind, fields):
        self.__dict__.update(fields)
        self._kind = kind

    def get_type(self):
        return self._kind

    def get_srcSystem(self):
        return 1

    def get_srcComponent(self):
        return 1


waypoint_process = None
waypoint_launch = None
survey = SurveyLauncher()   # Ruth's survey run (Missions page, Nov 11)
calibration_jobs = None
north_process = None
north_aborts = queue.Queue()
north_cancel_pending = False


def north_maneuver_request(action, distance):
    global north_process, north_cancel_pending
    if action == 'abort':
        north_aborts.put(True)
        return
    from robotx_graey_2026.api.navigation.north_maneuver import NorthManeuver
    NorthManeuver(time.monotonic(),distance)  # Validate before starting a process.
    c=control_bridge.snapshot() if control_bridge else {}
    if not (c.get('online') and c.get('vehicle_fresh') and c.get('armed') is True
            and c.get('mode')=='MANUAL' and c.get('sa_active') is False
            and c.get('esc_commanded_on') is True and not c.get('inhibit')
            and not c.get('mission_claimed')):
        raise ValueError('Deliberately enable propulsion and arm in MANUAL; keep the surface corridor clear')
    if any_mission_running() or (north_process is not None and north_process.poll() is None):
        raise ValueError('Another mission or alignment maneuver is running')
    while not north_aborts.empty():north_aborts.get_nowait()
    north_cancel_pending=False
    folder=Path(os.environ.get('GRAEY_WAYPOINT_LOG_DIR',str(Path.home()/'robotx_ws/logs')))
    folder.mkdir(parents=True,exist_ok=True)
    with (folder/('north-maneuver-'+uuid.uuid4().hex+'.log')).open('w') as log:
        north_process=subprocess.Popen(['ros2','run','robotx_graey_2026','north_maneuver','--ros-args',
            '-p','max_distance_m:='+str(float(distance))],stdout=log,stderr=subprocess.STDOUT)


def waypoint_simulation():
    return (os.environ.get('GRAEY_RUNTIME_SCOPE') == 'sitl'
            and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
            and os.environ.get('ROS_DOMAIN_ID') == '73')


def waypoint_capabilities():
    sim = waypoint_simulation()
    bridge=navigation.snapshot()['streams'].get('bridge',{})
    enabled=bool(bridge.get('age',float('inf'))<1. and bridge.get('data',{}).get('enabled'))
    return dict(simulation=sim, enabled=enabled,
                pool_registration='approximate',
                mapping_note=('Screenshot scale/anchor are approximate, not surveyed. Live pool-image starts are blocked.'
                    + ('' if enabled else ' Live waypoint execution is disabled pending vehicle commissioning.')))


def waypoint_request(body):
    global waypoint_process, waypoint_launch
    action = body.get('action')
    nav = navigation.snapshot()
    if action in ('preview','qgc_preview'):
        return waypoint_api.preview(body.get('plan'), nav,source='qgc' if action=='qgc_preview' else 'gui')
    if action == 'survey_start':
        # Ruth's survey run: the same checks as a live route start (dry: nothing moves, so no motion checks)
        if calibration_jobs is not None and calibration_jobs.snapshot().get('busy'):
            raise ValueError('Finish or cancel navigation preparation before starting a mission')
        if body.get('competition'):
            raise ValueError('The survey run does not wait for Competition RunStart; untick Competition')
        dry = body.get('dry', False)
        if type(dry) is not bool:
            raise ValueError('dry must be a boolean')
        if not dry:
            cap = waypoint_capabilities()
            if not cap['enabled']:
                raise ValueError('Live waypoint commissioning is not enabled (Start navigation on Sensors)')
            if not cap['simulation']:
                c = control_bridge.snapshot()
                if not (c.get('online') and c.get('vehicle_fresh') and not c.get('inhibit')
                        and not c.get('sa_active') and c.get('esc_commanded_on')):
                    raise ValueError('Control watchdog/ESC permission unavailable')
        if any_mission_running() or (waypoint_process is not None and waypoint_process.poll() is None):
            raise ValueError('Another mission is running')
        return survey.start(body.get('points'), dry)
    if action == 'survey_stop' or (action == 'abort' and survey.owns(body.get('run_id'))):
        return survey.stop(body.get('run_id'))
    if action == 'abort':
        status = nav['streams'].get('waypoint', {}).get('data', {})
        expected = waypoint_launch['run_id'] if waypoint_launch and waypoint_process.poll() is None else status.get('run_id')
        if not body.get('run_id') or body['run_id'] != expected:
            raise ValueError('Current mission run ID required')
        waypoint_aborts.put(body['run_id'])
        return dict(ok=True, msg='Abort requested; verify MANUAL mode and take control')
    if action not in ('start', 'simple_start'):
        raise ValueError('Unknown waypoint action')
    cap = waypoint_capabilities()
    if action == 'simple_start':
        item = {'plan': {'kind': 'simple_sequence', 'mode': 'surface'}}
    else:
        if calibration_jobs is not None and calibration_jobs.snapshot().get('busy'):
            raise ValueError('Finish or cancel navigation preparation before starting a mission')
        item, duplicate = waypoint_api.consume(body.get('plan_id'), nav)
        if duplicate:
            return dict(ok=True, run_id=item['run_id'], duplicate=True)
        cap = waypoint_capabilities()
        if not cap['enabled']:
            raise ValueError('Live waypoint commissioning is not enabled')
        if not cap['simulation']:
            if item['plan']['map'] == 'pool':
                raise ValueError('Pool image registration is approximate; survey required for live pool targets')
            c = control_bridge.snapshot()
            if not (c.get('online') and c.get('vehicle_fresh') and not c.get('inhibit')
                    and not c.get('sa_active') and c.get('esc_commanded_on')):
                raise ValueError('Control watchdog/ESC permission unavailable')
    if (any_mission_running(ignore_survey=survey.owns(body.get('survey_run_id')))
            or (waypoint_process is not None and waypoint_process.poll() is None)):
        raise ValueError('Another mission is running')
    run_id = uuid.uuid4().hex
    if type(body.get('competition',False)) is not bool:
        raise ValueError('competition must be a boolean')
    item['plan']['competition']=body.get('competition',False)
    folder = Path(os.environ.get('GRAEY_WAYPOINT_LOG_DIR', str(Path.home()/'robotx_ws/logs')))
    folder.mkdir(parents=True, exist_ok=True)
    argv = ['ros2', 'run', 'robotx_graey_2026', 'waypoint_mission', '--ros-args',
            '-p', "plan_json:='"+json.dumps(item['plan'])+"'", '-p', 'run_id:='+json.dumps(run_id),
            '-p', 'simulation:='+str(cap['simulation']).lower(), '-p', 'commissioned:=true',
            '-p', 'result_file:='+str(folder/('waypoint-'+run_id+'.json'))]
    with (folder/('waypoint-'+run_id+'.log')).open('w') as log:
        # Inherit the GUI process group: isolated SITL shutdown must reap this child too.
        waypoint_process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT)
    item['run_id'] = run_id
    waypoint_launch = dict(run_id=run_id, started=time.monotonic(), plan=item['plan'])
    return dict(ok=True, run_id=run_id)


def waypoint_snapshot():
    nav = navigation.snapshot()
    if waypoint_launch:
        current = nav['streams'].get('waypoint', {}).get('data', {})
        if current.get('run_id') != waypoint_launch['run_id']:
            exited = waypoint_process.poll()
            age = time.monotonic()-waypoint_launch['started']
            nav['streams']['waypoint'] = dict(age=age, fresh=exited is None and age < 5,
                data=dict(run_id=waypoint_launch['run_id'], plan=waypoint_launch['plan'],
                          stage='STARTING' if exited is None else 'ABORTED',
                          reason='Waiting for mission process' if exited is None else 'Mission process failed to start; check mission log'))
    return dict(capabilities=waypoint_capabilities(), navigation=nav, survey=survey.snapshot())

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

  const age = !Number.isFinite(s.ping_age)
    ? 'no successful reply'
    : s.ping_age.toFixed(1) + ' s since reply';

  document.getElementById('watchdogLinks').textContent =
    'RF heartbeat: ' + (s.rf_alive ? 'OK' : 'LOST') +
    ' | Tether: ' + (s.tether_alive ? 'OK' : 'UNREACHABLE') +
    ' (' + (s.tether_ip||'not configured') + ' via ' + (s.tether_interface||'unknown interface') + ', ' + age + ')' +
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

VEHICLE_CONTROLS_PAGE = ('<!DOCTYPE html><html><head><meta charset="utf-8"><title>Vehicle controls</title>'
                         '<style>body{margin:0;font-family:Segoe UI,Arial,sans-serif;background:#14171c;color:#e8e8e8}'
                         'button.act{background:#2d3442;color:#e8e8e8;border:1px solid #444;border-radius:6px;'
                         'padding:5px 16px;cursor:pointer;font-size:13px}button.act:hover{border-color:#5b9bd5}'
                         '</style></head><body>' + CONTROL_PANEL + '</body></html>')



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


def any_mission_running(ignore_survey=False):
    # ignore_survey: the survey run's own surface legs start while it runs
    return ((north_process is not None and north_process.poll() is None)
            or (not ignore_survey and survey.running())
            or (waypoint_process is not None and waypoint_process.poll() is None)
            or any(pids_for(name) for name in MISSION_EXECUTABLES))


def spawn(argv, out=subprocess.DEVNULL):
    # Enforce ownership here too, so another GUI route cannot bypass the
    # navigation endpoint's rejection and create competing sensor/odom owners.
    if any(os.path.basename(str(arg)) in CORE_MANAGED_EXECUTABLES for arg in argv):
        raise ValueError('Navigation processes are owned by core launch; GUI launch refused.')
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
    counts = {e: len(executable_pids(e)) for e in execs}
    up = sum(count > 0 for count in counts.values())
    return {
        'key': key, 'label': label, 'nodes': execs,
        'up': up, 'total': len(execs),
        'instances': counts, 'duplicate': any(count > 1 for count in counts.values()),
        'managed': key in CORE_MANAGED_GROUPS,
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
    if calibration_jobs is not None and calibration_jobs.snapshot().get('busy'):
        return False, 'Finish or cancel calibration/preparation before starting a mission'
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
    if survey.running():    # first: the survey aborts its own leg, or its transit surfaces
        survey.stop(survey.launch['run_id'])
        return True, 'Survey stop requested: its leg aborts or its transit surfaces; verify and take control'
    if (north_process is not None and north_process.poll() is None) or pids_for('north_maneuver'):
        north_aborts.put(True)
        return True, 'North alignment abort requested; verify neutral thrust and take control'
    if waypoint_launch and waypoint_process is not None and waypoint_process.poll() is None:
        waypoint_aborts.put(waypoint_launch['run_id'])
        return True, 'Waypoint abort requested; verify MANUAL mode and take control'
    if pids_for('waypoint_mission'):
        current = navigation.snapshot()['streams'].get('waypoint', {}).get('data', {})
        if not current.get('run_id'):
            return False, 'Waypoint status unavailable; use pilot controls. Stop has not been confirmed.'
        waypoint_aborts.put(current['run_id'])
        return True, 'Waypoint abort requested; verify MANUAL mode and take control'
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
        path = urlparse(self.path).path
        if path not in ('/api/controller', '/api/navigation/reset', '/api/waypoint', '/api/calibration', '/api/communications/tiers','/api/recording','/api/missions','/api/recording/session','/api/params'):
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
            if not 0 < size <= (65536 if path in ('/api/recording','/api/missions') else 2048):
                raise ValueError('invalid body size')
            self.connection.settimeout(3.0)
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError('expected object')

            if path == '/api/params':
                node=body.get('node');changes=body.get('values')
                if parameter_bridge is None or node not in parameter_graph or not isinstance(changes,dict):raise ValueError('Known node and parameter values required')
                with operation_lock:
                    if any_mission_running():raise ValueError('Stop missions before live parameter updates')
                    rows=parameter_bridge.set(node,changes)
                ok=all(row['ok'] for row in rows)
                self.reply_json(200 if ok else 409,dict(ok=ok,msg='Live parameters applied' if ok else '; '.join(row['reason'] for row in rows if not row['ok']),results=rows))
                return
            if path == '/api/recording/session':
                if recording_service is None or body.get('action')!='delete':raise ValueError('Unknown recording operation')
                self.reply_json(200,recording_service.delete(body.get('name')))
                return
            if path == '/api/recording':
                if recording_service is None:raise ValueError('Recording service unavailable')
                recording_service.submit(body)
                result=recording_service.wait_result(body['action'])
                self.reply_json(200 if result.get('ok') else 409,result)
                return
            if path == '/api/missions':
                action=body.get('action')
                if action=='save':result=mission_store.save(body.get('name'),body.get('mission'))
                elif action=='delete':mission_store.delete(body.get('name'));result=None
                else:raise ValueError('Unknown mission-library action')
                self.reply_json(200,dict(ok=True,mission=result,missions=mission_store.list()))
                return
            if path == '/api/communications/tiers':
                snapshot=communications.snapshot()
                data=snapshot['data']
                tiers=body.get('tiers')
                mission=body.get('mission','')
                if (not snapshot['fresh'] or snapshot['connection']!='connected'
                        or not data.get('ocs_run') or data.get('remote_state') in ('DECLARED','RUNNING')):
                    self.reply_json(409,dict(ok=False,msg='A connected OCS with no declared/running run is required.'))
                    return
                if (not isinstance(tiers,dict) or set(tiers)!={'task1','task2','task3','task4'}
                        or any(v not in ('TIER_NONE','TIER_CORE','TIER_ADVANCED','TIER_DISRUPTIVE') for v in tiers.values())
                        or not isinstance(mission,str) or len(mission)>128):
                    raise ValueError('All four valid task tiers and a mission name of at most 128 characters are required.')
                try:tiers_requests.put_nowait(dict(tiers=tiers,mission=mission,requested_monotonic=time.monotonic()))
                except queue.Full:
                    self.reply_json(409,dict(ok=False,msg='Another tier request is pending.'))
                    return
                self.reply_json(202,dict(ok=True,msg='Request handed to communications; check the OCS reply.'))
                return

            if path == '/api/calibration':
                with operation_lock:
                    try:
                        if calibration_jobs is None:
                            raise ValueError('Calibration services unavailable')
                        result = calibration_jobs.request(body)
                        if body.get('action') in ('calibrate','prepare'):
                            navigation.reset_display()
                    except (ValueError, TypeError, OSError) as error:
                        self.reply_json(409, dict(ok=False,msg=str(error)))
                        return
                self.reply_json(202,dict(ok=True,job=result))
                return

            if path == '/api/waypoint':
                with operation_lock:
                    try:
                        result = waypoint_request(body)
                    except (ValueError, TypeError, OSError) as error:
                        self.reply_json(409, dict(ok=False, msg=str(error)))
                        return
                self.reply_json(202, result)
                return

            if path == '/api/navigation/reset':
                if body != {'action': 'reset_display'}:
                    raise ValueError('invalid display request')
                self.reply_json(202, navigation.reset_display())
                return

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

        if u.path == '/api/fleet/logs':
            try:
                rows,cursor,dropped=fleet_logs.read(int(query.get('after',['0'])[0]),int(query.get('min_level',['20'])[0]))
                self.reply_json(200,dict(items=rows,cursor=cursor,dropped=dropped))
            except ValueError:self.reply_json(400,dict(ok=False,msg='Invalid log cursor/level'))
            return
        if u.path == '/api/params':
            name=query.get('node',[None])[0]
            if name is None:self.reply_json(200,dict(graph=list(parameter_graph)));return
            if parameter_bridge is None or name not in parameter_graph:
                self.reply_json(404,dict(ok=False,msg='Unknown GRAEY ROS node'));return
            try:
                rows=parameter_bridge.list(name,parameter_defaults.get(name.lstrip('/'),{}).get('ros__parameters',{}))
                self.reply_json(200,dict(ok=True,params=rows))
            except (ValueError,RuntimeError,TimeoutError) as exc:self.reply_json(409,dict(ok=False,msg=str(exc)))
            return
        if u.path == '/api/recording/archive':
            if recording_service is None:
                self.reply_json(503,dict(ok=False,msg='Recording service unavailable'));return
            name=query.get('name',[''])[0]
            from robotx_graey_2026.api.gui.recorder import safe_session_dir
            folder=safe_session_dir(str(recording_service.root),name)
            session=recording_service.snapshot().get('session') or {}
            if folder is None or not os.path.isdir(folder) or (session.get('active') and session.get('name')==name):
                self.reply_json(409,dict(ok=False,msg='Select a stopped existing recording session'));return
            self.send_response(200);self.send_header('Content-Type','application/gzip');self.send_header('Connection','close');self.end_headers();self.close_connection=True
            recording_service.archive_into(name,self.wfile)
            return
        if u.path == '/api/recording/sessions':
            self.reply_json(200,dict(sessions=recording_service.sessions() if recording_service else []))
            return
        if u.path == '/api/recording':
            self.reply_json(200,recording_service.snapshot() if recording_service else dict(error='Recording service unavailable',topics=[]))
            return
        if u.path == '/api/missions':
            try:
                name=query.get('name',[None])[0]
                self.reply_json(200,dict(ok=True,mission=mission_store.get(name)) if name else dict(ok=True,missions=mission_store.list()))
            except (ValueError,OSError) as exc:self.reply_json(404,dict(ok=False,msg=str(exc)))
            return
        if u.path.startswith('/api/run/'):
            self.reply_json(503,dict(ok=False,msg='Open the laptop GUI to access the shared OCS run.'))
            return
        if u.path == '/api/system':
            self.reply_json(200,system_info.snapshot())
            return

        if u.path == '/api/logs':
            try:
                self.reply_json(200,read_log(ROS_LOG,int(query.get('after',['0'])[0]),
                    query.get('generation',[''])[0],query.get('errors',['0'])[0]=='1'))
            except (ValueError,OSError) as exc:
                self.reply_json(400,dict(msg=str(exc)))
            return

        if u.path == '/api/calibration':
            self.reply_json(200,dict(job=calibration_jobs.snapshot() if calibration_jobs else {},
                controller=control_bridge.snapshot() if control_bridge else {},
                navigation=navigation.snapshot()))
            return

        if u.path == '/api/communications':
            self.reply_json(200,dict(**communications.snapshot(),gateway=gateway_view.snapshot(),
                controller=control_bridge.snapshot() if control_bridge else {},
                mission=navigation.snapshot()['streams'].get('waypoint',{})))
            return

        if u.path == '/api/waypoint':
            with operation_lock:
                self.reply_json(200, waypoint_snapshot())
            return

        if u.path == '/api/navigation/supervisor':
            self.reply_json(200, navigation.supervisor_snapshot())
            return

        if u.path == '/api/navigation':
            self.reply_json(200, navigation.snapshot())
            return

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

            c=control_bridge.snapshot() if control_bridge else {}
            if not (c.get('online') and c.get('vehicle_fresh') and c.get('armed') is False) or any_mission_running():
                self.reply_json(409,dict(ok=False,msg='Power action requires fresh disarmed telemetry and stopped missions.'))
                return
            origin=self.headers.get('Origin')
            if origin and urlparse(origin).netloc != self.headers.get('Host'):
                self.reply_json(403,dict(ok=False,msg='Origin rejected.'))
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
            if key in CORE_MANAGED_GROUPS:
                self.reply_json(409, {'ok': False, 'msg':
                    'Navigation is managed by core launch; GUI start/stop is disabled.'})
                return
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

        if u.path == '/control':
            # the GUI moved back to the root (Ruth, Nov 7): old /control links land there
            self.send_response(302)
            self.send_header('Location', '/')
            self.send_header('Content-Length', '0')
            self.end_headers()
            return

        if u.path == '/vehicle-controls':
            # only the controller failsafe panel, for the waypoint page's 'Vehicle controls' box
            # (it used to frame the whole GUI from /control)
            self.reply(200, VEHICLE_CONTROLS_PAGE.encode('utf-8'), 'text/html')
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
                    marker='<section id="overview" style="display:none">'
                    html=html.replace(marker,marker+CONTROL_PANEL,1) if marker in html else html[:end + 1]+CONTROL_PANEL+html[end + 1:]
                else:
                    html = CONTROL_PANEL + html
                body = html.encode('utf-8')

            if fname.endswith('.html'):
                body=page_navigation(body.decode('utf-8'),u.path).encode('utf-8')
            self.reply(200, body, TYPES[os.path.splitext(fname)[1]])
        except OSError:
            self.send_error(404, fname + ' not found in ' + web_dir)

    def log_message(self, *args):
        pass


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        global web_dir, link, control_bridge, calibration_jobs, recording_service, parameter_bridge, parameter_defaults

        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter('port',8090,ParameterDescriptor(read_only=True,description='HTTP listener; restart required'))
        self.declare_parameter('web_dir',web_dir,ParameterDescriptor(read_only=True,description='Asset root; restart required'))
        self.declare_parameter('mavlink','udpout:127.0.0.1:14555',ParameterDescriptor(read_only=True,description='FCU transport ownership; restart required'))

        web_dir = self.get_parameter('web_dir').value
        port = self.get_parameter('port').value

        self.link = link = Link(
            self.get_parameter('mavlink').value,
            195,
            self.get_logger(),
            node=self, rx_depth=RAW_RX_DEPTH,
            # pump() feeds raw frames to NavigationView except the summary kinds (on_fcu_summary has those).
            kinds=tuple(k for k in NAVIGATION_VIEW_KINDS if k not in SUMMARY_KINDS.values()))
        # The newest summary only (depth 1): a busy GUI still shows current data, with its true arrival time.
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        newest = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
        for topic, kind in SUMMARY_KINDS.items():
            self.create_subscription(String, '/graey/fcu/'+topic,
                                     lambda msg, kind=kind: self.on_fcu_summary(kind, msg), newest)
        from robotx_graey_2026.api.gui.param_client import ParamBridge
        parameter_bridge=ParamBridge(self)
        try:
            import yaml
            config=Path(web_dir).parent/'config/graey_params.yaml'
            parameter_defaults=yaml.safe_load(config.read_text(encoding='utf-8')) or {}
        except (ImportError,OSError,ValueError):parameter_defaults={}
        self.create_timer(1.,self.update_parameter_graph)
        self.create_subscription(Log,'/rosout',lambda m:fleet_logs.add(m.name,m.level,m.msg,stamp=m.stamp.sec+m.stamp.nanosec*1e-9),100)
        self.tiers_pub=self.create_publisher(String,'/graey/comms/tiers_request',10)
        self.create_timer(.1,self.flush_tiers)
        recording_service=RecordingService(os.environ.get('GRAEY_RECORD_DIR',str(Path.home()/'robotx_ws/recordings')))
        self.recording_service=recording_service
        self.create_timer(1.,lambda:recording_service.update_sources(self.get_topic_names_and_types(),navigation.snapshot()))
        self.led_pub = self.create_publisher(
            Int32, '/graey/led_state', 10
        )
        self.led_checked = 0.0
        self.create_subscription(String,'/graey/comms/status',self.on_communications,10)
        self.create_subscription(String,'/graey/fcu/gateway_status',self.on_gateway,10)

        control_bridge = ControlBridge()
        self.control_bridge = control_bridge
        self.calibration_services = ServiceQueue(self)
        calibration_jobs = CalibrationJobs(control_bridge.snapshot,navigation.snapshot,
            self.calibration_services.request,any_mission_running,maneuver=north_maneuver_request)
        self.create_timer(.05,self.calibration_services.pump)
        self.create_subscription(String, '/graey/vn100/calibration',
            lambda msg: self.calibration_sample('vn_calibration',msg),10)
        self.create_subscription(String, '/graey/navigation/bridge_status',
            lambda msg: self.calibration_sample('bridge',msg),10)
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
        self.create_subscription(Imu,'/graey/vn100/imu',lambda m:navigation.put('vn_imu',dict(
            angular_velocity=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],
            linear_acceleration=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],
            orientation=[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w],
            frame=m.header.frame_id)),10)
        self.create_subscription(TwistWithCovarianceStamped, '/graey/dvl/velocity',
            lambda m: navigation.put('dvl_velocity', {'xyz': [m.twist.twist.linear.x,
                m.twist.twist.linear.y, m.twist.twist.linear.z], 'frame': m.header.frame_id}), 10)
        self.create_subscription(String, '/graey/navigation/status', self.on_navigation_status, 10)
        self.create_subscription(String, '/graey/waypoint/status', self.on_waypoint_status, 10)
        self.waypoint_abort_pub = self.create_publisher(String, '/graey/waypoint/abort', 10)
        self.north_abort_pub = self.create_publisher(String, '/graey/navigation/north_abort', 10)
        self.create_subscription(String,'/graey/navigation/north_maneuver',
            lambda msg:self.calibration_sample('north_maneuver',msg),10)
        self.waypoint_abort_runs = set()
        self.create_timer(.2, self.flush_waypoint_aborts)
        self.create_timer(0.1, self.pump)
        self.create_timer(0.2, self.hold_led)

        self.http_server = ThreadingHTTPServer(
            ('127.0.0.1', port), Handler
        )
        self.http_server.daemon_threads = True
        threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True,
        ).start()

        self.get_logger().info(f'GRAEY backend on http://127.0.0.1:{port}; use the Crusader-layout laptop launcher')

    def on_waypoint_status(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                navigation.put('waypoint', value)
        except (ValueError, TypeError):
            pass

    def calibration_sample(self, name, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value,dict):
                navigation.put(name,value)
        except (ValueError,TypeError):
            pass

    def flush_waypoint_aborts(self):
        global north_cancel_pending
        if not north_aborts.empty():
            while not north_aborts.empty():north_aborts.get_nowait()
            north_cancel_pending=True
        if north_cancel_pending:
            self.north_abort_pub.publish(String(data='cancel'))
        while not waypoint_aborts.empty():
            self.waypoint_abort_runs.add(waypoint_aborts.get_nowait())
        current = navigation.snapshot()['streams'].get('waypoint', {}).get('data', {})
        if current.get('stage') in ('ABORTED', 'COMPLETE'):
            self.waypoint_abort_runs.discard(current.get('run_id'))
        for run_id in self.waypoint_abort_runs:
            self.waypoint_abort_pub.publish(String(data=run_id))

    def on_gateway(self,msg):
        try:gateway_view.update(json.loads(msg.data))
        except (ValueError,TypeError):pass

    def update_parameter_graph(self):
        global parameter_graph
        from robotx_graey_2026.api.gui.param_client import visible_nodes
        parameter_graph=visible_nodes([(namespace.rstrip('/')+'/'+name) for name,namespace in self.get_node_names_and_namespaces()])

    def flush_tiers(self):
        try:request=tiers_requests.get_nowait()
        except queue.Empty:return
        if 0<=time.monotonic()-request['requested_monotonic']<1:
            self.tiers_pub.publish(String(data=json.dumps(request)))

    def on_communications(self, msg):
        try:
            communications.update(json.loads(msg.data))
        except (ValueError,TypeError) as exc:
            self.get_logger().warning('Invalid communications status: '+str(exc))

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

    def on_navigation_status(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                navigation.put('supervisor', value)
        except (ValueError, TypeError):
            pass

    def on_dvl(self, msg):
        navigation.put('dvl_valid', {'valid': msg.data})
        tel['dvl_lock'], tel['dvl_t'] = msg.data, time.time()

    def on_alt(self, msg):
        navigation.put('clearance', {'meters': msg.data if msg.data >= 0 else None})
        tel['altitude'] = msg.data

    def on_hdg(self, msg):
        navigation.put('vn_heading', {'degrees': msg.data})
        tel['heading'], tel['hdg_t'] = msg.data, time.time()

    def on_fcu_summary(self, kind, msg):
        try:
            data = json.loads(msg.data)
            received, fields = float(data['received_monotonic']), dict(data['fields'])
        except (ValueError, TypeError, KeyError):
            return
        age = time.monotonic() - received
        if not 0 <= age < 5.0:              # another clock, or very old: never shown as new
            return
        frame = SummaryFrame(kind, fields)
        try:
            navigation.consume(kind, frame, received=received)
            if kind == 'HEARTBEAT':
                tel['armed'] = bool(frame.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                tel['mode'] = mavutil.mode_string_v10(frame)
                tel['mav_t'] = time.time() - age
        except (AttributeError, TypeError, ValueError):
            return

    def pump(self):
        def consume(kind, msg):
            if kind in SUMMARY_KINDS.values():
                return                      # these come from the gateway's summaries (on_fcu_summary)
            navigation.consume(kind, msg)
            if kind == 'LOCAL_POSITION_NED':
                tel['depth'] = msg.z

        self.link.drain(consume)

    def destroy_node(self):
        if hasattr(self,'recording_service'):self.recording_service.close()
        if hasattr(self, 'http_server'):
            self.http_server.shutdown()
            self.http_server.server_close()
        return super().destroy_node()


def main():
    run(GuiNode)
