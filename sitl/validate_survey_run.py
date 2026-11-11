"""SITL test of Ruth's survey run started from the GUI (Nov 11), localhost only.

Real ArduSub SITL and Beichen's SITL stack (sitl/run.sh: navigation bridge, supervisor, synthetic sensors,
gui_node), plus what the survey's transit needs and that stack doesn't start: telemetry_gateway (pipeline_mission
talks to the Cube through it), a flat fake floor FLOOR_M deep for the DVL altitude, and a stand-in sonar map
(8095: the transit sets it up and surfaces after 10 s without it) and camera launcher (8096). No sonar data or
pictures: survey_run skips the recordings in SITL. And the kill-switch watchdog's status in its UNLOCKED state
(SA released, ESC on - what the real one reports once unlocked), stamped as the gateway requires: the SITL
fixture's status has no published_monotonic and ESC off while disarmed, so the gateway refuses every mode, arm
and setpoint from the mission component (Beichen's waypoint mission in SITL bypasses the gateway).

Starts the survey through the GUI API exactly as the Missions page's Start mission does (action survey_start),
then checks: every surface leg COMPLETE, the transit dives to ~4 m above the floor and surfaces, SURVEY DONE.
--stop: presses Stop mission once the transit is under water; checks it surfaces and ends STOPPED.
--dry: the dry run (nothing moves).
"""
import argparse
import http.server
import json
import os
import signal
import subprocess
import threading
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

from robotx_graey_2026.api.navigation.waypoint_logic import ne_to_geo

FLOOR_M = 6.0           # fake floor depth: 4 m above it = 2 m deep
ALT_M = 4.0
GATEWAY_PORT = 14551    # MAVProxy udpin the gateway owns on Graey (sitl/run.sh opens it; nothing else uses it now)

FLOOR_NODE = r'''
import sys, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
floor = float(sys.argv[1]); depth = [0.0]
rclpy.init(); n = Node('sitl_fake_floor')
pub = n.create_publisher(Float32, '/graey/dvl/altitude', 10)
n.create_subscription(Float32, '/graey/sitl/depth_m', lambda m: depth.__setitem__(0, m.data), 10)
n.create_timer(0.1, lambda: pub.publish(Float32(data=float(max(0.2, floor - depth[0])))))
rclpy.spin(n)
'''


class FakeSonarAndLauncher(http.server.BaseHTTPRequestHandler):
    """Answers what pipeline_mission asks the sonar map (8095) and the launcher (8096); POSTs are logged."""
    t0 = time.monotonic()
    posts = []

    def log_message(self, *args):
        pass

    def reply(self, body):
        data = json.dumps(body).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith('/mapdata'):
            return self.reply(dict(sweep=int((time.monotonic()-self.t0)*2), sonar=[0., 0.], outlines=[], crumbs=[],
                                   running=True, heading=0., state='running', pose=[0., 0., 0.]))
        self.reply(dict(ok=True))

    def do_POST(self):
        n = int(self.headers.get('Content-Length', '0') or 0)
        FakeSonarAndLauncher.posts.append((self.server.server_address[1], self.path, self.rfile.read(n).decode()))
        self.reply(dict(ok=True, msg='SITL stand-in'))


CONTROL_NODE = r'''
import json, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init(); n = Node('sitl_unlocked_watchdog'); armed = [False]
def heartbeat(m):
    armed[0] = bool(json.loads(m.data).get('fields', {}).get('base_mode', 0) & 128)
n.create_subscription(String, '/graey/fcu/status', heartbeat, 10)
pub = n.create_publisher(String, '/graey/control_watchdog/status', 10)
n.create_timer(0.1, lambda: pub.publish(String(data=json.dumps(dict(
    published_monotonic=time.monotonic(), session='sitl-unlocked', vehicle_fresh=True, online=True,
    sa_active=False, esc_commanded_on=True, inhibit=None, armed=armed[0], autonomous=False)))))
rclpy.spin(n)
'''


def api(body=None):
    req = urllib.request.Request('http://127.0.0.1:8090/api/waypoint',
                                 data=json.dumps(body).encode() if body else None,
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=3) as response:
            return json.load(response)
    except urllib.error.HTTPError as e:
        return json.loads(e.read().decode() or '{}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--stop', action='store_true', help='press Stop mission during the transit')
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--timeout', type=float, default=1200.)
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[1]
    folder = root/'sitl/logs'/('survey-'+uuid.uuid4().hex[:10])
    folder.mkdir(parents=True)
    env = dict(os.environ, SITL_LOG_DIR=str(folder), SITL_RUN_DIR=str(folder/'runtime'),
               WAYPOINT_INTERACTIVE='true', SITL_CONTROL_FIXTURE='true', CONTINUOUS_FUSION='true',
               GRAEY_WAYPOINT_LOG_DIR=str(folder), GRAEY_SOURCE_DIR=str(root),
               # sitl/run.sh's isolated ROS network: the extra nodes below must join it
               ROS_DOMAIN_ID='73', ROS_LOCALHOST_ONLY='1', GRAEY_RUNTIME_SCOPE='sitl')
    env.pop('QGC_HOST', None)
    result = dict(passed=False, legs=[], stages=[], max_depth_m=0., final_depth_m=None, survey=None,
                  stop=args.stop, dry=args.dry)
    procs = []
    for port in (8095, 8096):
        server = http.server.ThreadingHTTPServer(('127.0.0.1', port), FakeSonarAndLauncher)
        threading.Thread(target=server.serve_forever, daemon=True).start()
    run_id = None
    stop_sent = False
    try:
        with (folder/'runner.log').open('w') as log:
            procs.append(subprocess.Popen(['bash', 'sitl/run.sh'], cwd=root, env=env, stdout=log,
                                          stderr=subprocess.STDOUT, start_new_session=True))
        ros = 'source /opt/ros/humble/setup.bash && source install/setup.bash && '
        extras = False
        deadline = time.monotonic()+args.timeout
        with (folder/'observations.jsonl').open('w') as obs:
            while time.monotonic() < deadline:
                if procs[0].poll() is not None:
                    raise RuntimeError('SITL exited before completion')
                if not extras and (root/'install/setup.bash').exists() and (folder/'ros.log').exists():
                    for name, cmd in (('gateway', f'ros2 run robotx_graey_2026 telemetry_gateway --ros-args '
                                                  f'-p mavlink:=udpout:127.0.0.1:{GATEWAY_PORT}'),
                                      ('floor', f'python3 -c "$FLOOR_NODE" {FLOOR_M}'),
                                      ('watchdog', 'python3 -c "$CONTROL_NODE"')):
                        with (folder/(name+'.log')).open('w') as log:
                            procs.append(subprocess.Popen(['bash', '-c', ros+cmd], cwd=root,
                                                          env=dict(env, FLOOR_NODE=FLOOR_NODE, CONTROL_NODE=CONTROL_NODE), stdout=log,
                                                          stderr=subprocess.STDOUT, start_new_session=True))
                    extras = True
                try:
                    state = api()
                except OSError as exc:
                    result['last_retry'] = str(exc)
                    time.sleep(1.)
                    continue
                if state.get('capabilities', {}).get('simulation') is not True:
                    raise RuntimeError('Refusing a non-SITL GUI')
                nav = state['navigation']
                streams = nav['streams']
                bridge = streams.get('bridge', {}).get('data', {})
                cube = nav['cube_estimate']
                if not run_id and cube.get('valid') and bridge.get('configuration_verified'):
                    time.sleep(5.)      # the gateway's first control-status sample
                    origin = (cube['lat'], cube['lon'])
                    # a path point, the start buoy, the end buoy 10 m east of it
                    points = [dict(zip(('lat', 'lon'), ne_to_geo(n, e, origin))) for n, e in ((3., 0.), (3., 3.), (3., 13.))]
                    reply = api(dict(action='survey_start', points=points, dry=args.dry))
                    if not reply.get('ok'):
                        raise RuntimeError('survey_start refused: '+reply.get('msg', str(reply)))
                    run_id = reply['run_id']
                    result['points'] = points
                    print('survey started', run_id, flush=True)
                if not run_id:
                    time.sleep(.5)
                    continue
                survey = state.get('survey') or {}
                depth = bridge.get('depth_m')
                if isinstance(depth, (int, float)):
                    result['max_depth_m'] = max(result['max_depth_m'], depth)
                leg = streams.get('waypoint', {}).get('data', {})
                if leg.get('run_id') and leg.get('stage') in ('COMPLETE', 'ABORTED'):
                    if not result['legs'] or result['legs'][-1]['run_id'] != leg['run_id']:
                        result['legs'].append(dict(run_id=leg['run_id'], stage=leg['stage'], reason=leg.get('reason')))
                        print('leg', leg['stage'], leg.get('reason'), flush=True)
                line = (survey.get('log') or [''])[-1]
                if not result['stages'] or result['stages'][-1] != line:
                    result['stages'].append(line)
                    print(line, flush=True)
                obs.write(json.dumps(dict(depth=depth, survey_running=survey.get('running'), leg=leg.get('stage'),
                                          cube=cube))+'\n')
                if args.stop and not stop_sent and isinstance(depth, (int, float)) and depth > 1.2:
                    reply = api(dict(action='survey_stop', run_id=run_id))
                    stop_sent = True
                    result['stop_reply'] = reply
                    print('STOP sent at depth', depth, reply, flush=True)
                if survey.get('run_id') == run_id and not survey.get('running'):
                    time.sleep(3.)
                    final = api()
                    result['survey'] = final['survey']
                    d = final['navigation']['streams'].get('bridge', {}).get('data', {}).get('depth_m')
                    result['final_depth_m'] = d
                    want = 'STOPPED' if args.stop else 'DONE'
                    target = FLOOR_M-ALT_M
                    result['passed'] = (final['survey']['result'] == want and d is not None and d < .3
                                        and (args.dry or args.stop
                                             or abs(result['max_depth_m']-target) < .4)
                                        and (args.dry or all(l['stage'] == 'COMPLETE' for l in result['legs'][:2])))
                    break
                time.sleep(.5)
            else:
                result['error'] = 'Validation timed out'
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        for proc in reversed(procs):
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for proc in reversed(procs):
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        result['sonar_and_launcher_posts'] = FakeSonarAndLauncher.posts[-40:]
        (folder/'validation.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(evidence=str(folder), **{k: v for k, v in result.items() if k != 'stages'})), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
