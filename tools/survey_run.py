#!/usr/bin/env python3
"""The survey run (Ruth, Nov 10): fully autonomous from one press of SE on the RadioMaster.

  1. Waits for SE (CH9, momentary). While it waits it sends nothing to the sub: drive it on the radio,
     move points in QGC, upload.
  2. On SE: reads the points plotted in QGC and uploaded to the Cube (QGC Plan view -> Upload).
     The last two are the START buoy and the END buoy; any before them are the path to the start buoy.
  3. Drives at the surface by GPS through the points, up to and including the start buoy - Beichen's
     waypoint mission, one route run, started through the GUI on this Jetson (as the Waypoints mission does).
  4. At the start buoy: starts a sonar recording and a camera recording for this run, then runs
     pipeline_mission: turn at the surface to face the end buoy, dive until the DVL reads pool_alt_m (4 m) to
     the floor (no dive where the water is shallower), transit only - no pipe checks - keeping pool_alt_m above
     the floor, for exactly the distance between the last two points, surface, disarm.
  5. Stops both recordings and waits for SE again.

Runs inside the graey container: tools/survey.sh starts it (sonar map and cameras first).
  --dry        nothing moves: no waypoint runs, pipeline_mission dry_run (its stages time out after 20 s)
  --skip-path  pool test: no surface legs, the transit starts where the sub is (bearing and distance from
               the last two points)
  anything after -- goes to pipeline_mission:   -- -p transit_speed:=0.15

From the GUI (Nov 11): the Missions page's "Survey run (Ruth)" starts this through gui_node with
  --points '[[lat, lon], ...]' --run-id ID      the GUI's route points (the last two are the start and the end
                                               buoy); one run at once - no SE, no QGC plan - then it exits.
  Stop mission sends SIGUSR1: a surface leg is aborted, the transit surfaces (pipeline_mission's STOP), the
  recordings stop, and it exits.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from robotx_graey_2026.api.pixhawk.mavlink import Link  # noqa: E402 - sets up pymavlink
from robotx_graey_2026.api.pixhawk.identity import FCU_COMPONENT, FCU_SYSTEM  # noqa: E402

SE_CHANNEL = 9              # SE, momentary (mission_base.py's rc_start_channel)
RC_HIGH = 1700              # pressed above this (mission_base.py's rc_high)
LEG_TIMEOUT_S = 600.0       # per route point (Beichen's mission) - abort the route after this x points
NAV_WAYPOINT = 16           # MAV_CMD_NAV_WAYPOINT
COMPONENT = 198             # the mission component in Beichen's telemetry_gateway
MISSION_CMD = [sys.executable, '-c',
               'from robotx_graey_2026.api.navigation.pipeline_mission import main; main()']

LOG_FILE = None
RUN_ID = None               # the GUI's survey run ID (GUI mode): its own leg starts quote it to gui_node
STOP = []                   # non-empty once the GUI's Stop mission arrived (SIGUSR1)


def stopped():
    return bool(STOP)


def log(msg):
    line = time.strftime('%H:%M:%S ') + msg
    print(line, flush=True)
    if LOG_FILE:
        try:
            with open(LOG_FILE, 'a') as f:
                f.write(line + '\n')
        except OSError:
            pass


class Logger:                   # what Link wants
    def info(self, m): log(m)
    warn = warning = error = info


def distance_bearing(a, b):
    """(metres, bearing deg from true north) from GPS point a to b - pipeline_mission's buoy_distance."""
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dp, dl = p2 - p1, math.radians(b[1] - a[1])
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    brg = math.atan2(math.sin(dl) * math.cos(p2), math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl))
    return 2 * 6371000.0 * math.asin(math.sqrt(h)), math.degrees(brg) % 360


# ---------------- MAVLink: the SE button and the QGC plan ----------------

class Fcu:
    """MAVLink through Beichen's telemetry_gateway (Nov 9): it owns the only MAVProxy port and passes frames
    over the ROS topics /graey/fcu/rx and /graey/fcu/tx. We send as the mission component, 198 - the gateway
    refuses unregistered ones. It already streams RC_CHANNELS at 10 Hz, so we ask for nothing."""

    def __init__(self):
        import threading
        import rclpy
        from rclpy.node import Node
        # No rclpy signal handlers: Ctrl-C stays a KeyboardInterrupt and SIGTERM still ends the program
        # (with rclpy's handlers only the spin thread stopped and this kept waiting for SE).
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.rclpy = rclpy
        self.node = Node('survey_run')
        self.link = Link('ros', COMPONENT, Logger(), node=self.node)
        self.spin = threading.Thread(target=self._spin, daemon=True)
        self.spin.start()

    def _spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception:       # shutdown
            pass

    def send(self, name, *args):
        self.link.send(name, *args)

    def recv(self, types, timeout):
        """The next fresh message from the Cube of one of these types, or None after timeout s."""
        end = time.monotonic() + timeout
        while True:
            got = []
            self.link.drain(lambda kind, m: got.append(m) if kind in types
                            and m.get_srcSystem() == FCU_SYSTEM
                            and m.get_srcComponent() == FCU_COMPONENT else None)
            if got:
                return got[0]
            if time.monotonic() >= end:
                return None
            time.sleep(0.01)

    def close(self):
        try:
            self.rclpy.shutdown()       # wakes the spin thread so it ends before the node goes
        except Exception:
            pass
        self.spin.join(timeout=2.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass


def wait_for_se(fcu):
    """Until SE goes from released to pressed. Old RC values from before count as released."""
    seen_low = False
    while True:
        m = fcu.recv(('RC_CHANNELS',), 0.5)
        if m is None:
            continue
        v = getattr(m, f'chan{SE_CHANNEL}_raw', 0)
        if 800 < v < RC_HIGH:
            seen_low = True
        elif v > RC_HIGH and seen_low:
            return


def read_plan(fcu):
    """The points uploaded from QGC: the Cube's mission items 1.. that are NAV_WAYPOINTs (item 0 is the
    Cube's home). Reading changes nothing on the Cube."""
    for _ in range(3):
        fcu.send('mission_request_list_send', 1, 1, 0)
        cnt = fcu.recv(('MISSION_COUNT',), 3)
        if cnt is not None:
            break
    else:
        raise RuntimeError('the Cube did not answer the request for the QGC plan (is the gateway letting '
                           'MISSION_REQUEST_LIST through? /graey/fcu/gateway_status shows its last refusal)')
    items = []
    for seq in range(cnt.count):
        for _ in range(3):
            fcu.send('mission_request_int_send', 1, 1, seq, 0)
            it = fcu.recv(('MISSION_ITEM_INT', 'MISSION_ITEM'), 2)
            if it is not None and it.seq == seq:
                break
        else:
            raise RuntimeError(f'point {seq} of the QGC plan did not arrive')
        items.append(it)
    fcu.send('mission_ack_send', 1, 1, 0, 0)        # MAV_MISSION_ACCEPTED: the download is finished
    pts = []
    for it in items:
        if it.seq == 0 or it.command != NAV_WAYPOINT:
            continue
        lat, lon = (it.x / 1e7, it.y / 1e7) if it.get_type() == 'MISSION_ITEM_INT' else (it.x, it.y)
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            continue
        pts.append((lat, lon))
    return pts


# ---------------- HTTP: Beichen's waypoint runs, the recordings ----------------

def http(url, body=None, json_body=None, timeout=5.0):
    if json_body is not None:
        req = urllib.request.Request(url, data=json.dumps(json_body).encode(),
                                     headers={'Content-Type': 'application/json'}, method='POST')
    elif body is not None:
        req = urllib.request.Request(url, data=body.encode(), method='POST')
    else:
        req = urllib.request.Request(url)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            text = r.read().decode()
    except urllib.error.HTTPError as e:
        text = e.read().decode(errors='replace')
    try:
        return json.loads(text)
    except ValueError:
        return {'ok': True, 'text': text}


def drive_route(args, legs):
    """The surface part: the path points and the start buoy as ONE route run of Beichen's waypoint mission - what
    the GUI's Waypoints mission sends (preview a route, start, poll). It arms itself, drives through the points at
    the surface and stops at the last. (True, why) on COMPLETE, (False, why) if refused, aborted or timed out.
    Not 'qgc_preview' per point (Nov 10): that is QGC's Go-to, which never arms - SITL Nov 11: 'QGC Go-to cancelled
    by disarm' - and each run ends disarmed."""
    api = f'http://127.0.0.1:{args.gui}/api/waypoint'
    points = [{'kind': 'waypoint', 'lat': lat, 'lon': lon, 'name': f'Survey {n}'} for n, (lat, lon) in enumerate(legs, 1)]
    p = http(api, json_body={'action': 'preview', 'plan': {'points': points, 'recording_mode': 'manual'}})
    if not p.get('ok') or 'plan_id' not in p:
        return False, f'the GUI refused the route: {p.get("msg", p)}'
    log(f'  route: {len(points)} point(s), the first {p.get("distance_m", float("nan")):.1f} m away')
    start = {'action': 'start', 'plan_id': p['plan_id'], 'competition': args.competition}
    if RUN_ID:
        start['survey_run_id'] = RUN_ID     # gui_node lets the survey's own route start while it runs
    s = http(api, json_body=start)
    if not s.get('ok') or 'run_id' not in s:
        return False, f'the GUI refused to start it: {s.get("msg", s)}'
    run_id, t0, last = s['run_id'], time.monotonic(), None
    while time.monotonic() - t0 < LEG_TIMEOUT_S * len(points):
        if stopped():
            http(api, json_body={'action': 'abort', 'run_id': run_id})
            return False, 'Stop mission from the GUI - asked the waypoint mission to abort'
        snap = http(api)
        w = (((snap.get('navigation') or {}).get('streams') or {}).get('waypoint') or {}).get('data') or {}
        if w.get('run_id') == run_id:
            now = (w.get('stage'), w.get('reason', ''))
            if now != last:
                log(f'  route: {now[0]} - {now[1]}')
                last = now
            if now[0] == 'COMPLETE':
                return True, now[1]
            if now[0] == 'ABORTED':
                return False, now[1] or 'aborted'
        time.sleep(1.0)
    http(api, json_body={'action': 'abort', 'run_id': run_id})
    return False, f'no arrival in {LEG_TIMEOUT_S * len(points):.0f} s - asked the waypoint mission to abort'


def up(port):
    try:
        urllib.request.urlopen(f'http://127.0.0.1:{port}/', timeout=2).close()
        return True
    except urllib.error.HTTPError:
        return True                     # it answered
    except OSError:
        return False


def ensure_recorders(args):
    """What tools/survey.sh does first: the sonar map and the cameras, through the GUI's launcher.
    False if the sonar map isn't running at the end."""
    for port, cmd, name in ((args.map, 'start?tool=map', 'the sonar map'), (8080, 'camera/start', 'the cameras')):
        if up(port):
            continue
        log(f'starting {name}...')
        try:
            http(f'http://127.0.0.1:{args.launcher}/{cmd}', body='', timeout=20)
        except OSError as e:
            log(f'  the GUI\'s launcher (port {args.launcher}) didn\'t answer: {e}')
        for _ in range(30):
            if up(port):
                break
            time.sleep(1.0)
        log(f'  {name}: {"running" if up(port) else "NOT running"}')
    return up(args.map)


def recordings(args, run, start):
    if os.environ.get('GRAEY_RUNTIME_SCOPE') == 'sitl':
        log(f'  [sitl] recordings {"started" if start else "stopped"} - skipped, no sonar or cameras in SITL')
        return
    cam = http(f'http://127.0.0.1:{args.launcher}/' +
               (f'record/start?name=survey_run_{run:02d}' if start else 'record/stop'), body='')
    try:
        son = http(f'http://127.0.0.1:{args.map}/record', body=f'start survey run {run}' if start else 'stop', timeout=2)
    except OSError as e:
        son = {'ok': False, 'msg': str(e)}
    log(f'  recordings {"started" if start else "stopped"}: cameras {cam.get("msg", cam.get("text", cam))!s:.80}, '
        f'sonar {son.get("msg", son.get("text", son))!s:.80}')


# ---------------- one run ----------------

def one_run(args, fcu, run, pts=None):
    """One survey. pts: the GUI's points; None = read the QGC plan from the Cube.
    Returns 'done', 'stopped' (Stop mission) or 'failed'."""
    after = 'Waiting for SE again.' if pts is None else 'Survey over.'
    if pts is None:
        try:
            pts = read_plan(fcu)
        except (RuntimeError, OSError) as e:
            log(f'run {run}: {e} - not started')
            return 'failed'
        where = 'the QGC plan'
    else:
        where = 'the GUI route'
    if len(pts) < 2:
        log(f'run {run}: {where} has {len(pts)} point(s) - it needs at least 2 (the last two are the '
            f'start buoy and the end buoy). Not started.')
        return 'failed'
    start, end = pts[-2], pts[-1]
    dist, brg = distance_bearing(start, end)
    log(f'run {run}: {len(pts)} points - path {len(pts) - 2} point(s), start buoy {start[0]:.7f} {start[1]:.7f}, '
        f'end buoy {end[0]:.7f} {end[1]:.7f}: {dist:.1f} m at {brg:.0f} deg')
    if dist < 1.0:
        log(f'run {run}: the start and end buoys are only {dist:.2f} m apart - not started')
        return 'failed'
    legs = [] if args.skip_path else pts[:-1]       # the path, then the start buoy itself
    for n, (lat, lon) in enumerate(legs, 1):
        tag = 'the start buoy' if n == len(legs) else f'path point {n}'
        log(f'run {run}: surface route point {n} of {len(legs)}: {tag} at {lat:.7f} {lon:.7f}')
    if legs and args.dry:
        log('  [dry] Beichen\'s waypoint mission would drive this route at the surface')
    elif legs:
        ok, why = drive_route(args, legs)
        if not ok:
            log(f'run {run}: the surface route stopped: {why}. {after}')
            return 'stopped' if stopped() else 'failed'
        log(f'  reached the start buoy: {why}')
    if stopped():
        log(f'run {run}: Stop mission - no transit. {after}')
        return 'stopped'
    recordings(args, run, True)
    argv = MISSION_CMD + ['--ros-args', '-p', 'part:=search', '-p', f'dry_run:={str(args.dry).lower()}',
                          '-p', 'scan_mode:=transit', '-p', 'transit_only:=true', '-p', 'face_end_buoy:=true',
                          '-p', 'transit_follow_floor:=true', '-p', 'record_external:=true',
                          '-p', f'start_buoy_gps:={start[0]:.7f} {start[1]:.7f}',
                          '-p', f'end_buoy_gps:={end[0]:.7f} {end[1]:.7f}', '-p', 'end_short_m:=0.0',
                          '-p', f'pool_alt_m:={args.alt:.2f}']
    if args.dry:
        argv += ['-p', 'phase_timeout:=20.0']
    argv += args.extra
    log(f'run {run}: the transit: {" ".join(argv[3:])}')
    passed_on = False
    try:
        child = subprocess.Popen(argv)
        while child.poll() is None:
            if stopped() and not passed_on:
                log(f'run {run}: Stop mission - the transit surfaces now')
                child.send_signal(signal.SIGUSR1)       # pipeline_mission: SURFACE, then disarm + MANUAL
                passed_on = True
            time.sleep(0.1)
        code = child.returncode
    except (OSError, KeyboardInterrupt) as e:
        code = repr(e)
    recordings(args, run, False)
    log(f'run {run}: {"stopped" if stopped() else "done"} (pipeline_mission exit {code}). {after}')
    return 'stopped' if stopped() else 'done' if code == 0 else 'failed'


def gui_run(args):
    """GUI mode: one survey of the GUI's points, then exit (0 done, 1 failed, 2 stopped)."""
    global RUN_ID
    RUN_ID = args.run_id
    signal.signal(signal.SIGUSR1, lambda *_: STOP.append(True))
    try:
        pts = [(float(p[0]), float(p[1])) for p in json.loads(args.points)]
    except (ValueError, TypeError, IndexError) as e:
        log(f'the GUI points are not [[lat, lon], ...]: {e}')
        return 1
    if os.environ.get('GRAEY_RUNTIME_SCOPE') != 'sitl' and not ensure_recorders(args):
        log('the sonar map is not running (look at Sensors) - survey not started')
        return 1
    try:
        result = one_run(args, None, 1, pts)
    except KeyboardInterrupt:
        result = 'stopped'
    log(f'SURVEY {result.upper()}')
    return {'done': 0, 'failed': 1, 'stopped': 2}[result]


def main():
    global LOG_FILE
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--competition', action='store_true',
                    help='hold the first surface route in POSHOLD until verified RoboCommand RunStart')
    ap.add_argument('--skip-path', action='store_true')
    ap.add_argument('--alt', type=float, default=4.0, help='m above the floor (pool_alt_m)')
    ap.add_argument('--gui', type=int, default=8090)
    ap.add_argument('--launcher', type=int, default=8096)
    ap.add_argument('--map', type=int, default=8095)
    ap.add_argument('--logdir', default='/root/robotx_ws/logs')
    ap.add_argument('--mission-cmd', default=None, help=argparse.SUPPRESS)     # tests only
    ap.add_argument('--points', default=None, help='GUI mode: JSON [[lat, lon], ...]; one run, no SE')
    ap.add_argument('--run-id', default=None, help='GUI mode: the run ID gui_node gave this survey')
    ap.add_argument('extra', nargs=argparse.REMAINDER)
    args = ap.parse_args()
    args.extra = [a for a in args.extra if a != '--']
    if args.mission_cmd:
        MISSION_CMD[:] = args.mission_cmd.split()
    if os.path.isdir(args.logdir):
        LOG_FILE = os.path.join(args.logdir, time.strftime('survey-%Y%m%d-%H%M%S.log'))
    log(f'survey run{" (DRY - nothing moves)" if args.dry else ""}{" (pool test: no path)" if args.skip_path else ""}'
        f'{" from the GUI" if args.points else ""} - log {LOG_FILE}')
    if args.points is not None:
        sys.exit(gui_run(args))
    run = 0
    fcu = Fcu()
    try:
        while True:
            log('waiting for SE on the RadioMaster (drive the sub on the radio and move points in QGC meanwhile)')
            wait_for_se(fcu)
            run += 1
            log(f'SE pressed - run {run}')
            one_run(args, fcu, run)
    except KeyboardInterrupt:
        log('stopped (Ctrl-C)')
    finally:
        fcu.close()


if __name__ == '__main__':
    main()
