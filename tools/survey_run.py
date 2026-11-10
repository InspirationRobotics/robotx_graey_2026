#!/usr/bin/env python3
"""The survey run (Ruth, Nov 10): fully autonomous from one press of SE on the RadioMaster.

  1. Waits for SE (CH9, momentary). While it waits it sends nothing to the sub: drive it on the radio,
     move points in QGC, upload.
  2. On SE: reads the points plotted in QGC and uploaded to the Cube (QGC Plan view -> Upload).
     The last two are the START buoy and the END buoy; any before them are the path to the start buoy.
  3. Drives at the surface by GPS to each point in turn, up to and including the start buoy - Beichen's
     waypoint mission, one run per point, started through the GUI on this Jetson (as the Waypoint tab does).
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
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from robotx_graey_2026.api.pixhawk.mavlink import Link  # noqa: E402 - sets up pymavlink

SE_CHANNEL = 9              # SE, momentary (mission_base.py's rc_start_channel)
RC_HIGH = 1700              # pressed above this (mission_base.py's rc_high)
LEG_TIMEOUT_S = 600.0       # one surface leg (Beichen's mission) - abort it after this long
NAV_WAYPOINT = 16           # MAV_CMD_NAV_WAYPOINT
MISSION_CMD = [sys.executable, '-c',
               'from robotx_graey_2026.api.navigation.pipeline_mission import main; main()']

LOG_FILE = None


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

def wait_for_se(link):
    """Until SE goes from released to pressed. Old RC values from before count as released."""
    seen_low, last_hb, last_req = False, 0.0, 0.0
    while True:
        now = time.monotonic()
        if now - last_hb >= 1.0:
            link.heartbeat()        # MAVProxy only sends to us after we send
            last_hb = now
        if now - last_req >= 5.0:   # ArduPilot only streams RC_CHANNELS if something asks
            try:
                link.mav.mav.command_long_send(1, 1, 511, 0, 65, 100000, 0, 0, 0, 0, 0)   # SET_MESSAGE_INTERVAL
            except OSError:
                pass
            last_req = now
        try:
            m = link.mav.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.5)
        except OSError:
            m = None
        if m is None:
            continue
        v = getattr(m, f'chan{SE_CHANNEL}_raw', 0)
        if 800 < v < RC_HIGH:
            seen_low = True
        elif v > RC_HIGH and seen_low:
            return


def read_plan(link):
    """The points uploaded from QGC: the Cube's mission items 1.. that are NAV_WAYPOINTs (item 0 is the
    Cube's home). Reading changes nothing on the Cube."""
    mav = link.mav
    for _ in range(3):
        mav.mav.mission_request_list_send(1, 1, 0)
        cnt = mav.recv_match(type='MISSION_COUNT', blocking=True, timeout=3)
        if cnt is not None:
            break
    else:
        raise RuntimeError('the Cube did not answer the request for the QGC plan')
    items = []
    for seq in range(cnt.count):
        for _ in range(3):
            mav.mav.mission_request_int_send(1, 1, seq, 0)
            it = mav.recv_match(type=['MISSION_ITEM_INT', 'MISSION_ITEM'], blocking=True, timeout=2)
            if it is not None and it.seq == seq:
                break
        else:
            raise RuntimeError(f'point {seq} of the QGC plan did not arrive')
        items.append(it)
    mav.mav.mission_ack_send(1, 1, 0, 0)        # MAV_MISSION_ACCEPTED: the download is finished
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


def drive_to(args, n, lat, lon):
    """One surface leg: Beichen's waypoint mission to (lat, lon). (True, why) once it reports COMPLETE
    (stopped within 1.5 m and disarmed), (False, why) if it refused, aborted or timed out."""
    api = f'http://127.0.0.1:{args.gui}/api/waypoint'
    p = http(api, json_body={'action': 'qgc_preview',
                             'plan': {'lat': lat, 'lon': lon, 'mode': 'surface', 'map': 'geographic'}})
    if not p.get('ok') or 'plan_id' not in p:
        return False, f'the GUI refused the point: {p.get("msg", p)}'
    log(f'  point {n}: {p.get("distance_m", float("nan")):.1f} m away')
    s = http(api, json_body={'action': 'start', 'plan_id': p['plan_id']})
    if not s.get('ok') or 'run_id' not in s:
        return False, f'the GUI refused to start it: {s.get("msg", s)}'
    run_id, t0, last = s['run_id'], time.monotonic(), None
    while time.monotonic() - t0 < LEG_TIMEOUT_S:
        snap = http(api)
        w = (((snap.get('navigation') or {}).get('streams') or {}).get('waypoint') or {}).get('data') or {}
        if w.get('run_id') == run_id:
            stage = w.get('stage')
            if stage != last:
                log(f'  point {n}: {stage} - {w.get("reason", "")}')
                last = stage
            if stage == 'COMPLETE':
                return True, w.get('reason', '')
            if stage == 'ABORTED':
                return False, w.get('reason', 'aborted')
        time.sleep(1.0)
    http(api, json_body={'action': 'abort', 'run_id': run_id})
    return False, f'no arrival in {LEG_TIMEOUT_S:.0f} s - asked the waypoint mission to abort'


def recordings(args, run, start):
    cam = http(f'http://127.0.0.1:{args.launcher}/' +
               (f'record/start?name=survey_run_{run:02d}' if start else 'record/stop'), body='')
    try:
        son = http(f'http://127.0.0.1:{args.map}/record', body=f'start survey run {run}' if start else 'stop', timeout=2)
    except OSError as e:
        son = {'ok': False, 'msg': str(e)}
    log(f'  recordings {"started" if start else "stopped"}: cameras {cam.get("msg", cam.get("text", cam))!s:.80}, '
        f'sonar {son.get("msg", son.get("text", son))!s:.80}')


# ---------------- one run ----------------

def one_run(args, link, run):
    try:
        pts = read_plan(link)
    except (RuntimeError, OSError) as e:
        log(f'run {run}: {e} - not started')
        return
    if len(pts) < 2:
        log(f'run {run}: the QGC plan has {len(pts)} point(s) - it needs at least 2 (the last two are the '
            f'start buoy and the end buoy). Not started.')
        return
    start, end = pts[-2], pts[-1]
    dist, brg = distance_bearing(start, end)
    log(f'run {run}: {len(pts)} points - path {len(pts) - 2} point(s), start buoy {start[0]:.7f} {start[1]:.7f}, '
        f'end buoy {end[0]:.7f} {end[1]:.7f}: {dist:.1f} m at {brg:.0f} deg')
    if dist < 1.0:
        log(f'run {run}: the start and end buoys are only {dist:.2f} m apart - not started')
        return
    legs = [] if args.skip_path else pts[:-1]       # the path, then the start buoy itself
    for n, (lat, lon) in enumerate(legs, 1):
        tag = 'the start buoy' if n == len(legs) else f'path point {n}'
        log(f'run {run}: to {tag} ({n} of {len(legs)}) at {lat:.7f} {lon:.7f}')
        if args.dry:
            log('  [dry] Beichen\'s waypoint mission would drive there at the surface')
            continue
        ok, why = drive_to(args, n, lat, lon)
        if not ok:
            log(f'run {run}: stopped at {tag}: {why}. Waiting for SE again.')
            return
        log(f'  reached {tag}: {why}')
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
    try:
        code = subprocess.call(argv)
    except (OSError, KeyboardInterrupt) as e:
        code = repr(e)
    recordings(args, run, False)
    log(f'run {run}: done (pipeline_mission exit {code}). Waiting for SE again.')


def main():
    global LOG_FILE
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--skip-path', action='store_true')
    ap.add_argument('--alt', type=float, default=4.0, help='m above the floor (pool_alt_m)')
    ap.add_argument('--mavlink', default='udpout:127.0.0.1:14553')
    ap.add_argument('--gui', type=int, default=8090)
    ap.add_argument('--launcher', type=int, default=8096)
    ap.add_argument('--map', type=int, default=8095)
    ap.add_argument('--logdir', default='/root/robotx_ws/logs')
    ap.add_argument('--mission-cmd', default=None, help=argparse.SUPPRESS)     # tests only
    ap.add_argument('extra', nargs=argparse.REMAINDER)
    args = ap.parse_args()
    args.extra = [a for a in args.extra if a != '--']
    if args.mission_cmd:
        MISSION_CMD[:] = args.mission_cmd.split()
    if os.path.isdir(args.logdir):
        LOG_FILE = os.path.join(args.logdir, time.strftime('survey-%Y%m%d-%H%M%S.log'))
    log(f'survey run{" (DRY - nothing moves)" if args.dry else ""}{" (pool test: no path)" if args.skip_path else ""}'
        f' - log {LOG_FILE}')
    run = 0
    while True:
        # The mission port is shared: Beichen's waypoint mission and pipeline_mission use it while they run,
        # so this program opens it only to wait for SE and read the plan.
        link = Link(args.mavlink, 192, Logger())
        log('waiting for SE on the RadioMaster (drive the sub on the radio and move points in QGC meanwhile)')
        try:
            wait_for_se(link)
        except KeyboardInterrupt:
            log('stopped (Ctrl-C)')
            return
        run += 1
        log(f'SE pressed - run {run}')
        try:
            one_run(args, link, run)
        except KeyboardInterrupt:
            log('stopped (Ctrl-C)')
            return
        finally:
            try:
                link.mav.close()
            except Exception:
                pass


if __name__ == '__main__':
    main()
