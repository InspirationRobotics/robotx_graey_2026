#!/usr/bin/env python3
"""Owned native SITL + deterministic sensor fixture; never contacts GRAEY.

This verifies external-only aiding through pressure/GPS transitions, not pool
hydrodynamics. DVL/VN/depth fixture inputs are independent of Cube estimates.
QGC forwarding is absent. Only the owned sysid-42 simulation can be armed.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
from robotx_graey_2026.api.navigation.bridge_estimator import BridgeEstimator
from robotx_graey_2026.api.navigation.odometry_profile import PROFILE


def run(binary):
    for port in (5864, 5762, 5763):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', port))
    root = Path(__file__).resolve().parent/'logs'
    root.mkdir(exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix='bridge-', dir=root))
    binary = Path(binary).resolve()
    params = dict(PROFILE, SYSID_THISMAV=42, EK2_ENABLE=0, LOG_DISARMED=1, SCR_ENABLE=0)
    (folder/'defaults.parm').write_text('\n'.join(f'{k} {v}' for k, v in params.items()))
    result = dict(evidence=str(folder), binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                  passed=False, phases=[], origins=[], fixture='independent stationary DVL/VN; prescribed pressure depth')
    m = None
    with (folder/'ardusub.log').open('w') as log, (folder/'gps-decisions.jsonl').open('w') as decisions:
        proc = subprocess.Popen([str(binary), '-w', '--model', 'vectored', '--sysid', '42',
            '--home', '32.924,-117.0385,0,0', '--defaults', str(folder/'defaults.parm'),
            '--serial0', 'tcp:5864'], cwd=folder, stdout=log, stderr=subprocess.STDOUT)
        try:
            time.sleep(3)
            m = mavutil.mavlink_connection('tcp:127.0.0.1:5864', source_system=255, source_component=197)
            hb = m.wait_heartbeat(timeout=10)
            assert hb and hb.get_srcSystem() == 42 and not hb.base_mode & 128
            for mid in (24, 32, 33, 49, 193):
                m.mav.command_long_send(42, 1, 511, 0, mid, 100000, 0, 0, 0, 0, 0)
            for name in PROFILE:
                m.mav.param_request_read_send(42, 1, name.encode(), -1)
            m.mav.set_gps_global_origin_send(42, 329240000, -1170385000, 0)
            est = BridgeEstimator(heading_validated=True)
            readback, latest, times = {}, {}, {}
            next_sync = 0.
            for phase, duration, depth in (('surface', 35, 0.), ('submerged_live_gps', 10, .5),
                                          ('submerged_stale_gps', 15, .5), ('surface_recovery', 15, 0.)):
                if phase == 'submerged_live_gps':
                    m.mav.command_long_send(42, 1, 176, 0, 1, 19, 0, 0, 0, 0, 0)
                    m.mav.command_long_send(42, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
                    m.mav.param_set_send(42, 1, b'SIM_GPS_GLITCH_X', .00045, 9)
                if phase == 'submerged_stale_gps':
                    m.mav.param_set_send(42, 1, b'SIM_GPS_DISABLE', 1, 9)
                if phase == 'surface_recovery':
                    m.mav.param_set_send(42, 1, b'SIM_GPS_GLITCH_X', 0, 9)
                    m.mav.param_set_send(42, 1, b'SIM_GPS_DISABLE', 0, 9)
                start = time.monotonic()
                accepted = emitted = 0
                max_xy = 0.
                while time.monotonic()-start < duration:
                    now = time.monotonic()
                    est.imu(now, (1., 0., 0., 0.), (0., 0., 0.))
                    est.pressure_depth(now, depth)
                    est.dvl(now, (0., 0., 0.), True)
                    if now >= next_sync:
                        m.mav.heartbeat_send(6, 8, 0, 0, 0)
                        m.mav.timesync_send(0, est.gps.clock.request(time.monotonic_ns()))
                        for name in PROFILE:
                            m.mav.param_request_read_send(42, 1, name.encode(), -1)
                        next_sync = now+1
                    while True:
                        msg = m.recv_match(blocking=False)
                        if msg is None:
                            break
                        if (msg.get_srcSystem(), msg.get_srcComponent()) != (42, 1):
                            continue
                        kind = msg.get_type()
                        if kind == 'PARAM_VALUE':
                            readback[msg.param_id.rstrip('\x00')] = msg.param_value
                        elif kind == 'GPS_GLOBAL_ORIGIN':
                            origin = (msg.latitude/1e7, msg.longitude/1e7, msg.altitude/1000)
                            if not result['origins'] or result['origins'][-1] != origin:
                                result['origins'].append(origin)
                            est.set_origin((msg.latitude/1e7, msg.longitude/1e7, msg.altitude/1000))
                        elif kind == 'TIMESYNC' and msg.tc1:
                            est.gps.clock.reply(msg.ts1, msg.tc1, time.monotonic_ns())
                        elif kind == 'GPS_RAW_INT':
                            fix = dict(cube_fix_time_us=msg.time_usec, received=time.monotonic(),
                                lat=msg.lat/1e7, lon=msg.lon/1e7, alt=msg.alt/1000,
                                accuracy=msg.h_acc/1000, fix=msg.fix_type,
                                speed_mps=msg.vel/100, course_deg=msg.cog/100,
                                speed_accuracy_mps=msg.vel_acc/1000)
                            accepted += est.observe_gps(time.monotonic(), fix)
                            decisions.write(json.dumps(dict(phase=phase, **est.event))+'\n')
                        if kind in ('HEARTBEAT', 'LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'EKF_STATUS_REPORT'):
                            latest[kind], times[kind] = msg.to_dict(), time.monotonic()
                        if kind == 'LOCAL_POSITION_NED':
                            max_xy = max(max_xy, math.hypot(msg.x, msg.y))
                    status = est.status(time.monotonic())
                    if status['healthy'] and all(readback.get(k) == v for k, v in PROFILE.items()):
                        m.mav.odometry_send(int(time.monotonic()*1e6), 20, 12,
                            *est.position, est.orientation(), *est.velocity(), *est.rates,
                            est.covariance(), [float('nan')]+[0.]*20, 0, 3, 80)
                        emitted += 1
                    time.sleep(.045)
                row = dict(phase=phase, accepted=accepted, emitted=emitted, max_horizontal_m=max_xy,
                           latest=dict(latest), status=est.status(time.monotonic()),
                           ages={k: time.monotonic()-v for k, v in times.items()})
                result['phases'].append(row)
                print(json.dumps(row), flush=True)
            result['profile_verified'] = all(readback.get(k) == v for k, v in PROFILE.items())
            result['profile_mismatches'] = {k: {'expected': v, 'actual': readback.get(k)}
                                          for k, v in PROFILE.items() if readback.get(k) != v}
            result['passed'] = bool(result['profile_verified']
                and result['phases'][0]['accepted'] > 10 and result['phases'][-1]['accepted'] > 10
                and all(r['accepted'] == 0 for r in result['phases'][1:3])
                and all(r['emitted'] > 50 and r['max_horizontal_m'] < 1.
                        and r['ages'].get('LOCAL_POSITION_NED', 99) < .5
                        and r['latest'].get('EKF_STATUS_REPORT', {}).get('flags', 0) & (8 | 16)
                        for r in result['phases'])
                and all(r['latest'].get('HEARTBEAT', {}).get('base_mode', 0) & 128
                        for r in result['phases'][1:]))
        except Exception as error:
            result['error'] = repr(error)
        finally:
            if m:
                m.close()
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            (folder/'result.json').write_text(json.dumps(result, indent=2))
            print(json.dumps(result), flush=True)
    return result['passed']


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--binary', required=True)
    raise SystemExit(0 if run(p.parse_args().binary) else 1)
