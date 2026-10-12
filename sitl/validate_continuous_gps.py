#!/usr/bin/env python3
"""Isolated native ArduSub GPS admission proof. No QGC forwarding.

Uses owned temporary state and TCP 5864, sysid 42. Does not contact a vehicle.
Tests transport/estimator behavior at rest, not underwater vehicle dynamics.
Optional arming is confined to the owned simulator; never a vehicle connection.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
from robotx_graey_2026.api.navigation.continuous_profile import PROFILE
from robotx_graey_2026.api.navigation.gps_admission import GPSAdmission, gps_input_args


def run(binary, armed_test=False):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 5864))
    evidence_root = Path(__file__).resolve().parent/'logs'
    evidence_root.mkdir(exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix='continuous-', dir=evidence_root))
    params = dict(PROFILE, SYSID_THISMAV=42, AHRS_EKF_TYPE=3, EK3_ENABLE=1,
                  EK2_ENABLE=0, VISO_TYPE=1, LOG_DISARMED=1, SCR_ENABLE=0)
    (folder/'defaults.parm').write_text('\n'.join(f'{k} {v}' for k, v in params.items()))
    result = dict(evidence=str(folder), phases=[], passed=False, armed_simulation=armed_test)
    m = None
    with (folder/'ardusub.log').open('w') as log:
        proc = subprocess.Popen([str(Path(binary).resolve()), '-w', '--model', 'vectored',
            '--sysid', '42', '--home', '32.924,-117.0385,0,0', '--defaults', str(folder/'defaults.parm'),
            '--serial0', 'tcp:5864'], cwd=folder, stdout=log, stderr=subprocess.STDOUT)
        try:
            time.sleep(3)
            m = mavutil.mavlink_connection('tcp:127.0.0.1:5864', source_system=255, source_component=200)
            h = m.wait_heartbeat(timeout=10)
            assert h and h.get_srcSystem() == 42 and not h.base_mode & 128
            for mid in (24, 124, 32, 33, 193):
                m.mav.command_long_send(42, 1, 511, 0, mid, 100000, 0, 0, 0, 0, 0)
            for name in PROFILE:
                m.mav.param_request_read_send(42, 1, name.encode(), -1)
            gate = GPSAdmission()
            readback = {}
            latest = {}
            forwarded = 0
            next_sync = 0.
            for phase, duration in (('acquire', 45), ('reject_jump', 20), ('recover', 20)):
                if phase == 'reject_jump' and armed_test:
                    m.mav.command_long_send(42, 1, 176, 0, 1, 19, 0, 0, 0, 0, 0)
                    m.mav.command_long_send(42, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
                    time.sleep(1)
                if phase != 'acquire':
                    m.mav.param_set_send(42, 1, b'SIM_GPS_GLITCH_X', .00045 if phase == 'reject_jump' else 0., 9)
                start = time.monotonic()
                before = forwarded
                while time.monotonic()-start < duration:
                    now = time.monotonic()
                    gate.quality.motion(now, (0., 0., 0.), True)
                    if now >= next_sync:
                        m.mav.heartbeat_send(6, 8, 0, 0, 0)
                        m.mav.timesync_send(0, gate.clock.request(time.monotonic_ns()))
                        next_sync = now+1
                    cov = [float('nan')]+[0.]*20
                    m.mav.odometry_send(int(now*1e6), 20, 12, 0, 0, 0,
                        [1., 0., 0., 0.], 0, 0, 0, 0, 0, 0, cov, cov, 0, 3)
                    while True:
                        msg = m.recv_match(blocking=False)
                        if msg is None:
                            break
                        if msg.get_srcSystem() != 42 or msg.get_srcComponent() != 1:
                            continue
                        kind = msg.get_type()
                        if kind == 'HEARTBEAT':
                            if not armed_test:
                                assert not msg.base_mode & 128, 'Unexpected armed state'
                            latest['HEARTBEAT'] = msg.to_dict()
                        if kind == 'PARAM_VALUE':
                            readback[msg.param_id.rstrip('\x00')] = msg.param_value
                        elif kind == 'TIMESYNC' and msg.tc1:
                            gate.clock.reply(msg.ts1, msg.tc1, time.monotonic_ns())
                        elif kind == 'GPS_RAW_INT':
                            fix = dict(cube_fix_time_us=msg.time_usec, lat=msg.lat/1e7,
                                       lon=msg.lon/1e7, accuracy=msg.h_acc/1000, fix=msg.fix_type)
                            if gate.check(time.monotonic(), fix, True) and all(readback.get(k) == v for k, v in PROFILE.items()):
                                m.mav.gps_input_send(*gps_input_args(msg))
                                forwarded += 1
                        if kind in ('GPS_RAW_INT', 'GPS2_RAW', 'GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED', 'EKF_STATUS_REPORT'):
                            latest[kind] = msg.to_dict()
                    time.sleep(.04)
                row = dict(phase=phase, forwarded=forwarded-before, status=gate.status(time.monotonic()), latest=dict(latest))
                result['phases'].append(row)
                print(json.dumps(row), flush=True)
            result['profile_verified'] = all(readback.get(k) == v for k, v in PROFILE.items())
            a, b, c = result['phases']
            result['passed'] = bool(result['profile_verified'] and a['forwarded'] > 10
                and b['forwarded'] <= 2 and c['forwarded'] > 10
                and b['latest'].get('GPS2_RAW', {}).get('fix_type', 9) < 3
                and 'LOCAL_POSITION_NED' in a['latest']
                and a['latest'].get('EKF_STATUS_REPORT', {}).get('flags', 0) & 8)
            result['admission_passed'] = result['passed']
            result['underwater_position_status_passed'] = bool(b['latest'].get('EKF_STATUS_REPORT', {}).get('flags', 0) & (8 | 16))
            result['armed_confirmed'] = bool(b['latest'].get('HEARTBEAT', {}).get('base_mode', 0) & 128)
            result['passed'] &= result['underwater_position_status_passed']
            if armed_test:
                result['passed'] &= result['armed_confirmed']
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
    p.add_argument('--armed-test', action='store_true', help='Arm only this owned sysid-42 simulator')
    args = p.parse_args()
    raise SystemExit(0 if run(args.binary, args.armed_test) else 1)
