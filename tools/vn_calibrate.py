#!/usr/bin/env python3
"""Guided, volatile VN HSI calibration. Run with core stopped, MAVProxy running.

This never writes flash, starts a mission, arms, or assigns a compass offset.
The operator must maintain hardware kill while handling the assembled vehicle.
"""
import argparse
import json
import math
from pathlib import Path
import time

from robotx_graey_2026.api.process_ownership import ProcessOwnership
from robotx_graey_2026.api.navigation.vn_protocol import transact, relative_heading


def gravity_face(ypr):
    _, pitch, roll = map(math.radians, ypr)
    down = (-math.sin(pitch), math.sin(roll)*math.cos(pitch), math.cos(roll)*math.cos(pitch))
    axis = max(range(3), key=lambda i: abs(down[i]))
    return axis*2+int(down[axis] < 0) if abs(down[axis]) > .85 else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9DQE-if00-port0')
    parser.add_argument('--seconds', type=int, default=120)
    parser.add_argument('--calibrate', action='store_true', help='Reset and collect onboard magnetic calibration')
    parser.add_argument('--killed-disarmed', action='store_true', help='Operator confirms hardware kill remains engaged')
    parser.add_argument('--output', default=str(Path.home()/'robotx_ws/logs'/f'vn-calibration-{time.time_ns()}.jsonl'))
    args = parser.parse_args()
    if args.calibrate and (not args.killed_disarmed or not 90 <= args.seconds <= 300):
        parser.error('Calibration needs --killed-disarmed and 90..300 seconds')
    import serial
    from pymavlink import mavutil
    # Same lock as the ROS reader: never steal its serial port or suspend it.
    with ProcessOwnership('vn100_node'):
        link = mavutil.mavlink_connection('udpout:127.0.0.1:14551', source_system=255, source_component=193)
        try:
            link.mav.heartbeat_send(6, 8, 0, 0, 0)
            heartbeat = link.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if not heartbeat or (heartbeat.get_srcSystem(), heartbeat.get_srcComponent()) != (1, 1) or heartbeat.base_mode & 128:
                raise RuntimeError('Fresh disarmed Cube heartbeat required')
            last_disarmed = time.monotonic()
            path = Path(args.output)
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('x', encoding='utf-8') as log, serial.Serial(args.port, 115200, timeout=.1, exclusive=True) as port:
                def record(event, data):
                    log.write(json.dumps(dict(monotonic=time.monotonic(), event=event, data=data), allow_nan=False)+'\n')
                    log.flush()
                before = {str(r): transact(port, f'VNRRG,{r:02d}') for r in (1, 4, 23, 26, 35, 44, 47)}
                record('before', before)
                print('Backup:', path, flush=True)
                if not args.calibrate:
                    print(json.dumps(before, indent=2))
                    return
                relative_heading(port)
                collecting = False
                try:
                    # No write-settings command: all changes remain volatile.
                    collecting = True
                    transact(port, 'VNWRG,44,2,3,1')
                    transact(port, 'VNWRG,44,1,3,1')
                    print('START: slowly rotate all axes; cover all six faces. Keep kill engaged.', flush=True)
                    faces = [0]*6
                    deadline = time.monotonic()+args.seconds
                    while time.monotonic() < deadline:
                        link.mav.heartbeat_send(6, 8, 0, 0, 0)
                        for _ in range(100):
                            h = link.recv_match(blocking=False)
                            if h is None:
                                break
                            if h.get_type() == 'HEARTBEAT' and (h.get_srcSystem(), h.get_srcComponent()) == (1, 1):
                                if h.base_mode & 128:
                                    raise RuntimeError('Cube armed; calibration stopped')
                                last_disarmed = time.monotonic()
                        if time.monotonic()-last_disarmed > 3:
                            raise RuntimeError('Cube heartbeat lost; calibration stopped')
                        ypr = list(map(float, transact(port, 'VNRRG,08')))
                        face = gravity_face(ypr)
                        if face is not None:
                            faces[face] += 1
                        record('sample', dict(ypr=ypr, faces=faces[:], calibration=transact(port, 'VNRRG,47')))
                        print(f'{max(0, deadline-time.monotonic()):.0f}s remaining; six-face samples {faces}', flush=True)
                        time.sleep(1)
                finally:
                    if collecting:
                        transact(port, 'VNWRG,44,0,3,1')
                after = {str(r): transact(port, f'VNRRG,{r:02d}') for r in (35, 44, 46, 47)}
                covered = min(faces) >= 5
                record('result', dict(after=after, six_faces_covered=covered, saved=False,
                    accepted=False, note='Coverage is not a heading-accuracy or fit-quality proof'))
                print('Learning stopped; relative mode; NOT saved. Six faces covered:', covered)
                print('Next: start a NEW navigation session, GPS/DVL north alignment, then verify Cube fusion.')
                if not covered:
                    raise RuntimeError('Insufficient six-face coverage; repeat calibration')
        finally:
            link.close()


if __name__ == '__main__':
    main()
