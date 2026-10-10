#!/usr/bin/env python3
"""Bounded local-only SITL runs; deliberately drop source evidence via UDP proxy.

Run under Ubuntu/ROS dependencies. Never connects to vehicle network addresses.
Each run retains its own logs and stops its owned process group on completion.
"""
import argparse
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import threading
import time
import urllib.request
import uuid


class EvidenceProxy:
    def __init__(self, mode):
        from pymavlink import mavutil
        self.parser = mavutil.mavlink.MAVLink(None)
        self.parser.robust_parsing = True
        self.mode, self.dropped, self.error = mode, 0, None
        self.switch_seen = False
        self.commands = mavutil.mavlink.MAVLink(None)
        self.front = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.front.bind(('127.0.0.1', 14560))
        self.back = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.back.bind(('127.0.0.1', 0))
        self.peer = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        try:
            while not self.stop.is_set():
                for sock in select.select([self.front, self.back], [], [], .1)[0]:
                    data, address = sock.recvfrom(65535)
                    if sock is self.front:
                        self.peer = address
                        for msg in self.commands.parse_buffer(data) or []:
                            if msg.get_type() == 'COMMAND_LONG' and msg.command == 42007:
                                self.switch_seen = True
                        self.back.sendto(data, ('127.0.0.1', 14557))
                    elif self.peer:
                        for msg in self.parser.parse_buffer(data) or []:
                            drop = (self.mode == 'drop-ack' and msg.get_type() == 'COMMAND_ACK'
                                    and msg.command == 42007)
                            drop |= (self.mode == 'drop-source' and self.switch_seen
                                     and msg.get_type() == 'NAMED_VALUE_FLOAT'
                                     and msg.name.rstrip('\x00') == 'NAV_SRC')
                            if drop:
                                self.dropped += 1
                            else:
                                self.front.sendto(msg.get_msgbuf(), self.peer)
        except Exception as error:
            self.error = repr(error)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        self.front.close()
        self.back.close()


def run(mode, timeout):
    root = Path(__file__).resolve().parents[1]
    folder = root / 'sitl/logs' / ('switch-'+mode+'-'+uuid.uuid4().hex[:10])
    folder.mkdir(parents=True)
    env = dict(os.environ, SITL_LOG_DIR=str(folder), SITL_RUN_DIR=str(folder/'runtime'))
    env.pop('QGC_HOST', None)
    env['SUPERVISOR_MAVLINK'] = 'udpout:127.0.0.1:14560'
    # No physical network endpoint, joystick output or vehicle process is used.
    proxy = EvidenceProxy(mode)
    result = dict(mode=mode, passed=False, evidence_directory=str(folder))
    proc = None
    try:
        with (folder/'runner.log').open('w') as output:
            proc = subprocess.Popen(['bash', str(root/'sitl/run.sh')], cwd=root,
                                    env=env, stdout=output, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                if proxy.error:
                    result['error'] = proxy.error
                    break
                if proc.poll() is not None:
                    result['error'] = 'Runner exited before validation: '+str(proc.returncode)
                    break
                path = folder/'scenario-result.json'
                if mode == 'nominal' and path.exists():
                    try:
                        scenario = json.loads(path.read_text())
                    except json.JSONDecodeError:
                        continue
                    result.update(passed=scenario.get('passed') is True, scenario=scenario)
                    break
                try:
                    with urllib.request.urlopen('http://127.0.0.1:8090/api/navigation', timeout=1) as response:
                        streams = json.load(response)['streams']
                        status = streams['supervisor']['data']
                    trace = status.get('source_transaction') or {}
                    if mode != 'nominal' and status.get('state') == 'Fault' and trace:
                        expected = 'accepted command ACK' if mode == 'drop-ack' else 'source feedback'
                        unexpected = 'source feedback' if mode == 'drop-ack' else 'accepted command ACK'
                        result.update(passed=(expected in status.get('reason', '')
                            and unexpected not in status.get('reason', '')
                            and status.get('navigation_ready') is False and proxy.dropped > 0
                            and streams['heartbeat']['data']['armed'] is False),
                            supervisor=status)
                        break
                except (OSError, KeyError, ValueError):
                    pass
                time.sleep(.2)
            else:
                result['error'] = 'Validation deadline exceeded'
    finally:
        if proc is not None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        proxy.close()
        result['dropped_messages'] = proxy.dropped
        if proxy.error:
            result.update(passed=False, proxy_error=proxy.error)
        (folder/'validation.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result['passed']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['nominal', 'drop-ack', 'drop-source'], default='nominal')
    parser.add_argument('--timeout', type=float, default=150)
    args = parser.parse_args()
    raise SystemExit(0 if run(args.mode, args.timeout) else 1)
