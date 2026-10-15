#!/usr/bin/env python3
"""Watch readiness; optionally request checked, session-only mission permission.

Never arms, selects a flight mode, starts a mission or clears a fault.
"""
import argparse
import json
import math
import time
import urllib.request


def assess(nav):
    s = nav.get('streams', {}).get('supervisor', {})
    if not s.get('fresh') or not 0 <= s.get('age', math.inf) < .5:
        return False, 'Supervisor telemetry stale or unavailable'
    state = s.get('data', {})
    a = state.get('north_alignment') or {}
    if not a.get('aligned'):
        return False, a.get('reason', 'North alignment unavailable')
    cube = nav.get('cube_estimate') or {}
    if not cube.get('valid'):
        return False, 'North aligned; waiting for healthy Cube position and heading'
    attitude = nav.get('streams', {}).get('attitude', {})
    # The mission supervisor checks VN/Cube yaw consistency, uncertainty,
    # sensor ages and intent independently. This watcher is not an approval.
    if not attitude.get('fresh'):
        return False, 'Cube attitude stale'
    if state.get('surfaced') is not True or state.get('state') == 'Fault':
        return False, state.get('reason', 'Confirmed healthy surface state required')
    if not state.get('configuration_verified') or state.get('mode') != 'Active':
        return False, 'North aligned; Cube available; mission commissioning remains disabled'
    heading = state.get('dive_heading_check') or {}
    if not heading.get('allowed'):
        return False, heading.get('reason', 'Pre-dive heading check pending')
    return True, 'Preparation checks passed; preview target and verify Abort before Start'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', default='http://localhost:8090')
    p.add_argument('--watch', action='store_true')
    p.add_argument('--commission', action='store_true', help='Request checked supervisor permission while killed/disarmed')
    args = p.parse_args()
    if args.commission:
        import rclpy
        from std_srvs.srv import Trigger
        rclpy.init()
        node = rclpy.create_node('commission_navigation_client')
        try:
            client = node.create_client(Trigger, '/graey/navigation/commission')
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError('Commissioning service unavailable')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            if not future.done() or future.exception():
                raise RuntimeError('Commissioning response unavailable; inspect current state before retrying')
            response = future.result()
            print(response.message, flush=True)
            if not response.success:
                return 2
        finally:
            node.destroy_node()
            rclpy.shutdown()
    previous = None
    while True:
        try:
            with urllib.request.urlopen(args.url.rstrip('/')+'/api/navigation', timeout=3) as response:
                ready, reason = assess(json.load(response))
        except (OSError, ValueError) as error:
            ready, reason = False, 'Navigation unavailable: '+str(error)
        if reason != previous:
            print(('READY: ' if ready else 'WAIT: ')+reason, flush=True)
            previous = reason
        if ready or not args.watch:
            return 0 if ready else 2
        time.sleep(1)


if __name__ == '__main__':
    raise SystemExit(main())
