#!/usr/bin/env python3
"""12-second passive ROS/GUI snapshot. Never sends vehicle or ROS commands."""
import argparse
import json
import statistics
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path


def assess(bridge, supervisor, controller):
    checks = {
        'vehicle_telemetry': controller.get('vehicle_fresh') is True,
        'disarmed': controller.get('armed') is False,
        'kill_and_esc_off': controller.get('sa_active') is True and controller.get('esc_commanded_on') is False,
        'continuous_bridge': bridge.get('fusion_policy') == 'bridge_odometry',
        'bridge_healthy': bridge.get('healthy') is True,
        'dvl_valid': (bridge.get('inputs') or {}).get('dvl') is True,
        'vn_fresh': (bridge.get('inputs') or {}).get('vn') is True,
        'surface': bridge.get('surfaced') is True,
        'depth_near_zero': isinstance(bridge.get('depth_m'), (int, float)) and abs(bridge['depth_m']) <= .1,
        'gps_qualified': (bridge.get('gps_admission') or {}).get('qualified') is True,
        'cube_profile': bridge.get('configuration_verified') is True,
        'odometry_transmitting': bridge.get('transmitting') is True,
        'heading_commissioned': bridge.get('heading_reference_validated') is True,
        'supervisor_active': supervisor.get('mode') == 'Active',
    }
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/root/robotx_ws/logs/waypoint-preflight.json')
    args = parser.parse_args()
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('waypoint_preflight_readonly')
    rows = []
    node.create_subscription(String, '/graey/navigation/bridge_status',
                             lambda m: rows.append((time.monotonic(), json.loads(m.data))), 20)
    end = time.monotonic()+12
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    def get(path):
        try:
            return json.load(urllib.request.urlopen('http://localhost:8090'+path, timeout=3))
        except Exception as error:
            return {'error': str(error)}
    controller = get('/api/controller')
    stream = get('/api/navigation/supervisor').get('streams', {}).get('supervisor', {})
    supervisor = stream.get('data', {}) if stream.get('fresh') else {}
    bridge = rows[-1][1] if rows and time.monotonic()-rows[-1][0] < .5 else {}
    pressure = {r['pressure_sample']['cube_boot_ms']: r['pressure_sample']['hpa']
                for _, r in rows if r.get('pressure_sample') and
                0 <= r['published_monotonic']-r['pressure_sample']['received'] < .5}
    checks = assess(bridge, supervisor, controller)
    result = dict(captured_utc=datetime.now(timezone.utc).isoformat(), checks=checks,
                  blocked=[k for k, v in checks.items() if not v], bridge=bridge,
                  supervisor=supervisor, controller=controller,
                  bridge_instances=sorted({r.get('instance', '') for _, r in rows}),
                  pressure_samples=len(pressure),
                  observed_pressure_median_hpa=statistics.median(pressure.values()) if pressure else None,
                  note='Snapshot only; does not validate true heading, map accuracy, fusion or mission permission.')
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(blocked=result['blocked'], depth_m=bridge.get('depth_m'),
                         yaw_deg=(bridge.get('heading_vn') or {}).get('yaw'),
                         profile_mismatches=bridge.get('profile_mismatches'), evidence=str(path))))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
