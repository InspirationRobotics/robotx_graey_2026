"""Offline MAVLink audit and production GUI-cache replay (requires pymavlink).

Usage: python tools/audit_navigation_tlog.py session.tlog --output report.json
No vehicle connection or outgoing MAVLink messages. Output omits coordinates.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pymavlink import mavutil
from robotx_graey_2026.api.gui.navigation_view import NavigationView


def audit(path):
    now = 0.
    view = NavigationView(lambda: now)
    link = mavutil.mavlink_connection(str(path))
    counts, flags_at_jump, marker_at_jump = Counter(), Counter(), Counter()
    params = defaultdict(set)
    previous = {}
    ekf_flags = None
    max_jump = defaultdict(float)
    gps_accuracy_rejected = gps_no_fix = source_commands = source_reports = 0
    first = last = None
    odom_first = odom_last = None
    try:
        while True:
            msg = link.recv_match()
            if msg is None:
                break
            now = msg._timestamp
            if not math.isfinite(now):
                continue
            first = now if first is None else first
            last = now
            kind = msg.get_type()
            counts[kind] += 1
            if kind == 'ODOMETRY':
                odom_first = now if odom_first is None else odom_first
                odom_last = now
            if kind == 'COMMAND_LONG' and msg.command == 42007:
                source_commands += 1
            if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
                continue
            if kind == 'PARAM_VALUE' and str(msg.param_id).startswith(('EK3_', 'GPS_', 'VISO_')):
                params[str(msg.param_id)].add(msg.param_value)
            if kind == 'NAMED_VALUE_FLOAT' and msg.name.rstrip('\x00') == 'NAV_SRC':
                source_reports += 1
            if kind == 'EKF_STATUS_REPORT':
                ekf_flags = msg.flags
            if kind == 'GPS_RAW_INT':
                gps_no_fix += msg.fix_type < 3
                gps_accuracy_rejected += msg.fix_type >= 3 and not 0 < getattr(msg, 'h_acc', 0) <= 3000
            view.consume(kind, msg)
            if kind in ('GPS_RAW_INT', 'GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED'):
                old = previous.get(kind)
                previous[kind] = (now, msg)
                if old is None or not 0 < now-old[0] < 2:
                    continue
                a = old[1]
                if kind == 'LOCAL_POSITION_NED':
                    distance = math.hypot(msg.x-a.x, msg.y-a.y)
                else:
                    distance = 111320/1e7*math.hypot(msg.lat-a.lat,
                        (msg.lon-a.lon)*math.cos(math.radians(a.lat/1e7)))
                max_jump[kind] = max(max_jump[kind], distance)
                if distance > 1:
                    counts[kind+'_jumps_over_1m'] += 1
                    if kind == 'GLOBAL_POSITION_INT':
                        flags_at_jump[str(ekf_flags)] += 1
                        marker_at_jump[str(view.samples['global']['data']['valid'])] += 1
    finally:
        link.close()
    with open(path, 'rb') as source:
        hasher = hashlib.sha256()
        for chunk in iter(lambda: source.read(1024*1024), b''):
            hasher.update(chunk)
        digest = hasher.hexdigest()
    return dict(sha256=digest, first_timestamp=first, last_timestamp=last,
        counts=dict(counts), maximum_consecutive_step_m=dict(max_jump),
        global_jump_ekf_flags=dict(flags_at_jump), replay_global_marker_at_jump=dict(marker_at_jump),
        gps_no_3d_fix=gps_no_fix, gps_3d_accuracy_rejected=gps_accuracy_rejected,
        source_commands=source_commands, source_reports=source_reports,
        recorded_parameters={k: sorted(v) for k, v in params.items()},
        odometry_first=odom_first, odometry_last=odom_last,
        limitations='Telemetry gaps are not proof of sensor failure. Replay tests GUI admission, not EKF fusion. No DVL stationarity or innovation tuning is inferred.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps(audit(args.log), indent=2)+'\n')
