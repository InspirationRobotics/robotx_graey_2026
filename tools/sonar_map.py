#!/usr/bin/env python3
"""Breadcrumb map, manual mode -> http://<jetson-ip>:8095/map   (Ctrl-C to stop)

The VN-100 and DVL readings live inside the ROS container, so this starts
tools/pose_relay.py in there to pass them out, and it stops with this:

    PYTHONPATH=. python3 tools/sonar_map.py --port 8095 \
        --device /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D2010VWF-if00-port0

The sonar sweeps nonstop. The Radar tab is the usual picture. The Map tab puts
every echo on a top-down map using the VN-100 and the DVL (pose.py) - never
the Cube. Press Start: where the sub is then becomes (0, 0), y forward along
its heading then, x to the right. Pause freezes the map but keeps tracking the
sub; Reset clears it and makes a new (0, 0).

This only watches. It sends nothing to the sub: hold depth in QGC and drive.

Range (2-20 m), sector and threshold are set on the Map tab. A new range or
sector, or Start/Reset, drops the sweep in progress and starts a fresh one at
once. A new threshold applies from the next ping. Pings taken while the sub
turns faster than --max-turn are ignored (crumbs.py says why).

Every sweep, finished or cut short, is saved with the pose at each ping, so a
run can be replayed on the Mac. Record (Map tab) also keeps every VN-100/DVL
reading, the settings and your notes in a folder of its own (recorder.py).
"""
import argparse
import json
import math
import atexit
import os
import subprocess
import sys
import time

import numpy as np

from robotx_graey_2026.api.sonar import settings as S
from robotx_graey_2026.api.sonar import webview
from robotx_graey_2026.api.sonar.outlines import Outliner
from robotx_graey_2026.api.sonar.recorder import Recorder
from robotx_graey_2026.api.sonar.crumbs import FLOOR_CUT_M, MAX_TURN_DPS, THRESHOLD, CrumbMap
from robotx_graey_2026.api.sonar.detect import perceive
from robotx_graey_2026.api.sonar.pose import RELAY_PORT, UdpPose
from robotx_graey_2026.api.sonar.sweep import Sonar
from robotx_graey_2026.api.sonar.viewer import SIZE, Radar, render

RANGE_LO, RANGE_HI = 2.0, 20.0
SNAPSHOT_S = 0.3            # how often the Map tab's data is rebuilt
RADAR_S = 0.25              # and the Radar tab's picture. Both happen between
                            # pings, so doing them every ping would slow the sweep.
TRACK_STEP_M = 0.02         # a track point when the sub has moved this far


# Runs pose_relay.py inside the ROS container, the way graey-ros.service runs
# the core launch. -i keeps its input open; it quits when that closes.
RELAY_CMD = ["docker", "exec", "-i", "graey", "bash", "-c",
             "source /opt/ros/humble/setup.bash && exec python3 "
             "/root/robotx_ws/src/robotx_graey_2026/tools/pose_relay.py {port}"]


def start_relay(port):
    """Start the relay, or warn and carry on - the Map tab will say there is no
    data, and the radar still works."""
    try:
        proc = subprocess.Popen([c.format(port=port) for c in RELAY_CMD],
                                stdin=subprocess.PIPE)
    except OSError as exc:
        print(f"[WARN] couldn't start pose_relay.py: {exc}")
        return None
    time.sleep(1.0)
    if proc.poll() is not None:
        print(f"[WARN] pose_relay.py stopped at once (exit {proc.returncode}) - see above")
        return None
    atexit.register(proc.terminate)
    return proc


class _Restart(Exception):
    """Raised inside a sweep to drop it: new range or sector, or Start/Reset."""


def make_pose(port):
    """The VN-100 + DVL dead reckoner. A function so a test can swap it."""
    return UdpPose(port)


def sector_arc(start, end):
    """Counterclockwise from start to end, unwrapped the way Sonar.sweep wants.
    Equal means the full circle."""
    start, end = start % 360.0, end % 360.0
    if end <= start:
        end += 360.0
    return start, end


def reach(start, end, range_m):
    """How far the sector reaches left and right of the sonar, for drawing."""
    c = np.cos(np.radians(np.arange(start, end + 1e-9, 1.0)))
    return round(range_m * max(0.0, -c.min()), 2), round(range_m * max(0.0, c.max()), 2)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", help="serial by-id path, if not using pingproxy")
    p.add_argument("--udp", help="host:port of pingproxy, e.g. 127.0.0.1:9092")
    p.add_argument("--port", type=int, default=8095, help="web page port")
    p.add_argument("--pose-port", type=int, default=RELAY_PORT,
                   help="UDP port pose_relay.py sends to")
    p.add_argument("--no-relay", action="store_true",
                   help="don't start pose_relay.py (it is running by hand)")
    p.add_argument("--range", type=float, default=4.0, help="metres, 2-20")
    p.add_argument("--start", type=float, default=180.0, help="sector start, degrees")
    p.add_argument("--end", type=float, default=0.0,
                   help="sector end, degrees, counterclockwise from start "
                        "(default 180 -> 0 = the bottom half)")
    p.add_argument("--step", type=float, default=2.0, help="degrees between pings")
    p.add_argument("--threshold", type=int, default=THRESHOLD, help="crumb threshold, 1-255")
    p.add_argument("--floor-cut", type=float, default=FLOOR_CUT_M,
                   help="m: no crumbs from echoes this close above the floor (DVL altitude); 0 = off")
    p.add_argument("--max-turn", type=float, default=MAX_TURN_DPS,
                   help="deg/s; pings while turning faster than this are ignored")
    p.add_argument("--down-gradian", type=int, default=S.DOWN_GRADIAN)
    p.add_argument("--save-dir", default=None,
                   help="where sweeps are saved (default sonar_data/map_<date_time>)")
    p.add_argument("--no-save", action="store_true")
    p.add_argument("--size", type=int, default=SIZE, help="radar picture size, px")
    args = p.parse_args()
    if not RANGE_LO <= args.range <= RANGE_HI:
        sys.exit(f"--range must be {RANGE_LO:g}-{RANGE_HI:g} m")

    save_dir = None
    if not args.no_save:
        save_dir = args.save_dir or os.path.join(
            "sonar_data", time.strftime("map_%Y%m%d_%H%M%S"))
        os.makedirs(save_dir, exist_ok=True)
        print(f"[INFO] saving sweeps to {save_dir}")

    try:
        pose = make_pose(args.pose_port)
    except OSError as exc:
        sys.exit(f"can't listen for pose_relay.py on UDP {args.pose_port}: {exc}")
    relay = None if args.no_relay else start_relay(args.pose_port)

    udp = None
    if args.udp:
        host, port = args.udp.split(":")
        udp = (host, int(port))
    sonar = Sonar(device=args.device, udp=udp, down_gradian=args.down_gradian)

    webview.serve(args.port)
    webview.enable_map()
    webview.set_target_editable(False)
    webview.set_note("map tool - see the Map tab")
    webview.set_range_limits(RANGE_LO, RANGE_HI)
    webview.set_range(args.range)
    webview.set_range_editable(True)
    webview.set_sector(args.start, args.end)
    webview.set_threshold(args.threshold)
    webview.set_floor_cut(args.floor_cut)
    print(f"[INFO] map: http://<this-computer>:{args.port}/map")

    cmap = CrumbMap(threshold=args.threshold, max_turn_dps=args.max_turn,
                    floor_cut_m=args.floor_cut)
    rec = Recorder(os.path.dirname(save_dir) if save_dir else "sonar_data")
    atexit.register(lambda: rec.stop(stopped_by="map tool closed") if rec.active else None)
    seen = {"settings": None}

    def settings_now():
        return {"range_m": webview.range_m(), "sector": list(webview.sector()),
                "step_deg": args.step, "threshold": webview.threshold(),
                "floor_cut_m": webview.floor_cut(), "max_turn_dps": args.max_turn}

    def recording():
        """Record / Stop / Mark presses, and settings changes into the marks."""
        for cmd, note in webview.record_commands():
            if cmd == "start":
                meta = {"settings": settings_now(), "ping360": dict(sonar.settings),
                        "mount": {"down_gradian": args.down_gradian, "sonar_fwd_m": S.SONAR_FWD_M,
                                  "sonar_right_m": S.SONAR_RIGHT_M, "min_range_m": S.MIN_RANGE_M,
                                  "beam_deg": [S.BEAM_IN_PLANE_DEG, S.BEAM_FORE_AFT_DEG]},
                        "map_state": run["state"], "device": args.device or args.udp}
                print(f"[INFO] recording to {rec.start(note, meta)}")
                pose.log = rec.nav
                seen["settings"] = None
            elif cmd == "stop":
                pose.log = None
                print(f"[INFO] recording saved: {rec.stop(ping360_at_end=dict(sonar.settings))}")
            elif cmd == "mark":
                rec.mark(note or "(mark)")
        if rec.active and settings_now() != seen["settings"]:
            seen["settings"] = settings_now()
            rec.mark("settings: " + json.dumps(seen["settings"]), auto=True)
    radar = Radar()
    outliner = Outliner()
    track = []
    run = {"state": "waiting", "epoch": 0, "sweeps": 0}
    cur = {}                    # the sweep in progress
    last = {"snap": 0.0, "radar": 0.0}

    def new_origin():
        pose.reset()
        cmap.clear()
        outliner.clear()
        track.clear()
        run["epoch"] += 1

    def buttons():
        """Act on presses since the last ping. Start and Reset restart the sweep,
        so the first sweep on a new map is a whole one."""
        restart = False
        for cmd in webview.map_commands():
            state = run["state"]
            if cmd == "start" and state == "waiting":
                new_origin()
                run["state"] = "running"
                restart = True
            elif cmd == "pause" and state == "running":
                run["state"] = "paused"
            elif cmd == "resume" and state == "paused":
                run["state"] = "running"
            elif cmd == "reset":
                new_origin()
                restart = True
            print(f"[INFO] {cmd} -> {run['state']}")
            rec.mark(f"map {cmd} -> {run['state']}", auto=True)
        return restart

    def take(sweep):
        """Pings of this sweep not handled yet: look up the pose, make crumbs."""
        for i in range(cur["done"], len(sweep.angles_deg)):
            t = sweep.ping_times[i] if sweep.ping_times else None
            here = pose.pose_at(t) if t is not None else None
            turn = pose.turn_rate(t) if t is not None else None
            cur["poses"].append(here)
            row = sweep.image[i]
            # a ping that got no reply is all zeros: it saw nothing because it
            # heard nothing, so it must not fade anything
            use = run["state"] == "running" and row.max() > 0
            cur["used"].append(use)             # saved, so a replay uses exactly these
            if use:
                cmap.add_ping(sweep.angles_deg[i], row, sweep.metres_per_bin, here,
                              sweep.step_deg, cur["range"], turn)
        cur["done"] = len(sweep.angles_deg)
        cur["sweep"] = sweep

    def snapshot():
        now = pose.now()
        if now is not None and (not track or math.hypot(now.x - track[-1][0],
                                                        now.y - track[-1][1]) >= TRACK_STEP_M):
            track.append((now.x, now.y))
        c = cmap.crumbs()
        crumbs = np.column_stack([np.round(c["x"], 2), np.round(c["y"], 2),
                                  np.round(c["brightness"]), c["seen"], np.round(c["up"], 2)]).tolist()
        webview.set_map({
            "state": run["state"],
            "dvl": "no data" if now is None else ("ok" if now.valid else "lost"),
            "pose": None if now is None else [round(now.x, 3), round(now.y, 3),
                                              round(now.heading_deg, 1)],
            "sonar": None if now is None else [round(v, 3) for v in cmap.sonar_xy(now)],
            "track": [[round(x, 2), round(y, 2)] for x, y in track[-5000:]],
            "crumbs": crumbs, "sweep": run["sweeps"], "count": len(crumbs),
            "outlines": outliner.update(c, time.monotonic()),
            "range": cur["range"], "sector": list(cur["sector"]),
            "threshold": cmap.threshold, "floorCut": cmap.floor_cut_m,
            "turning": cmap.turning and run["state"] == "running",
            "skippedTurning": cmap.skipped_turning,
            "slice": reach(*sector_arc(*cur["sector"]), cur["range"]),
            "recording": rec.status()})

    def show(sweep, force=False):
        t = time.monotonic()
        if force or t - last["snap"] >= SNAPSHOT_S:
            snapshot()
            last["snap"] = t
        if force or t - last["radar"] >= RADAR_S:
            radar.update(sweep)
            per = perceive(sweep, target=None, tuning={"threshold": S.DETECT["threshold"]},
                           floor=None, require_floor=False)
            webview.publish(render(per, None, radar=radar, size=args.size,
                                   state=f"MAP {run['state'].upper()}  range={cur['range']:g}m"))
            last["radar"] = t

    def live(partial):
        recording()
        restart = buttons()
        if (restart or webview.range_m() != cur["range"]
                or webview.sector() != cur["sector"]):
            raise _Restart
        cmap.threshold = webview.threshold()
        cmap.floor_cut_m = webview.floor_cut()
        take(partial)
        show(partial)

    def save(aborted):
        sweep = cur.get("sweep")
        also = rec.sweep_path(run["sweeps"]) if sweep is not None else None
        if (save_dir is None and also is None) or sweep is None:
            return
        poses = np.array([[q.x, q.y, q.heading_deg, q.valid] if q else [np.nan] * 4
                          for q in cur["poses"]], float)
        data = dict(
            image=sweep.image, angles_deg=np.array(sweep.angles_deg),
            ping_times=np.array(sweep.ping_times or [], float),
            metres_per_bin=sweep.metres_per_bin, pose_x_y_heading_valid=poses,
            pose_roll_pitch=np.array([[q.roll_deg, q.pitch_deg] if q else [np.nan] * 2
                                      for q in cur["poses"]], float),
            pose_alt=np.array([q.alt_m if q else -1.0 for q in cur["poses"]], float),
            floor_cut_m=cmap.floor_cut_m,
            range_m=cur["range"], sector=np.array(cur["sector"]),
            threshold=cmap.threshold, state=run["state"], map_epoch=cur["epoch"],
            used=np.array(cur["used"], bool),
            sonar_fwd_m=cmap.sonar_fwd_m, sonar_right_m=cmap.sonar_right_m,
            ping360=json.dumps(sonar.settings), step_deg=args.step,
            aborted=aborted, wall_time=time.time())
        for path in (os.path.join(save_dir, f"sweep_{run['sweeps']:05d}.npz") if save_dir else None,
                     also):
            if path:
                np.savez_compressed(path, **data)

    while True:
        recording()
        buttons()                        # presses between sweeps: a new one starts anyway
        cur.clear()
        # epoch as the sweep began: Start/Reset end a sweep, so all its pings
        # belong to the map that was current when it started
        cur.update(done=0, poses=[], used=[], epoch=run["epoch"],
                   range=webview.range_m(), sector=webview.sector())
        cmap.threshold = webview.threshold()
        cmap.floor_cut_m = webview.floor_cut()
        cmap.new_sweep()
        run["sweeps"] += 1
        start, end = sector_arc(*cur["sector"])
        try:
            sweep = sonar.sweep(start, end, args.step, cur["range"], on_ping=live)
        except _Restart:
            save(aborted=True)
            continue
        take(sweep)
        save(aborted=False)
        show(sweep, force=True)


if __name__ == "__main__":
    main()
