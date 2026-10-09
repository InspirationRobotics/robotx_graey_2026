#!/usr/bin/env python3
"""Replay a saved breadcrumb-map run -> http://localhost:8095/map   (Ctrl-C to stop)

sonar_map.py saves every sweep with the sub's pose at each ping. This puts the
sweeps back through the same crumb maths (api/sonar/crumbs.py) and shows the
result on the same Map page, building up sweep by sweep - so settings can be
tried at a desk instead of in the pool. Runs on the Mac; no sonar needed.

    scp -r graey@graey.local:robotx_ws/src/robotx_graey_2026/sonar_data/map_<date_time> sonar_data/
    PYTHONPATH=. python3 tools/sonar_map_replay.py sonar_data/map_<date_time>

On the page: Pause/Resume the playback, Reset plays it again from the start,
and a new threshold replays the whole run with it. Range and sector are what
was recorded, so they are locked.

Exactly the pings the live map used are replayed, from the last Start or Reset
(--epoch picks another). Runs saved before Oct 5 don't record that per ping, so
there a whole sweep counts if the map was running when it ended. The turn rate
comes from the saved per-ping headings, so it is close to, not exactly, what
the live map used.
"""
import argparse
import glob
import math
import os
import sys
import time

import numpy as np

from robotx_graey_2026.api.sonar import webview
from robotx_graey_2026.api.sonar.outlines import Outliner
from robotx_graey_2026.api.sonar.crumbs import MAX_TURN_DPS, CrumbMap
from robotx_graey_2026.api.sonar.pose import Pose, wrap180
from sonar_map import SNAPSHOT_S, TRACK_STEP_M, reach, sector_arc

TURN_WINDOW_S = 0.3         # the same window DeadReckoner.turn_rate uses


def load(folder):
    """Every saved sweep the live map used at least one ping of."""
    sweeps = []
    for f in sorted(glob.glob(os.path.join(folder, "sweep_*.npz"))):
        with np.load(f) as z:
            s = {k: z[k] for k in z.files}
        if "used" not in s:                     # saved before per-ping flags
            s["used"] = np.full(len(s["angles_deg"]), str(s["state"]) == "running")
        if s["used"].any():
            sweeps.append(s)
    return sweeps


def step_deg(angles):
    """Degrees between pings, wrapped at 0/360 like Sweep.step_deg."""
    if len(angles) < 2:
        return 2.0
    return abs((float(angles[1]) - float(angles[0]) + 180.0) % 360.0 - 180.0)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("folder", help="a sonar_data/map_<date_time> folder")
    p.add_argument("--port", type=int, default=8095)
    p.add_argument("--speed", type=float, default=4.0,
                   help="times real time; 0 = as fast as possible")
    p.add_argument("--threshold", type=int, default=None, help="default: what the run used")
    p.add_argument("--max-turn", type=float, default=MAX_TURN_DPS)
    p.add_argument("--epoch", type=int, default=None,
                   help="which Start/Reset of the run to replay (default: the last)")
    args = p.parse_args()

    running = load(args.folder)
    if not running:
        sys.exit(f"no sweeps taken while the map was running in {args.folder}")
    epochs = sorted({int(s["map_epoch"]) for s in running})
    epoch = epochs[-1] if args.epoch is None else args.epoch
    sweeps = [s for s in running if int(s["map_epoch"]) == epoch]
    if not sweeps:
        sys.exit(f"no running sweeps after Start/Reset #{epoch}; the run has {epochs}")
    first = sweeps[0]
    print(f"[INFO] {len(sweeps)} sweeps from Start/Reset #{epoch} (the run has {epochs})")

    webview.serve(args.port)
    webview.enable_map()
    webview.set_target_editable(False)
    webview.set_note("replay - see the Map tab")
    webview.set_range(float(first["range_m"]))
    webview.set_range_editable(False)
    webview.set_sector(*first["sector"], editable=False)
    webview.set_threshold(args.threshold if args.threshold is not None else int(first["threshold"]))
    print(f"[INFO] replay: http://localhost:{args.port}/map")

    play = {"state": "running", "done": False}

    def controls(cmap):
        """Act on the page. True = start the replay over."""
        for cmd in webview.map_commands():
            if cmd == "pause":
                play["state"] = "paused"
            elif cmd in ("resume", "start"):
                if play["done"]:
                    return True
                play["state"] = "running"
            elif cmd == "reset":
                return True
        return webview.threshold() != cmap.threshold

    outliner = Outliner()

    def snapshot(cmap, track, here, n, s):
        c = cmap.crumbs()
        webview.set_map({
            "state": "paused" if play["done"] else play["state"],
            "dvl": "no data" if here is None else ("ok" if here.valid else "lost"),
            "pose": None if here is None else [round(here.x, 3), round(here.y, 3),
                                               round(here.heading_deg, 1)],
            "sonar": None if here is None else [round(v, 3) for v in cmap.sonar_xy(here)],
            "track": [[round(x, 2), round(y, 2)] for x, y in track[-5000:]],
            "crumbs": np.column_stack([np.round(c["x"], 2), np.round(c["y"], 2),
                                       np.round(c["brightness"]), c["seen"]]).tolist(),
            "sweep": f"{n}/{len(sweeps)}" + (" (end)" if play["done"] else ""),
            "count": len(c["x"]), "range": float(s["range_m"]),
            "outlines": outliner.update(c, time.monotonic(), force=play["done"]),
            "sector": [float(v) for v in s["sector"]], "threshold": cmap.threshold,
            "turning": cmap.turning, "skippedTurning": cmap.skipped_turning,
            "slice": reach(*sector_arc(*s["sector"]), float(s["range_m"]))})

    while True:                                 # one pass of the run per loop
        play["done"] = False
        play["state"] = "running"
        cmap = CrumbMap(threshold=webview.threshold(), max_turn_dps=args.max_turn,
                        sonar_fwd_m=float(first["sonar_fwd_m"]),
                        sonar_right_m=float(first["sonar_right_m"]))
        track, recent, here = [], [], None
        outliner.clear()
        last_snap, prev_t, n, s = 0.0, None, 0, first
        restart = False
        for n, s in enumerate(sweeps, 1):
            cmap.new_sweep()
            poses, times = s["pose_x_y_heading_valid"], s["ping_times"]
            tilt = s.get("pose_roll_pitch")     # not in runs saved before Oct 5
            for i, ang in enumerate(s["angles_deg"]):
                while True:                     # wait here while paused
                    restart = controls(cmap)
                    if restart or play["state"] == "running":
                        break
                    snapshot(cmap, track, here, n, s)
                    time.sleep(0.1)
                if restart:
                    break
                if not s["used"][i]:
                    continue
                t = float(times[i])
                if args.speed > 0 and prev_t is not None:
                    time.sleep(min(max(t - prev_t, 0.0), 1.0) / args.speed)
                prev_t = t
                x, y, h, ok = (float(v) for v in poses[i])
                if math.isnan(x):
                    continue
                roll, pitch = (0.0, 0.0) if tilt is None else (float(v) for v in tilt[i])
                here = Pose(t, x, y, h, bool(ok), roll, pitch)
                # turn rate from the saved headings, over the same window as live
                recent = [r for r in recent if r[0] > t - 1.0] + [(t, h)]
                before = [r for r in recent[:-1] if r[0] <= t - TURN_WINDOW_S] or recent[:1]
                turn = (wrap180(h - before[-1][1]) / (t - before[-1][0])
                        if before[-1][0] < t else None)
                if not track or math.hypot(x - track[-1][0], y - track[-1][1]) >= TRACK_STEP_M:
                    track.append((x, y))
                cmap.add_ping(ang, s["image"][i], float(s["metres_per_bin"]), here,
                              step_deg(s["angles_deg"]), float(s["range_m"]), turn)
                if time.monotonic() - last_snap >= SNAPSHOT_S:
                    snapshot(cmap, track, here, n, s)
                    last_snap = time.monotonic()
            if restart:
                break
        if restart:
            continue
        play["done"] = True
        snapshot(cmap, track, here, n, s)
        print(f"[INFO] end of run: {len(cmap)} crumbs. Resume or Reset on the page plays it again.")
        while not controls(cmap):
            time.sleep(0.1)


if __name__ == "__main__":
    main()
