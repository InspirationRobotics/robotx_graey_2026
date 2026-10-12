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

from robotx_graey_2026.api.sonar import settings as S
from robotx_graey_2026.api.sonar import webview
from robotx_graey_2026.api.sonar.outlines import Outliner
from robotx_graey_2026.api.sonar.crumbs import LITTLE_DOTS_NOTE, MAX_TURN_DPS, CrumbMap
from robotx_graey_2026.api.sonar.pose import Pose, wrap180
from robotx_graey_2026.api.sonar.detect import perceive
from robotx_graey_2026.api.sonar.sweep import Sweep
from robotx_graey_2026.api.sonar.viewer import SIZE, Radar, render
from sonar_map import SNAPSHOT_S, TRACK_STEP_M, reach, sector_arc

TURN_WINDOW_S = 0.3         # the same window DeadReckoner.turn_rate uses
RADAR_S = 0.5               # Radar tab picture at most this often (wall clock)


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
    p.add_argument("--floor-cut", type=float, default=0.0,
                   help="m above the floor (DVL altitude) with no crumbs; default 0 = off, as the Map "
                        "page has no floor cut since Oct 10 (a mission run's own value is in its sweeps)")
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
    webview.set_floor_cut(args.floor_cut)
    webview.set_fade(bool(first.get("fade", True)))           # not saved before Oct 10 pm
    saved = first.get("band_m")                                # nor this
    webview.set_band(tuple(saved) if saved is not None and np.isfinite(saved).all() else None)
    webview.set_compact(bool(first.get("compact", False)))            # not saved before Oct 11
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
        return (webview.threshold() != cmap.threshold or webview.floor_cut() != cmap.floor_cut_m
                or webview.fade() != cmap.fade
                or webview.band() != cmap.band or webview.compact() != cmap.compact)

    outliner = Outliner()
    radar, last_radar = Radar(), [-1e9]

    def show_radar(s, force=False):
        """The Radar tab's picture of sweep s, drawn as the live tool draws it,
        so the height band's lines can be dragged in a replay too."""
        if not force and time.monotonic() - last_radar[0] < RADAR_S:
            return
        sw = Sweep(s["image"], [float(a) for a in s["angles_deg"]], float(s["metres_per_bin"]),
                   None, [float(t) for t in s["ping_times"]])
        radar.update(sw)
        per = perceive(sw, target=None, tuning={"threshold": S.DETECT["threshold"]},
                       floor=None, require_floor=False)
        webview.publish(render(per, None, radar=radar, size=SIZE,
                               state=f"REPLAY  range={float(s['range_m']):g}m",
                               note=LITTLE_DOTS_NOTE if webview.compact() else ""))
        webview.set_radar_geom(SIZE, radar.max_range_m)
        last_radar[0] = time.monotonic()

    def snapshot(cmap, track, here, n, s):
        crumbs, c = cmap.for_page()
        webview.set_map({
            "state": "paused" if play["done"] else play["state"],
            "dvl": "no data" if here is None else ("ok" if here.valid else "lost"),
            "pose": None if here is None else [round(here.x, 3), round(here.y, 3),
                                               round(here.heading_deg, 1)],
            "sonar": None if here is None else [round(v, 3) for v in cmap.sonar_xy(here)],
            "track": [[round(x, 2), round(y, 2)] for x, y in track[-5000:]],
            "crumbs": crumbs,
            "sweep": f"{n}/{len(sweeps)}" + (" (end)" if play["done"] else ""),
            "count": len(crumbs), "range": float(s["range_m"]),
            "outlines": outliner.update(c, time.monotonic(), force=play["done"]),
            "sector": [float(v) for v in s["sector"]], "threshold": cmap.threshold,
            "floorCut": cmap.floor_cut_m, "fade": cmap.fade,
            "band": list(cmap.band) if cmap.band else None, "compact": cmap.compact,
            "turning": cmap.turning, "skippedTurning": cmap.skipped_turning,
            "slice": reach(*sector_arc(*s["sector"]), float(s["range_m"]))})

    while True:                                 # one pass of the run per loop
        play["done"] = False
        play["state"] = "running"
        cmap = CrumbMap(threshold=webview.threshold(), max_turn_dps=args.max_turn,
                        floor_cut_m=webview.floor_cut(), fade=webview.fade(),
                        band=webview.band(), compact=webview.compact(),
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
                if S.VN_PITCH_BACKWARDS and "vn_pitch_unflipped" not in s:
                    pitch = -pitch          # saved before Oct 10, with the VN-100's pitch backwards
                alt = float(s["pose_alt"][i]) if "pose_alt" in s else -1.0     # not before Oct 9
                here = Pose(t, x, y, h, bool(ok), roll, pitch, alt)
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
            show_radar(s)
        if restart:
            continue
        play["done"] = True
        cmap.end_sweep()                    # little dots only: the last sweep's crumbs
        snapshot(cmap, track, here, n, s)
        show_radar(s, force=True)
        print(f"[INFO] end of run: {len(cmap)} crumbs. Resume or Reset on the page plays it again.")
        while not controls(cmap):
            time.sleep(0.1)


if __name__ == "__main__":
    main()
