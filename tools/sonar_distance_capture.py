#!/usr/bin/env python3
"""Guided capture: the pipe at several distances, one stop at a time.

    python3 tools/sonar_distance_capture.py \
        --device /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D2010VWF-if00-port0

For each stop it tells you where to put the pipe, then KEEPS SWEEPING so you
can position it: the radar is live on http://<graey>:8081, and a line per sweep
says where the nearest blob is ("1.62 m - move it 0.6 m closer"). Press Enter
when it's right and it measures that stop. s skips a stop, q finishes early.

WHY. Thickness and brightness change with distance - a close pipe echoes
louder, so more of its echo clears the threshold and it looks thicker. To fix
that properly we need the same pipe measured at known distances.

WHAT IT WRITES - and what it leaves alone. Every MEASURED sweep is saved raw
(same format as sonar_record.py --save-raw) into one folder, plus summary.csv.
Positioning sweeps are not saved. It scores against --target so you can see
the effect live, but it NEVER writes to the object library (objects.json).

Needs PYTHONPATH=. from the repo root.
"""
import argparse
import csv
import os
import queue
import signal
import socket
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sonar_record import _save_raw, pick  # noqa: E402  same pick rule as the recorder

from robotx_graey_2026.api.sonar import library as lib  # noqa: E402
from robotx_graey_2026.api.sonar import settings as S  # noqa: E402
from robotx_graey_2026.api.sonar import webview  # noqa: E402
from robotx_graey_2026.api.sonar.detect import perceive  # noqa: E402
from robotx_graey_2026.api.sonar.sweep import Sonar, arc_around  # noqa: E402
from robotx_graey_2026.api.sonar.viewer import Radar, render  # noqa: E402

WARN_OFF_M = 0.3    # measured blob this far from the stop = probably not the pipe
CLOSE_ENOUGH_M = 0.15
NOT_PIPE_PCT = 20   # below this, ask for a look at the viewer (the score itself is what distance breaks)


def parse_distances(text):
    out = []
    for part in text.split(","):
        d = float(part)
        if d <= S.MIN_RANGE_M:
            raise ValueError(f"{d} m is inside the sonar's {S.MIN_RANGE_M} m blind zone")
        out.append(d)
    return out


def port_free(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("0.0.0.0", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def start_keyboard():
    """Read lines in the background so sweeping can go on while we wait."""
    lines = queue.Queue()

    def reader():
        while True:
            line = sys.stdin.readline()
            if not line:            # Ctrl-D / end of input
                lines.put(None)
                return
            lines.put(line.strip().lower())

    threading.Thread(target=reader, daemon=True).start()
    return lines


def key_action(lines):
    """None if nothing typed yet, else 'go' / 'skip' / 'quit'."""
    try:
        line = lines.get_nowait()
    except queue.Empty:
        return None
    if line is None or line in ("q", "quit"):
        return "quit"
    if line in ("s", "skip"):
        return "skip"
    return "go"


def guidance(d, dist):
    if d is None:
        return "nothing on that side - is the pipe in the sonar's view?"
    off = d.range_m - dist
    conf = "-" if d.confidence is None else f"{d.confidence}%"
    where = f"nearest blob {d.range_m:.2f} m (score {conf})"
    if d.confidence is not None and d.confidence < NOT_PIPE_PCT:
        where += " - low score, check on the viewer that this is the pipe"
    if abs(off) <= CLOSE_ENOUGH_M:
        return f"{where} - looks right, press Enter"
    return f"{where} - move it {abs(off):.1f} m {'closer' if off > 0 else 'farther'}"


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", help="serial by-id path")
    p.add_argument("--udp", help="host:port of pingproxy")
    p.add_argument("--distances", default="1,1.5,2,3",
                   help="stops in metres, in order (default 1,1.5,2,3)")
    p.add_argument("--sweeps", type=int, default=5, help="measured sweeps per stop")
    p.add_argument("--bearing", type=float, default=0.0,
                   help="which way the pipe lies: 0 starboard, 90 up, 180 port")
    p.add_argument("--width", type=float, default=60.0, help="arc width, degrees")
    p.add_argument("--range", type=float, default=4.0)
    p.add_argument("--step", type=float, default=2.0)
    p.add_argument("--threshold", type=int, default=S.DETECT["threshold"])
    p.add_argument("--target", default="pvc_pipe",
                   help="scored for display only - the library is never changed")
    p.add_argument("--port", type=int, default=8081, help="viewer port")
    p.add_argument("--no-web", action="store_true", help="don't serve the viewer")
    p.add_argument("--out", default=None, help="folder for the sweeps and summary")
    args = p.parse_args()

    try:
        distances = parse_distances(args.distances)
    except ValueError as exc:
        sys.exit(f"--distances: {exc}")
    if max(distances) >= args.range:
        sys.exit(f"--range {args.range} m doesn't reach the {max(distances)} m stop")
    try:
        profile = lib.resolve(args.target) if args.target else None
    except (KeyError, ValueError) as exc:
        sys.exit(f"bad --target: {exc}")

    out = args.out or os.path.expanduser(
        f"~/sonar_data/distance_capture_{time.strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(out, exist_ok=True)

    web = not args.no_web
    if web and not port_free(args.port):
        print(f"[WARN] port {args.port} is busy - is another sonar viewer still "
              f"running? Continuing WITHOUT the viewer.")
        web = False

    udp = None
    if args.udp:
        host, port = args.udp.split(":")
        udp = (host, int(port))
    start_deg, end_deg = arc_around(args.bearing, args.width)
    sonar = Sonar(device=args.device, udp=udp)

    radar = Radar()
    if web:
        webview.set_target(args.target or "")
        webview.set_target_editable(False)
        webview.set_note("distance capture - the library is not changed")
        webview.serve(args.port)

    side = {0: "RIGHT (starboard)", 180: "LEFT (port)", 90: "UP", 270: "DOWN"}.get(
        int(args.bearing) % 360, f"bearing {args.bearing:.0f} deg")
    print(f"Pipe goes straight out to the {side}. Measure from the sonar.")
    print(f"Stops: {', '.join(f'{d:g} m' for d in distances)}  -  "
          f"{args.sweeps} measured sweeps each")
    if web:
        print(f"Watch it live: http://graey.local:{args.port}  "
              f"(or http://192.168.2.2:{args.port})")
    print(f"Saving to {out}\n")

    def one_sweep(state):
        sweep = sonar.sweep(start_deg, end_deg, args.step, args.range)
        per = perceive(sweep, target=profile, tuning={"threshold": args.threshold},
                       require_floor=False)
        if web:
            radar.update(sweep)
            webview.publish(render(per, None, state=state, radar=radar,
                                   target=args.target or ""))
        return sweep, per

    keys = start_keyboard()
    rows = []
    quit_all = False
    try:
        for n, dist in enumerate(distances, 1):
            print(f"[{n}/{len(distances)}] Move the pipe to {dist:g} m. "
                  f"Press Enter to measure  (s = skip, q = finish)")
            action = None
            while action is None:           # positioning: sweep until a key
                _, per = one_sweep(f"POSITIONING  stop {n}/{len(distances)}: {dist:g} m")
                action = key_action(keys)
                if action is None:
                    d = pick(per.candidates, dist, None, args.bearing)
                    print(f"   live: {guidance(d, dist)}")
            if action == "quit":
                quit_all = True
                break
            if action == "skip":
                print("   skipped\n")
                continue

            print(f"   measuring at {dist:g} m...")
            found = []
            for i in range(args.sweeps):
                sweep, per = one_sweep(f"MEASURING {dist:g} m  {i + 1}/{args.sweeps}")
                _save_raw(out, f"pipe_{dist:g}m", i, sweep)
                d = pick(per.candidates, dist, None, args.bearing)
                row = {"stop_m": dist, "sweep": i, "found": d is not None,
                       "range_m": None, "offset_m": None, "thickness_m": None,
                       "brightness": None, "solidity": None, "span_deg": None,
                       "conf_pct": None}
                if d is None:
                    print(f"   sweep {i + 1}: nothing found")
                else:
                    row.update(range_m=round(d.range_m, 3), offset_m=round(d.offset_m, 3),
                               thickness_m=round(d.thickness_m, 4),
                               brightness=round(d.brightness, 1),
                               solidity=round(d.solidity, 3),
                               span_deg=round(d.span_deg, 1), conf_pct=d.confidence)
                    found.append(row)
                    flag = ("   <- not near the stop, wrong blob?"
                            if abs(d.range_m - dist) > WARN_OFF_M else "")
                    conf = "-" if d.confidence is None else f"{d.confidence}%"
                    print(f"   sweep {i + 1}: {d.range_m:.2f} m  "
                          f"thick {d.thickness_m:.3f}  bright {d.brightness:.0f}  "
                          f"solid {d.solidity:.2f}  score {conf}{flag}")
                rows.append(row)
            good = [r for r in found if abs(r["range_m"] - dist) <= WARN_OFF_M]
            if good:
                med = lambda k: statistics.median(r[k] for r in good)  # noqa: E731
                print(f"   -> at {dist:g} m: thickness {med('thickness_m'):.3f}, "
                      f"brightness {med('brightness'):.0f}, "
                      f"solidity {med('solidity'):.2f}  ({len(good)}/{args.sweeps} sweeps)\n")
            else:
                print(f"   -> nothing near {dist:g} m - check the pipe's position\n")
    except KeyboardInterrupt:
        # A second Ctrl-C while saving would otherwise lose the summary.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print("\n   stopped - keeping what was measured")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rows:
            with open(os.path.join(out, "summary.csv"), "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)

    if quit_all:
        print("   finished early")
    print(f"Done. {len(rows)} measured sweeps saved in {out}")
    print("The object library was not changed.")


if __name__ == "__main__":
    main()
