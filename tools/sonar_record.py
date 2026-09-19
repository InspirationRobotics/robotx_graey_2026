#!/usr/bin/env python3
"""Measure one object over several sweeps and store what it looks like.

    python3 tools/sonar_record.py --name pvc_pipe --near 1.3 --sweeps 8 \
        --no-floor --notes "1.3 m off starboard, pool" \
        --device /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D2010VWF-if00-port0

--device takes the by-id path, never a ttyUSB number: the numbers are handed out
in whatever order things enumerate at boot, and on Graey ttyUSB0 is the LED
controller. --udp 127.0.0.1:9092 instead, if pingproxy is running.

Everything lands in one file, ~/sonar_data/objects.json by default, with each
object under its own name. Run it again with the same --name and the new sweeps
are added to the old ones. Use a different --name for a different thing, and
measure the CLUTTER as well as the target: knowing what a pool wall scores is
what tells you whether your target is actually distinguishable.

WHICH BLOB IS THE OBJECT

A sweep usually finds several. Two ways to say which one you mean:

    --near 1.3              the one closest to 1.3 m
    --near 1.3 --bearing 0  the one closest to 1.3 m OFF THE STARBOARD SIDE
    --pick 0                by row number, as printed

--near on its own compares distances and nothing else, so a blob at the right
range on the wrong side wins just as easily as your object. Give --bearing too
whenever anything else sits at a similar distance. Bearings are 0 starboard,
90 up, 180 port, 270 straight down.

--pick is for when the table is stable and you can see which row it is, but row
order can change between sweeps, so prefer the measurements you took yourself.

WHAT IT ACTUALLY COLLECTS, AND WHAT IT THROWS AWAY

Not everything. Per sweep it keeps SEVEN NUMBERS ABOUT ONE BLOB - the one you
pointed at with --near or --pick:

    height_m  brightness  span_deg  solidity  width_m  thickness_m  range_m

Every other blob in that sweep, and the sonar picture itself, is discarded. So
if you later change your mind about the threshold, or want a feature nobody has
written yet - an acoustic shadow, a straightness score - the only way back is
another trip to the water.

--save-raw fixes that. It dumps each sweep exactly as it came off the sonar,
about 200 kB apiece, alongside the angles, the metres-per-bin and the
calibration in force. Any measurement can then be recomputed at a desk. Use it:
pool time is the scarce thing, not disk.

Of the seven, only four go into the profile - height above floor, brightness,
radial thickness, solidity. Span and width are recorded but left out, because
they change with which way you are looking at the pipe rather than with what it
is. The profile takes the median of each as the ideal and the observed spread as
the edges, and both it and the raw samples are kept, so it can be rebuilt under
different rules later.

Needs the workspace sourced, or PYTHONPATH=. from the repo root.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

from robotx_graey_2026.api.sonar import library as lib
from robotx_graey_2026.api.sonar import settings as S
from robotx_graey_2026.api.sonar import webview
from robotx_graey_2026.api.sonar.detect import OK, Perception, perceive
from robotx_graey_2026.api.sonar.sweep import Sonar
from robotx_graey_2026.api.sonar.viewer import Radar, render


def _save_raw(directory, name, index, sweep):
    """Dump one sweep exactly as it came off the sonar.

    The measurements this tool stores are seven numbers about one blob. That is
    what the scorer needs, and it is also everything else thrown away: change
    your mind about the threshold, or want a feature nobody has written yet, and
    the only way back is another trip to the water.

    A raw sweep is the whole picture. Keep them and any measurement can be
    recomputed at a desk, for about 200 kB a sweep.
    """
    os.makedirs(directory, exist_ok=True)
    stem = os.path.join(directory, f"{name}_{int(time.time())}_{index:03d}")
    np.save(stem + ".npy", sweep.image)
    with open(stem + ".json", "w") as fh:
        json.dump({"angles_deg": list(sweep.angles_deg),
                   "metres_per_bin": sweep.metres_per_bin,
                   "heading_deg": sweep.heading_deg,
                   "down_gradian": S.DOWN_GRADIAN,
                   "speed_of_sound": S.SPEED_OF_SOUND}, fh)


def _xy(range_m, angle_deg):
    a = math.radians(angle_deg)
    return range_m * math.cos(a), range_m * math.sin(a)


def pick(candidates, near=None, index=None, bearing=None):
    """Which detection is the object we mean?

    --pick takes a row number straight off the table. Otherwise we take the
    candidate nearest the place you said the object was.

    RANGE ALONE IS NOT ALWAYS ENOUGH, and this is the trap. --near compares
    distances and nothing else, so a blob at the right distance on the WRONG
    SIDE scores exactly as well as the object. In a small pool that is not
    hypothetical: reverberation puts a ring of return at one range in every
    direction at once, and it only has to drift a few centimetres to become the
    closest thing to your number.

    --bearing breaks the tie with the other thing you know - which side you put
    it on. Given both, the pick is by real distance in the scan plane rather
    than by range, so the ring on the far side is half a circle away and cannot
    win.
    """
    if not candidates:
        return None
    if index is not None:
        return candidates[index] if index < len(candidates) else None
    if near is None:
        return candidates[0]
    if bearing is None:
        return min(candidates, key=lambda d: abs(d.range_m - near))
    wx, wy = _xy(near, bearing)
    return min(candidates,
               key=lambda d: math.hypot(_xy(d.range_m, d.angle_deg)[0] - wx,
                                        _xy(d.range_m, d.angle_deg)[1] - wy))


def main():
    sys.stdout.reconfigure(line_buffering=True)
    p = argparse.ArgumentParser()
    p.add_argument("--udp", help="host:port of pingproxy")
    p.add_argument("--device", help="serial by-id path instead of pingproxy")
    p.add_argument("--name", required=True, help="what to call this object")
    p.add_argument("--library", default=lib.DEFAULT_PATH)
    p.add_argument("--reset", action="store_true",
                   help="throw away everything already recorded under --name "
                        "and start this object over. Profiles are built from "
                        "EVERY sample an object has ever had, so one run that "
                        "measured the wrong blob stays in the numbers for good "
                        "unless you do this.")
    p.add_argument("--notes", default="", help="where it was, how far, anything")
    p.add_argument("--sweeps", type=int, default=8)
    p.add_argument("--near", type=float,
                   help="pick the blob nearest this range, in metres")
    p.add_argument("--pick", type=int, help="pick this row number instead")
    p.add_argument("--bearing", type=float, default=None,
                   help="which way the object lies, in degrees: 0 straight out "
                        "to starboard, 90 up, 180 to port, 270 straight down. "
                        "Use it with --near whenever there is clutter at a "
                        "similar distance - it is what stops the pick grabbing "
                        "the reverberation ring on the far side.")
    p.add_argument("--range", type=float, default=4.0)
    p.add_argument("--start", type=float, default=0.0)
    p.add_argument("--end", type=float, default=360.0)
    p.add_argument("--step", type=float, default=2.0)
    p.add_argument("--threshold", type=int, default=S.DETECT["threshold"])
    p.add_argument("--down-gradian", type=int, default=S.DOWN_GRADIAN)
    p.add_argument("--web", action="store_true",
                   help="watch it on http://<this-machine>:8081 while it "
                        "records. One process, so it does not fight the "
                        "viewer for the motor.")
    p.add_argument("--port", type=int, default=8081)
    p.add_argument("--save-raw", default=None,
                   help="also dump every sweep as it came off the sonar, into "
                        "this directory. About 200 kB a sweep, and the only way "
                        "to re-measure something later without getting wet "
                        "again. Use it - pool time is the scarce thing, not disk.")
    p.add_argument("--no-floor", action="store_true",
                   help="skip the floor. Height is then not recorded at all, "
                        "which costs you the strongest identity feature - "
                        "unavoidable in water too shallow to see the bottom.")
    args = p.parse_args()

    if args.near is None and args.pick is None:
        sys.exit("say which blob you mean: --near <metres> or --pick <row>")

    udp = None
    if args.udp:
        host, port = args.udp.split(":")
        udp = (host, int(port))
    sonar = Sonar(device=args.device, udp=udp, down_gradian=args.down_gradian)

    print(f"[INFO] recording '{args.name}' for {args.sweeps} sweeps")
    if args.no_floor:
        print("[INFO] no floor - height_m will be missing from these samples")

    radar = Radar()
    if args.web:
        # No target while recording - this tool exists to find out what the
        # numbers ARE, so there is nothing to score against yet and the box
        # would be a lie.
        webview.set_target("")
        webview.set_target_editable(False)
        webview.set_note("recording - nothing is being scored")
        webview.serve(args.port)
        print(f"[INFO] watch it on http://0.0.0.0:{args.port}")

    samples, missed = [], 0
    for i in range(args.sweeps):
        def live(partial):
            radar.update(partial)
            webview.publish(render(Perception(partial, None, [], None, OK), None,
                                   state=f"RECORDING {args.name}  {i + 1}/{args.sweeps}",
                                   radar=radar))

        sweep = sonar.sweep(args.start, args.end, args.step, args.range,
                            on_ping=live if args.web else None)
        # No target, always. This is the tool that finds out what the numbers
        # are; scoring them against a guess first would be circular.
        per = perceive(sweep, tuning={"threshold": args.threshold},
                       require_floor=not args.no_floor)
        if args.save_raw:
            _save_raw(args.save_raw, args.name, i, sweep)
        if args.web:
            radar.update(sweep)
            webview.publish(render(per, None, radar=radar,
                                   state=f"RECORDING {args.name}  {i + 1}/{args.sweeps}"))
        d = pick(per.candidates, args.near, args.pick, args.bearing)
        if d is None:
            missed += 1
            print(f"  [{i}] nothing to record")
            continue
        s = lib.sample_from(d)
        samples.append(s)
        print(f"  [{i}] range {s.get('range_m', 0):.2f} m  "
              f"bright {s.get('brightness', 0):.0f}  "
              f"span {s.get('span_deg', 0):.0f} deg  "
              f"thick {s.get('thickness_m', 0):.3f} m"
              + (f"  height {s['height_m']:.2f} m" if "height_m" in s else ""))
        if s.get("span_deg", 0) > 90:
            print("        ^^ that span is enormous. Almost certainly the "
                  "reverberation ring, not your object - add --bearing")

    if not samples:
        sys.exit("nothing recorded - was the object in the arc, and above the "
                 "threshold?")

    library = lib.load(args.library)

    if args.reset:
        gone = lib.clear(library, args.name)
        if gone:
            print(f"[INFO] --reset: discarded {gone} earlier sample(s) of "
                  f"'{args.name}'")

    # Appending is right when you are measuring the same thing again and wrong
    # when you have drifted onto something else, and the two look identical from
    # the command line. Say which this is before writing.
    old = library.get(args.name)
    if old and old.get("samples"):
        print(f"\n[INFO] adding to {len(old['samples'])} sample(s) already "
              f"recorded under '{args.name}'.")
        odd = lib.disagreement(old, samples)
        if odd:
            print("[WARN] these sweeps do not look like the earlier ones:")
            for name, mid, lo, hi in odd:
                print(f"       {name:<13} now {mid:>8.3f}, "
                      f"before {lo:.3f} to {hi:.3f}")
            print("       The median of this run sits outside everything the")
            print("       old samples ever saw. That is evidence of TWO objects,")
            print("       not more evidence about one - and the profile is built")
            print("       from both. Re-run with --reset if the earlier run")
            print("       measured the wrong blob.")

    entry = lib.add_samples(library, args.name, samples, args.notes)
    lib.save(library, args.library)

    print(f"\n[INFO] {len(samples)} sampled, {missed} missed. "
          f"'{args.name}' now has {len(entry['samples'])} samples in total.")
    print(f"[INFO] saved to {args.library}")
    print("\nprofile built from every sample of this object so far:")
    for name, spec in sorted(entry["profile"].items()):
        print(f"    {name:<13} min {spec['min']:>8.3f}   "
              f"ideal {spec['ideal']:>8.3f}   max {spec['max']:>8.3f}")

    others = [n for n in library if n != args.name and library[n].get("profile")]
    if others:
        print("\nhow this object scores against the others you have measured,")
        print("fed its OWN ideal values. Two names scoring close together means")
        print("your features do not separate them:")
        worst = None
        from robotx_graey_2026.api.sonar.detect import Detection
        d = Detection(range_m=1.0, angle_deg=0.0, offset_m=0.0,
                      height_m=entry["profile"].get("height_m", {}).get("ideal"),
                      span_deg=0.0,
                      brightness=entry["profile"].get("brightness", {}).get("ideal", 0),
                      solidity=entry["profile"].get("solidity", {}).get("ideal", 0),
                      contour=None,
                      thickness_m=entry["profile"].get("thickness_m", {}).get("ideal"))
        for other, sc in reversed(lib.rank(d, library)):
            flag = "  <- this one" if other == args.name else ""
            print(f"    {other:<20} {sc * 100:3.0f}%{flag}")
            if other != args.name and worst is None:
                worst = sc
        if worst is not None and worst > 0.5:
            print("    WARNING: something else scores above 50% on this "
                  "object's own ideal values. Your features do not separate "
                  "them, and the code will confuse them in the water.")


if __name__ == "__main__":
    main()
