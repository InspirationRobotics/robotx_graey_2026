#!/usr/bin/env python3
"""Measure one object over several sweeps and store what it looks like.

    python3 tools/sonar_record.py --udp 127.0.0.1:9092 --name pvc_pipe \
        --near 1.3 --sweeps 8 --no-floor --notes "1.3 m off starboard, pool"

Everything lands in one file, ~/sonar_data/objects.json by default, with each
object under its own name. Run it again with the same --name and the new sweeps
are added to the old ones. Use a different --name for a different thing, and
measure the CLUTTER as well as the target: knowing what a pool wall scores is
what tells you whether your target is actually distinguishable.

WHICH BLOB IS THE OBJECT

A sweep usually finds several. Two ways to say which one you mean:

    --near 1.3      the one closest to 1.3 m. Use the distance you measured.
    --pick 0        by row number, as printed.

--near is the honest one, because you know where you put the thing. --pick is
for when the table is stable and you can see which row it is.

WHAT COMES OUT

Raw samples, one per sweep, plus a profile built from them: the median becomes
the ideal and the observed spread becomes the edges. The profile is what
identify() scores against. Both are kept, so a profile can be rebuilt later
under different rules without getting wet again.

Only pose-invariant features go into the profile - height above floor,
brightness, radial thickness, solidity. Span and width are recorded but left
out, because they change with which way you are looking at the pipe rather than
with what it is.

Needs the workspace sourced, or PYTHONPATH=. from the repo root.
"""
import argparse
import sys
import time

from robotx_graey_2026.api.sonar import library as lib
from robotx_graey_2026.api.sonar import settings as S
from robotx_graey_2026.api.sonar.detect import perceive
from robotx_graey_2026.api.sonar.sweep import Sonar


def pick(candidates, near=None, index=None):
    """Which detection is the object we mean?"""
    if not candidates:
        return None
    if index is not None:
        return candidates[index] if index < len(candidates) else None
    if near is not None:
        return min(candidates, key=lambda d: abs(d.range_m - near))
    return candidates[0]


def main():
    sys.stdout.reconfigure(line_buffering=True)
    p = argparse.ArgumentParser()
    p.add_argument("--udp", help="host:port of pingproxy")
    p.add_argument("--device", help="serial by-id path instead of pingproxy")
    p.add_argument("--name", required=True, help="what to call this object")
    p.add_argument("--library", default=lib.DEFAULT_PATH)
    p.add_argument("--notes", default="", help="where it was, how far, anything")
    p.add_argument("--sweeps", type=int, default=8)
    p.add_argument("--near", type=float,
                   help="pick the blob nearest this range, in metres")
    p.add_argument("--pick", type=int, help="pick this row number instead")
    p.add_argument("--range", type=float, default=4.0)
    p.add_argument("--start", type=float, default=0.0)
    p.add_argument("--end", type=float, default=360.0)
    p.add_argument("--step", type=float, default=2.0)
    p.add_argument("--threshold", type=int, default=S.DETECT["threshold"])
    p.add_argument("--down-gradian", type=int, default=S.DOWN_GRADIAN)
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

    samples, missed = [], 0
    for i in range(args.sweeps):
        sweep = sonar.sweep(args.start, args.end, args.step, args.range)
        per = perceive(sweep, profile=None, tuning={"threshold": args.threshold},
                       require_floor=not args.no_floor)
        d = pick(per.candidates, args.near, args.pick)
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

    if not samples:
        sys.exit("nothing recorded - was the object in the arc, and above the "
                 "threshold?")

    library = lib.load(args.library)
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
        print("\nhow this object scores against the others you have measured.")
        print("two names scoring close together means they are NOT separable:")
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
            print(f"    {other:<20} {sc:.2f}{flag}")
            if other != args.name and worst is None:
                worst = sc
        if worst is not None and worst > 0.5:
            print("    WARNING: something else scores above 0.5 on this "
                  "object's own ideal values. Your features do not separate "
                  "them, and the code will confuse them in the water.")


if __name__ == "__main__":
    main()
