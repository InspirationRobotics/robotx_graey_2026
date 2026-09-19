#!/usr/bin/env python3
"""Ping360 radar view -> http://<jetson-ip>:8081   (Ctrl-C to stop)

    on Graey, once:   pingproxy.py --device /dev/ttyUSB0 --port 9092
    then:             python3 tools/sonar_pole_test.py --udp 127.0.0.1:9092 --no-floor --web

THE TARGET IS A SETTING, AND IT STARTS BLANK

Without a target nothing is judged. Every blob is measured and listed, and the
radar draws no outlines, because nothing has been said about what counts. That
is the mode for looking, and for finding out what a real object's numbers are.

Give it a target and each blob gets a confidence from 0 to 100%, drawn round its
real shape and shaded by that number - dark for a sure match, pale for a poor
one. The panel then breaks the number down feature by feature, so you can see
WHICH one is costing you the score rather than only that something is.

With --web you change the target in the browser: type in the box at the top,
press enter, and the sweep already on screen is re-judged at once. No restart,
and no waiting out the twenty seconds of the next sweep. --target only seeds
that box:

    --target pvc_pipe                       an object you measured with
                                            tools/sonar_record.py
    --target height_m=1.5,brightness=140    ideal values typed straight in
    --target height_m=1.5 --tolerance 0.3   the same, but fussier

Nothing here has a default target. Outlines you did not ask for are a lie about
what the code knows.

You can equally run the whole thing from a laptop and leave only pingproxy on
the sub - pingproxy binds 0.0.0.0, so --udp <jetson-ip>:9092 works from
anywhere on the network, and then you get a real window with live calibration
keys instead of a browser tab.

WHAT THIS TESTS
    The sonar talks through the sweep code, and DOWN_GRADIAN is calibrated -
    which sonar angle really points down. Blobs are found, measured and shown
    with real numbers. In deep enough water, the floor detector finds bottom.

WHAT THIS DOES NOT TEST
    A vertical pole is not a horizontal pipeline. It sits ON the bottom, so
    there is no clear water underneath it - and height above the floor is the
    main way this code tells a pipeline from the seabed. It is round about its
    vertical axis, so yawing barely changes how it looks, which leaves the
    whole span-to-orientation idea with nothing to bite on. And it does not run
    anywhere, so there is no following and no bends.

    So this proves the plumbing and the measurements. It cannot tell you
    whether ORIENT or FOLLOW work.

Ping Viewer connects to the same proxy, so use it first to prove the sonar is
alive. Several clients may connect at once, but they all steer one motor - run
Ping Viewer and this one at a time when you care about the numbers.

Needs the package importable, either way round:
    source /root/robotx_ws/install/setup.bash        # on Graey
    PYTHONPATH=. python3 tools/sonar_pole_test.py    # from a plain checkout

Nothing here imports rclpy. The sonar package underneath is byte-identical to
the one on Onyx, which runs ROS 1.
"""
import argparse
import os
import sys
import time

import cv2

from robotx_graey_2026.api.sonar import library as lib
from robotx_graey_2026.api.sonar import settings as S
from robotx_graey_2026.api.sonar import webview
from robotx_graey_2026.api.sonar.detect import perceive, rescore
from robotx_graey_2026.api.sonar.driver import explain
from robotx_graey_2026.api.sonar.floor import Floor
from robotx_graey_2026.api.sonar.sweep import Sonar
from robotx_graey_2026.api.sonar.viewer import Radar, render


def main():
    # Line-buffer stdout so each sweep prints as it finishes even when piped
    # into tee. Python block-buffers a pipe by default, and since one sweep is
    # only a few hundred bytes the terminal stays blank for dozens of them,
    # which looks exactly like a stalled sonar.
    sys.stdout.reconfigure(line_buffering=True)

    p = argparse.ArgumentParser()
    p.add_argument("--udp", help="host:port of pingproxy, e.g. 127.0.0.1:9092")
    p.add_argument("--device", help="serial by-id path, if not using pingproxy")
    p.add_argument("--range", type=float, default=4.0, help="max range, metres")
    p.add_argument("--start", type=float, default=0.0, help="sweep start, degrees")
    p.add_argument("--end", type=float, default=360.0, help="sweep end, degrees")
    p.add_argument("--step", type=float, default=2.0, help="degrees between pings")
    p.add_argument("--down-gradian", type=int, default=S.DOWN_GRADIAN,
                   help="which hardware gradian points straight down")
    p.add_argument("--assume-floor", type=float, default=None,
                   help="metres below the sonar. Use in shallow water where the "
                        "bottom sits inside the 0.75 m blind zone and genuinely "
                        "cannot be measured.")
    p.add_argument("--no-floor", action="store_true",
                   help="skip the floor entirely. Every blob is reported with "
                        "its range and bearing and nothing is filtered by "
                        "height. This is the mode for a first look, and the "
                        "right one at short range - see the note in main().")
    p.add_argument("--threshold", type=int, default=S.DETECT["threshold"])
    p.add_argument("--target", default=None,
                   help="what to score blobs against: a name from the object "
                        "library, or ideals like 'height_m=1.5,brightness=140'. "
                        "Leave it off and nothing is judged and no rings are "
                        "drawn - just the picture and the numbers.")
    p.add_argument("--library", default=lib.DEFAULT_PATH,
                   help="where the recorded objects live")
    p.add_argument("--tolerance", type=float, default=lib.TOLERANCE,
                   help="half-width of an ideals-only target, as a fraction of "
                        "the ideal. Smaller is fussier. Ignored for a library "
                        "name, whose edges come from real samples.")
    p.add_argument("--web", action="store_true",
                   help="serve the view at http://<this-machine>:8081 instead "
                        "of opening a window. Use this on the Jetson.")
    p.add_argument("--port", type=int, default=8081,
                   help="port for --web. 8080 is the OAK-D view.")
    p.add_argument("--headless", action="store_true",
                   help="numbers only, no view at all")
    p.add_argument("--save-dir", default=None,
                   help="also write a PNG per sweep into this directory")
    p.add_argument("--once", action="store_true", help="one sweep, then exit")
    args = p.parse_args()

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    udp = None
    if args.udp:
        host, port = args.udp.split(":")
        udp = (host, int(port))

    # Resolve the target ONCE, here, and hand the resulting profile down. A name
    # resolved per sweep would re-read the library file every few seconds and,
    # worse, would silently change what the radar means halfway through a run.
    try:
        profile = lib.resolve(args.target, path=args.library,
                              tolerance=args.tolerance)
    except (KeyError, ValueError) as exc:
        sys.exit(f"bad --target: {exc}")
    label = args.target or ""

    sonar = Sonar(device=args.device, udp=udp, down_gradian=args.down_gradian)

    # A FULL circle by default, not the downward half the mission states use.
    # In shallow water a pole standing a couple of metres away sits near the
    # horizontal, not below you, so a downward-only sweep would miss most of it.
    pings = int((args.end - args.start) / args.step)
    print(f"[INFO] {pings} pings per sweep at {args.range} m")
    windowed = not (args.web or args.headless)
    hint = "  (use [ and ] to adjust)" if windowed else ""
    print(f"[INFO] down gradian = {args.down_gradian}{hint}")
    if args.assume_floor:
        print(f"[INFO] assuming a floor {args.assume_floor} m below - not measuring it")
    if args.no_floor:
        print("[INFO] ignoring the floor - reporting range and bearing only")
    if profile is None:
        print("[INFO] no --target: measuring everything, judging nothing, "
              "drawing no rings")
    else:
        print(f"[INFO] target '{label}':")
        for name, spec in sorted(profile.items()):
            print(f"       {name:<13} min {spec['min']:>8.3f}   "
                  f"ideal {spec['ideal']:>8.3f}   max {spec['max']:>8.3f}")
    if args.web:
        webview.set_target(label)
        webview.set_note("blank = no rings" if profile is None
                         else f"{len(profile)} features")
        webview.serve(args.port)
        print(f"[INFO] serving on http://0.0.0.0:{args.port}")
        print("[INFO] the target box on that page changes what is scored, live")

    down_gradian = args.down_gradian
    tuning = {"threshold": args.threshold}

    # The picture persists across sweeps: each angle keeps its last measurement
    # until the head comes back round, as in Ping Viewer. Detections shown while
    # a sweep is in progress are the LAST COMPLETE sweep's, so the rings stay
    # attached to the picture underneath them instead of vanishing the moment a
    # new sweep starts and has only seen a few degrees.
    radar = Radar()
    shown = [None]

    # The target can be retyped in the browser at any time. `applied` is the
    # string currently in force; a new one is resolved once, here, and a bad one
    # leaves the working profile alone rather than blanking the radar.
    applied = [label]

    def retarget():
        nonlocal profile, label
        wanted = webview.target()
        if wanted == applied[0]:
            return
        applied[0] = wanted
        try:
            profile = lib.resolve(wanted or None, path=args.library,
                                  tolerance=args.tolerance)
        except (KeyError, ValueError) as exc:
            webview.set_note(f"ignored '{wanted}': {exc}  (still on '{label}')")
            print(f"[WARN] ignored target '{wanted}': {exc}")
            return
        label = wanted
        webview.set_note("blank = no rings" if profile is None
                         else "  ".join(f"{k} {v['ideal']:g}"
                                        for k, v in sorted(profile.items())))
        print(f"[INFO] target is now '{wanted or '(none)'}'")

        # Answer at once. Measuring is the slow half and it does not depend on
        # the target, so the sweep already on screen can be re-judged and
        # republished now rather than in another fifteen to twenty-five seconds.
        if shown[0] is not None:
            rescore(shown[0], profile)
            webview.publish(render(shown[0], None, radar=radar, target=label,
                                   state=f"RESCORED  down={down_gradian}"))

    # A sweep takes many seconds, so publish partial frames while the head is
    # still moving. Without this the browser shows one frozen picture and you
    # cannot tell a working sonar from a stalled one.
    #
    # This is also where the target box gets read. Checking it once per ping
    # rather than once per sweep is what makes pressing enter feel immediate
    # instead of costing you the rest of a twenty-second sweep.
    def live(partial):
        retarget()
        radar.update(partial)
        per = shown[0]
        if per is None:
            per = perceive(partial, target=profile, tuning=tuning, floor=None,
                           require_floor=False)
        webview.publish(render(per, None, state=f"SCANNING  down={down_gradian}",
                               radar=radar, target=label))

    while True:
        if args.web:
            retarget()                  # in case a sweep returns with no pings
        sonar.down_gradian = down_gradian
        t0 = time.time()
        sweep = sonar.sweep(args.start, args.end, args.step, args.range,
                            on_ping=live if args.web else None)
        radar.update(sweep)
        floor = Floor.from_known_depth(args.assume_floor) if args.assume_floor else None

        # profile is None unless --target was given: measure everything, judge
        # nothing. That is discovery mode and it is the default on purpose -
        # the first thing you want from new hardware is what the real numbers
        # ARE, not whether they match a guess.
        #
        # A trap worth knowing at short range: the floor detector takes the
        # FARTHEST return in each direction, so if the bottom is beyond --range
        # it latches onto whatever large thing IS in view - the pole - calls
        # that the floor, and then reports nothing above it. Use --no-floor
        # until the range genuinely reaches the bottom.
        per = perceive(sweep, target=profile, tuning=tuning, floor=floor,
                       require_floor=not args.no_floor)

        shown[0] = per
        elapsed = time.time() - t0
        view = None
        if not args.headless:
            view = render(per, None, state=f"POLE TEST  down={down_gradian}",
                          radar=radar, target=label)
            cv2.putText(view, f"sweep {elapsed:.1f}s", (16, view.shape[0] - 18),
                        cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        if args.save_dir and view is not None:
            path = os.path.join(args.save_dir, f"sweep_{int(time.time())}.png")
            cv2.imwrite(path, view)

        if windowed:
            cv2.imshow("sonar pole test", view)
            if args.once:
                cv2.waitKey(0)
                break
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                name = f"pole_{int(time.time())}.png"
                cv2.imwrite(name, view)
                print(f"[INFO] saved {name}")
            if key == ord("["):
                down_gradian = (down_gradian - 5) % S.GRADIANS
                print(f"[INFO] down gradian = {down_gradian}")
            if key == ord("]"):
                down_gradian = (down_gradian + 5) % S.GRADIANS
                print(f"[INFO] down gradian = {down_gradian}")
            continue

        # web and headless both print, because over SSH the numbers are the
        # thing you can actually read back to someone.
        _print_sweep(per, elapsed, down_gradian)
        if view is not None:
            webview.publish(view)
        if args.once:
            break

    if windowed:
        cv2.destroyAllWindows()


def _print_sweep(per, elapsed, down_gradian):
    """The same information the viewer shows, as text."""
    print(f"\n[{elapsed:.1f}s] down={down_gradian}  {len(per.candidates)} candidates")
    if per.floor is not None:
        print(f"    floor {per.floor.depth_m:.2f} m below "
              f"(agreement {per.floor.confidence:.0%})")
    if not per.candidates:
        print(f"    nothing found: {explain(per)}")
        return
    print(f"    {'#':<3}{'range':>8}{'bearing':>9}{'height':>9}"
          f"{'span':>7}{'bright':>8}{'solid':>7}{'thick':>8}{'conf':>7}")
    for i, d in enumerate(per.candidates[:6]):
        height = " -" if d.height_m is None else f"{d.height_m:.2f}"
        thick = " -" if d.thickness_m is None else f"{d.thickness_m:.3f}"
        conf = " -" if d.confidence is None else f"{d.confidence}%"
        print(f"    {i:<3}{d.range_m:>7.2f}m{d.angle_deg:>8.0f}d{height:>9}"
              f"{d.span_deg:>6.0f}d{d.brightness:>8.0f}{d.solidity:>7.2f}"
              f"{thick:>8}{conf:>7}")


if __name__ == "__main__":
    main()
