#!/usr/bin/env python3
"""Sonar search-and-approach: let the state machine drive the sub.

    python3 tools/sonar_follow.py --target pvc_pipe --device SONAR
    python3 tools/sonar_follow.py --target pvc_pipe --device SONAR --live

where SONAR is the by-id path, never a ttyUSB number - on Graey that is
    /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D2010VWF-if00-port0
and ttyUSB0 is the LED controller. Or --udp 127.0.0.1:9092 with pingproxy up.

--target is required and has no default. This tool moves the sub towards
whatever matches it, so "what am I hunting" is not something to leave implied:
a wrong default here is a sub swimming confidently at the wrong object. Pass a
name from the object library, or ideals typed straight in - say
--target height_m=1.5,brightness=140 for something pipeline-shaped you have not
measured yet.

SAFETY, copied from mission_base.py because the reasoning is the same: dry run
is the DEFAULT. It prints every target it would command and sends nothing. Pass
--live only in the water, with the e-stop in reach.

This never arms and never changes mode. Put the vehicle in GUIDED and ARM it
yourself in QGroundControl first, exactly as tools/guided_goto.py expects. A
script that can arm the sub is a script that can arm the sub by accident.

HOW IT CONNECTS
    The Driver emits body-frame moves - FORWARD 0.5 m, STRAFE -0.4 m, YAW_BY 45
    deg - and frames.body_to_world(n, e, yaw, forward, right) takes exactly
    those units. So each Action becomes one NED target and goes out through the
    same Link.goto_ned() that guided_goto.py uses. There is no translation layer
    worth the name; that is the point of the Driver returning Actions instead of
    touching the vehicle itself.

WHY THE SLOW CADENCE IS FINE
    Link.goto_posvel's docstring warns that goto_ned() must not be sent faster
    than every few seconds, because wp_nav resets its S-curve on every message
    and the sub never leaves its acceleration phase. A sweep takes about nine
    seconds, so one new target per sweep sits naturally inside that limit. The
    RESEND_S gate below is the same one mission_base uses, for the same reason.

WHAT THIS STILL DOES NOT PROVE
    A pole is not a pipeline. ORIENT and FOLLOW judge alignment by how far the
    span COLLAPSES when you turn, and a vertical pole looks the same from every
    heading - so those states cannot be validated against one. Expect SEARCH and
    APPROACH to be meaningful here and the rest to be theatre.

Needs the workspace sourced, or PYTHONPATH=. from the repo root.
"""
import argparse
import math
import sys
import time

from robotx_graey_2026.api.navigation.frames import body_to_world
from robotx_graey_2026.api.pixhawk.mavlink import Link
from robotx_graey_2026.api.sonar import library as lib
from robotx_graey_2026.api.sonar import settings as S
from robotx_graey_2026.api.sonar import webview
from robotx_graey_2026.api.sonar.detect import OK, Perception, perceive
from robotx_graey_2026.api.sonar.driver import (
    Driver, FINISHED, FORWARD, HOLD, STRAFE, YAW_BY, YAW_TO)
from robotx_graey_2026.api.sonar.sweep import Sonar
from robotx_graey_2026.api.sonar.viewer import Radar, render

RESEND_S = 3.0          # matches mission_base: unchanged targets go no faster
POSE_TIMEOUT_S = 5.0
MAX_STEP_M = 1.0        # refuse to command a jump bigger than this, whatever
                        # the Driver asks for. A detector bug should not become
                        # a three-metre lunge.


def warn_settings(target, library_path, current):
    """Same check sonar_pole_test does, and it matters more here.

    A profile scored under the wrong threshold does not fail loudly - it scores
    zero, the state machine sees nothing, and the sub searches an empty pool
    for as long as you let it.
    """
    if not target or "=" in target:
        return []
    try:
        clash = lib.settings_clash(lib.load(library_path), target, current)
    except Exception:
        return []
    if not clash:
        return []
    out = [f"'{target}' was measured with different settings:"]
    for key, was, now in clash:
        out.append(f"    {key}: recorded at {was}, running {now}")
    out.append("brightness is measured over the pixels that passed the")
    out.append("threshold, so scores will be wrong. Match it, or re-record.")
    return out


def refresh_pose(link, pose, timeout=POSE_TIMEOUT_S, need_pos=True):
    """Drain MAVLink until we have a fresh attitude, and a position if needed.

    need_pos is False for a dry run. Position is used for exactly one thing,
    turning a body-frame move into an absolute NED target, and a dry run never
    sends one. Demanding it anyway made the tool unrunnable wherever the EKF has
    no position fix - on the bench, and in water too shallow for the DVL to
    lock - which is precisely where you want to rehearse the state machine.
    ATTITUDE comes straight off the IMU and is always there.
    """
    def handle(kind, m):
        if kind == 'LOCAL_POSITION_NED':
            pose['pos'] = (m.x, m.y, m.z)
        elif kind == 'ATTITUDE':
            pose['yaw'] = m.yaw

    t0 = time.time()
    pose['pos'] = pose['yaw'] = None
    while time.time() - t0 < timeout:
        if pose['yaw'] is not None and (pose['pos'] is not None or not need_pos):
            break
        link.heartbeat()                    # MAVProxy will not route to us first
        link.drain(handle)
        time.sleep(0.05)
    return pose['yaw'] is not None and (pose['pos'] is not None or not need_pos)


def action_to_target(action, n, e, down, yaw):
    """One Action -> (north, east, down, yaw) target. None means 'no move'.

    Depth is passed straight through. This tool does not change depth, so a
    detector that goes wrong cannot drive the sub into the bottom.
    """
    if action.kind == FORWARD:
        step = max(-MAX_STEP_M, min(MAX_STEP_M, action.value))
        tn, te = body_to_world(n, e, yaw, step, 0.0)
        return tn, te, down, yaw
    if action.kind == STRAFE:
        step = max(-MAX_STEP_M, min(MAX_STEP_M, action.value))
        tn, te = body_to_world(n, e, yaw, 0.0, step)
        return tn, te, down, yaw
    if action.kind == YAW_BY:
        return n, e, down, yaw + math.radians(action.value)
    if action.kind == YAW_TO:
        return n, e, down, math.radians(action.value)
    if action.kind == HOLD:
        return n, e, down, yaw
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--udp", help="host:port of pingproxy, e.g. 127.0.0.1:9092")
    p.add_argument("--device", help="serial by-id path, if not using pingproxy")
    p.add_argument("--mavlink", default="udpout:127.0.0.1:14553")
    p.add_argument("--target", required=True,
                   help="what to hunt: a name from the object library, or "
                        "ideals like 'height_m=1.5,brightness=140'")
    p.add_argument("--library", default=lib.DEFAULT_PATH)
    p.add_argument("--tolerance", type=float, default=lib.TOLERANCE,
                   help="half-width of an ideals-only target, as a fraction of "
                        "the ideal")
    p.add_argument("--down-gradian", type=int, default=S.DOWN_GRADIAN)
    p.add_argument("--threshold", type=int, default=S.DETECT["threshold"])
    p.add_argument("--no-floor", action="store_true",
                   help="skip the floor. Right at short range, where the bottom "
                        "is beyond the sweep and the detector would mistake the "
                        "target for it.")
    p.add_argument("--live", action="store_true",
                   help="ACTUALLY SEND setpoints. Without this it only prints.")
    p.add_argument("--web", action="store_true", help="radar view on :8081")
    p.add_argument("--port", type=int, default=8081)
    p.add_argument("--max-sweeps", type=int, default=40,
                   help="stop after this many sweeps, whatever the state")
    p.add_argument("--stop-at", default=None,
                   help="stop the moment the driver enters this state. "
                        "--stop-at ORIENT means 'search, approach, and park "
                        "when you are overhead' - which is the whole of what a "
                        "vertical pole can test, since ORIENT and FOLLOW judge "
                        "alignment by the span collapsing and a pole looks the "
                        "same from every heading.")
    args = p.parse_args()

    udp = None
    if args.udp:
        host, port = args.udp.split(":")
        udp = (host, int(port))

    # Fail on a bad target before touching the sonar or the vehicle.
    try:
        profile = lib.resolve(args.target, path=args.library,
                              tolerance=args.tolerance)
    except (KeyError, ValueError) as exc:
        sys.exit(f"bad --target: {exc}")

    for _line in warn_settings(args.target, args.library,
                               {"threshold": args.threshold,
                                "down_gradian": args.down_gradian}):
        print(f"[WARN] {_line}")

    sonar = Sonar(device=args.device, udp=udp, down_gradian=args.down_gradian)
    link = Link(args.mavlink, 191)
    pose = {'pos': None, 'yaw': None}

    # A dry run needs heading only. --live needs position too, because that is
    # what an absolute NED setpoint is built from.
    need_pos = args.live
    if not refresh_pose(link, pose, need_pos=need_pos):
        if pose['yaw'] is None:
            sys.exit("no ATTITUDE - is MAVProxy running, and is 14553 free?")
        sys.exit("no LOCAL_POSITION_NED - the EKF has no position fix. The DVL "
                 "feeds it, so check the DVL is reachable and dvl_node and "
                 "nav_ekf_bridge are up. Drop --live to rehearse without it.")

    driver = Driver(profile, start_heading_deg=math.degrees(pose['yaw']))
    print(f"[INFO] hunting '{args.target}':")
    for name, spec in sorted(driver.profile.items()):
        print(f"       {name:<13} min {spec['min']:>8.3f}   "
              f"ideal {spec['ideal']:>8.3f}   max {spec['max']:>8.3f}")
    # The radar the mission draws on. Persistent, so the picture survives
    # between sweeps instead of blinking empty every time the head restarts.
    radar = Radar()
    if args.web:
        # The target is FIXED for a run. The box is disabled rather than hidden
        # so the page still shows what is being hunted - but changing it from a
        # browser while the sub is moving would leave the state machine on its
        # old heading with its old memory while the detector scored something
        # else, which is a good way to lose a vehicle.
        webview.set_target(args.target)
        webview.set_target_editable(False)
        webview.set_note("fixed for this run")
        webview.serve(args.port)
        print(f"[INFO] view on http://0.0.0.0:{args.port}")

    mode = "LIVE - SENDING SETPOINTS" if args.live else "DRY RUN - sending nothing"
    print(f"[INFO] {mode}")
    if args.live:
        print("[INFO] vehicle must already be in GUIDED and ARMED")
    elif pose['pos'] is None:
        print("[INFO] no position fix; heading only. The NED figures below are "
              "measured from an assumed origin and mean nothing in absolute "
              "terms - the state machine and the moves it picks are still real.")

    tuning = {"threshold": args.threshold}
    target = None
    last_send = 0.0

    # The last complete perception, so a partial frame can show the detections
    # the driver actually acted on rather than half-measured ones from a sweep
    # that is still in progress.
    shown = [None]

    def _blank(partial):
        """A Perception with no candidates, for the very first partial frame."""
        return Perception(partial, None, [], None, OK)

    for i in range(args.max_sweeps):
        if not refresh_pose(link, pose, need_pos=need_pos):
            print("[WARN] lost pose - holding")
            continue
        n, e, down = pose['pos'] or (0.0, 0.0, 0.0)
        yaw = pose['yaw']
        heading_deg = math.degrees(yaw) % 360.0

        state = driver.state

        # Publish while the head is still moving. A sweep is 9-25 s, so without
        # this the picture freezes for the whole of one and you cannot tell a
        # working sonar from a stalled one - which is exactly the question you
        # want answered while the sub is under way. The detections drawn are the
        # LAST complete sweep's, the ones the driver actually acted on.
        def live(partial):
            radar.update(partial)
            webview.publish(render(shown[0] or _blank(partial), driver.memory,
                                   state=f"{driver.state}  scanning",
                                   radar=radar, target=args.target))

        sweep = sonar.sweep_for_state(driver.sweep_state(), heading_deg,
                                      on_ping=live if args.web else None)
        radar.update(sweep)
        # driver.profile, not args.target: one resolved profile, so the state
        # machine cannot be steering towards something the detector never scored.
        per = perceive(sweep, target=driver.profile, tuning=tuning,
                       require_floor=not args.no_floor)
        action = driver.tick(per, heading_deg)

        print(f"\n[{i}] {state} -> {driver.state}  hdg {heading_deg:.0f}  "
              f"{len(per.candidates)} blobs")
        print(f"    {action.kind} "
              f"{'' if action.value is None else f'{action.value:+.2f}'}: {action.why}")

        shown[0] = per
        if args.web:
            webview.publish(render(per, driver.memory, state=driver.state,
                                   radar=radar, target=args.target))

        if action.kind == FINISHED:
            print("[INFO] driver reports finished")
            break

        # Stop BEFORE carrying out the action that would leave the state you
        # asked to stop at. --stop-at ORIENT should park the sub overhead, not
        # yaw 90 degrees first.
        if args.stop_at and driver.state == args.stop_at.upper():
            print(f"[INFO] reached {driver.state} - stopping as asked, "
                  f"without sending '{action.kind}'")
            break

        tgt = action_to_target(action, n, e, down, yaw)
        if tgt is None:
            continue
        tn, te, td, tyaw = tgt

        # The same change gate mission_base uses: an unchanged target re-sent too
        # often restarts ArduSub's trajectory planner and the sub crawls.
        changed = (target is None
                   or any(abs(a - b) > 0.01 for a, b in zip((tn, te, td), target[:3]))
                   or abs(tyaw - target[3]) > 0.02)
        target = (tn, te, td, tyaw)

        if not args.live:
            print(f"    [dry] NED=({tn:.2f},{te:.2f},{td:.2f}) "
                  f"yaw={math.degrees(tyaw):.0f}")
            continue
        if not changed and time.time() - last_send < RESEND_S:
            continue
        last_send = time.time()
        link.goto_ned(tn, te, td, tyaw)
        print(f"    sent NED=({tn:.2f},{te:.2f},{td:.2f}) "
              f"yaw={math.degrees(tyaw):.0f}")

    print("\n[INFO] done. Switch to POSHOLD in QGroundControl before disarming.")


if __name__ == "__main__":
    main()
