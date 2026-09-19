"""Verify the whole chain against the simulator. No sonar, no water.

Checks the geometry first (floor depth, height above floor, offset, and that
span really does respond to orientation), then runs the state machine to see
that it transitions sensibly.

Needs the package importable, either way round:
    source /root/robotx_ws/install/setup.bash   # on Graey
    PYTHONPATH=. python3 tools/sonar_check.py   # from a plain checkout
"""
import os
import sys

import cv2

from robotx_graey_2026.api.sonar import settings as S
from robotx_graey_2026.api.sonar.detect import ACROSS, ALONG, NO_FLOOR, perceive
from robotx_graey_2026.api.sonar.driver import APPROACH, Driver, FOLLOW, ORIENT, SEARCH
from robotx_graey_2026.api.sonar.floor import Floor
from robotx_graey_2026.api.sonar.simulate import FakeSonar, Scene, simulate_sweep
from robotx_graey_2026.api.sonar.viewer import render

FLOOR_Z = -4.0
PIPE_Z = -2.5           # 1.5 m above the floor
results = []


def check(name, got, want, tol):
    ok = got is not None and abs(got - want) <= tol
    shown = "None" if got is None else f"{got:.3f}"
    print(f"{'PASS' if ok else 'FAIL'}  {name}: got {shown}, want {want} (+/-{tol})")
    results.append(ok)
    return ok


def check_true(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  ' + detail) if detail else ''}")
    results.append(bool(ok))
    return ok


def sweep_of(scene, sub_pos=(0.0, 0.0, 0.0), heading=0.0, state="SEARCH"):
    cfg = S.SWEEP[state]
    return simulate_sweep(scene, sub_pos, heading, cfg["start_deg"], cfg["end_deg"],
                          cfg["step_deg"], cfg["max_range_m"], seed=3)


# ---------------------------------------------------------------- geometry
print("--- geometry ---")

# a pipe running fore-aft (north-south), 0.8 m to starboard
along = Scene([((0.8, -4.0, PIPE_Z), (0.8, 4.0, PIPE_Z))], floor_z=FLOOR_Z)
p = perceive(sweep_of(along), profile=S.PIPELINE_PROFILE)

check("floor depth", p.floor.depth_m if p.floor else None, 4.0, 0.25)
check_true("pipe detected", p.best is not None)
if p.best:
    check("height above floor", p.best.height_m, 1.5, 0.3)
    check("offset to starboard", p.best.offset_m, 0.8, 0.35)
    check_true("orientation reads along", p.best.orientation == ALONG,
               f"span {p.best.span_deg:.0f} deg -> {p.best.orientation}")

# the same pipe turned across the view (east-west)
across = Scene([((-3.0, 0.0, PIPE_Z), (3.0, 0.0, PIPE_Z))], floor_z=FLOOR_Z)
q = perceive(sweep_of(across), profile=S.PIPELINE_PROFILE)
check_true("across pipe detected", q.best is not None)
if q.best and p.best:
    check_true("span is wider across than along",
               q.best.span_deg > p.best.span_deg * 2,
               f"{p.best.span_deg:.0f} deg along vs {q.best.span_deg:.0f} deg across")
    check_true("orientation reads across", q.best.orientation == ACROSS)

# ------------------------------------------------------------ failure modes
print("\n--- failure modes ---")

empty = Scene([], floor_z=FLOOR_Z)
r = perceive(sweep_of(empty), profile=S.PIPELINE_PROFILE)
check_true("floor alone gives no candidates", r.best is None, f"reason: {r.reason}")

# nothing at all: no floor, no pipe
void = Scene([], floor_z=-40.0)
v = perceive(sweep_of(void), profile=S.PIPELINE_PROFILE)
check_true("no floor is reported as such", v.reason == NO_FLOOR, f"reason: {v.reason}")

# asserted floor, for the shallow pool where the bottom is inside the blind zone
w = perceive(sweep_of(along), profile=S.PIPELINE_PROFILE,
             floor=Floor.from_known_depth(4.0))
check_true("asserted floor still detects the pipe", w.best is not None)

# ------------------------------------------------------------ state machine
print("\n--- state machine ---")

scene = Scene.zigzag(start=(1.3, 0.0), z=PIPE_Z, floor_z=FLOOR_Z, first_heading_deg=90.0)
sonar = FakeSonar(scene, sub_pos=(0.0, 0.0, 0.0), heading_deg=0.0, seed=3)
driver = Driver(S.PIPELINE_PROFILE)

seen_states = []
for step in range(90):
    sweep = sonar.sweep_for_state(driver.sweep_state(), sonar.heading_deg)
    per = perceive(sweep, profile=S.PIPELINE_PROFILE)
    action = driver.tick(per, sonar.heading_deg)
    seen_states.append(driver.state)
    sonar.apply(action)
    if step < 10 or driver.state not in (SEARCH, APPROACH) or step % 6 == 0:
        print(f"  {step:2d} {driver.state:<7} hdg {sonar.heading_deg:6.1f}  {action}")
    if driver.state == FOLLOW and seen_states.count(FOLLOW) > 3:
        break

check_true("search ran", SEARCH in seen_states)
check_true("progressed past search",
           any(st in seen_states for st in (APPROACH, ORIENT, FOLLOW)),
           f"states: {sorted(set(seen_states))}")
check_true("reached follow", FOLLOW in seen_states,
           f"states: {sorted(set(seen_states))}")

# ------------------------------------------------- Action -> NED conversion
# The geometry that turns a Driver decision into a setpoint. Wrong signs here
# send the sub the opposite way from the one the detector asked for, and that
# is not something you want to discover in the water.
print("\n--- action to NED target ---")
try:
    import math as _m
    from tools.sonar_follow import MAX_STEP_M, action_to_target
    from robotx_graey_2026.api.sonar.driver import Action, FORWARD, HOLD, STRAFE, YAW_BY

    def near(got, want):
        return all(abs(a - b) <= 1e-6 for a, b in zip(got, want))

    check_true("forward, heading north -> +north",
               near(action_to_target(Action(FORWARD, 1.0), 0, 0, 2, 0.0), (1, 0, 2, 0)))
    check_true("strafe starboard, heading north -> +east",
               near(action_to_target(Action(STRAFE, 1.0), 0, 0, 2, 0.0), (0, 1, 2, 0)))
    check_true("strafe starboard, heading east -> -north",
               near(action_to_target(Action(STRAFE, 1.0), 0, 0, 2, _m.pi / 2),
                    (-1, 0, 2, _m.pi / 2)))
    check_true("yaw_by 90 turns without translating",
               near(action_to_target(Action(YAW_BY, 90.0), 5, 5, 2, 0.0),
                    (5, 5, 2, _m.pi / 2)))
    check_true("oversized step is clamped",
               near(action_to_target(Action(FORWARD, 5.0), 0, 0, 2, 0.0),
                    (MAX_STEP_M, 0, 2, 0)))
    check_true("depth is never altered",
               action_to_target(Action(HOLD), 0, 0, 7.5, 0.0)[2] == 7.5)
except ImportError as exc:
    print(f"SKIP  action->NED checks ({exc}) - pymavlink not installed")

# ---------------------------------------------------------------- rendering
print("\n--- rendering ---")
img = render(p, driver.memory, state=driver.state)
cv2.imwrite("check_view.png", img)
check_true("viewer rendered", img.size > 0, f"{img.shape[1]}x{img.shape[0]} -> check_view.png")

# The radar warps polar to cartesian, and OpenCV measures its polar angle the
# opposite way round from this package. Get that wrong and the whole picture is
# mirrored - which is invisible on a symmetric scene and ruinous on a real one,
# so it is worth pinning down rather than eyeballing.
import math as _mm

import numpy as _np

from robotx_graey_2026.api.sonar.detect import Perception as _Per
from robotx_graey_2026.api.sonar.sweep import Sweep as _Sw
from robotx_graey_2026.api.sonar.viewer import _radar as _rad


def _drawn_angle(target_deg):
    """Put a bright band at one angle, return the angle it was drawn at."""
    angles = [a * 2.0 for a in range(180)]

    def shot(deg):
        im = _np.zeros((len(angles), 400), _np.uint8)
        if deg is not None:
            for r, a in enumerate(angles):
                if abs((a - deg + 180) % 360 - 180) < 3:
                    im[r, 200:220] = 255
        return _rad(_Per(sweep=_Sw(im, angles, metres_per_bin=0.01), candidates=[],
                         best=None, floor=None, reason="OK"), 520).astype(int)

    diff = _np.abs(shot(target_deg) - shot(None)).sum(axis=2)
    y, x = _np.unravel_index(_np.argmax(diff), diff.shape)
    return _mm.degrees(_mm.atan2(260 - y, x - 260)) % 360


for _want in (270, 90, 0, 180):
    _got = _drawn_angle(_want)
    _err = abs((_got - _want + 180) % 360 - 180)
    check_true(f"radar draws {_want} deg where it belongs", _err < 6,
               f"drawn at {_got:.0f} deg")

# The picture must survive the start of the next sweep. A sweep is 15-25 s on
# Graey, so wiping at the end of each one leaves an almost empty disc on screen.
from robotx_graey_2026.api.sonar.viewer import Radar as _Radar

_angs = [a * 2.0 for a in range(180)]
_img = _np.zeros((180, 400), _np.uint8)
_img[:, 200:220] = 200                          # a ring of returns all the way round
_r = _Radar()
_r.update(_Sw(_img, _angs, 0.01))
_blank = _Per(sweep=None, candidates=[], best=None, floor=None, reason="OK")
_before = _rad(_blank, 520, _r).astype(int)
_r.update(_Sw(_np.zeros((20, 400), _np.uint8), _angs[:20], 0.01))   # next sweep, 40 deg in
_after = _rad(_blank, 520, _r).astype(int)
_changed = (_np.abs(_before - _after).sum(axis=2) > 30).mean()
check_true("radar keeps the old sweep while the next one starts", _changed < 0.2,
           f"{_changed:.0%} of the image changed after 40 deg of new data")
_r.update(_Sw(_np.zeros((180, 200), _np.uint8), _angs, 0.02))
check_true("radar resets when the range changes", _r.polar.shape[1] == 200)

# ------------------------------------------------------------- object library
print("\n--- object library ---")
import tempfile as _tf

from robotx_graey_2026.api.sonar import library as _lib
from robotx_graey_2026.api.sonar.detect import Detection as _Det


def _det(bright, height, thick, solid=0.8, rng=1.3, span=16.0):
    return _Det(range_m=rng, angle_deg=0.0, offset_m=rng, height_m=height,
                span_deg=span, brightness=bright, solidity=solid,
                contour=None, thickness_m=thick)


# real PVC numbers from the pool, 16 Sep 2026, plus a plausible wall
_pipe = [_lib.sample_from(_det(b, 1.5, 0.09))
         for b in (146, 140, 134, 132, 143, 141)]
_wall = [_lib.sample_from(_det(b, 0.05, 0.45))
         for b in (149, 144, 152, 138, 141)]

_L = {}
_lib.add_samples(_L, "pvc_pipe", _pipe, "pool, 1.3 m off starboard")
_lib.add_samples(_L, "pool_wall", _wall, "pool wall")

_prof = _lib.profile_for(_L, "pvc_pipe")
check_true("profile centres on the median, not a stray sweep",
           139 <= _prof["brightness"]["ideal"] <= 142,
           f"ideal {_prof['brightness']['ideal']}")
check_true("profile edges sit outside the samples seen",
           _prof["brightness"]["min"] < 132 and _prof["brightness"]["max"] > 146)
check_true("span is recorded but kept out of identity",
           "span_deg" in _pipe[0] and "span_deg" not in _prof)

_name, _sc, _ = _lib.identify(_det(141, 1.5, 0.09), _L)
check_true("identifies pipe as pipe", _name == "pvc_pipe", f"{_name} {_sc:.2f}")
_name, _sc, _ = _lib.identify(_det(145, 0.05, 0.45), _L)
check_true("identifies wall as wall", _name == "pool_wall", f"{_name} {_sc:.2f}")

# brightness alone cannot separate them - that is the whole point of the pool
# result. thickness and height are what do the work.
_bright_only = {k: {"pvc_pipe": {"profile": {"brightness": _prof["brightness"]}},
                    "pool_wall": {"profile": {"brightness":
                                  _lib.profile_for(_L, "pool_wall")["brightness"]}}}[k]
                for k in ("pvc_pipe", "pool_wall")}
_n1, _s1, _ = _lib.identify(_det(141, 1.5, 0.09), _bright_only)
_n2, _s2, _ = _lib.identify(_det(141, 0.05, 0.45), _bright_only)
check_true("on brightness alone the two are NOT separable", _n1 == _n2,
           f"both score as {_n1}")

with _tf.NamedTemporaryFile(suffix=".json", delete=False) as _fh:
    _path = _fh.name
_lib.save(_L, _path)
check_true("library survives a save and load", _lib.load(_path) == _L)
os.unlink(_path)

print(f"\n{sum(results)}/{len(results)} checks passed")
sys.exit(0 if all(results) else 1)
