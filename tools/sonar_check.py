"""Verify the whole chain against the simulator. No sonar, no water.

Checks the geometry first (floor depth, height above floor, offset, and that
span really does respond to orientation), then runs the state machine to see
that it transitions sensibly.

Needs the package importable, either way round:
    source /root/robotx_ws/install/setup.bash   # on Graey
    PYTHONPATH=. python3 tools/sonar_check.py   # from a plain checkout
"""
import json as _json
import os
import sys
import time as _time

import cv2

from robotx_graey_2026.api.sonar import library as _lib
from robotx_graey_2026.api.sonar import settings as S
from robotx_graey_2026.api.sonar.detect import ACROSS, ALONG, NO_FLOOR, perceive
from robotx_graey_2026.api.sonar.driver import (APPROACH, Driver, FOLLOW, ORIENT,
                                                SEARCH, YAW_BY)
from robotx_graey_2026.api.sonar.floor import Floor
from robotx_graey_2026.api.sonar.simulate import FakeSonar, Scene, simulate_sweep
from robotx_graey_2026.api.sonar.viewer import render

FLOOR_Z = -4.0
PIPE_Z = -2.5           # 1.5 m above the floor

# The object these checks hunt. A test owns its fixtures. This used to be
# imported from settings.py, which is exactly the thing that is now gone: the
# sonar package no longer ships with any particular object baked into it, so a
# test that wants one has to say so.
TARGET = {
    "height_m":   {"min": 0.4, "ideal": 1.5, "max": 2.8},
    "brightness": {"min": 70,  "ideal": 140, "max": 255},
}

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
p = perceive(sweep_of(along), target=TARGET)

check("floor depth", p.floor.depth_m if p.floor else None, 4.0, 0.25)
check_true("pipe detected", p.best is not None)
if p.best:
    check("height above floor", p.best.height_m, 1.5, 0.3)
    check("offset to starboard", p.best.offset_m, 0.8, 0.35)
    check_true("orientation reads along", p.best.orientation == ALONG,
               f"span {p.best.span_deg:.0f} deg -> {p.best.orientation}")

# the same pipe turned across the view (east-west)
across = Scene([((-3.0, 0.0, PIPE_Z), (3.0, 0.0, PIPE_Z))], floor_z=FLOOR_Z)
q = perceive(sweep_of(across), target=TARGET)
check_true("across pipe detected", q.best is not None)
if q.best and p.best:
    check_true("span is wider across than along",
               q.best.span_deg > p.best.span_deg * 2,
               f"{p.best.span_deg:.0f} deg along vs {q.best.span_deg:.0f} deg across")
    check_true("orientation reads across", q.best.orientation == ACROSS)

# ------------------------------------------------------------ failure modes
print("\n--- failure modes ---")

empty = Scene([], floor_z=FLOOR_Z)
r = perceive(sweep_of(empty), target=TARGET)
check_true("floor alone gives no candidates", r.best is None, f"reason: {r.reason}")

# nothing at all: no floor, no pipe
void = Scene([], floor_z=-40.0)
v = perceive(sweep_of(void), target=TARGET)
check_true("no floor is reported as such", v.reason == NO_FLOOR, f"reason: {v.reason}")

# asserted floor, for the shallow pool where the bottom is inside the blind zone
w = perceive(sweep_of(along), target=TARGET,
             floor=Floor.from_known_depth(4.0))
check_true("asserted floor still detects the pipe", w.best is not None)

# ------------------------------------------------------------ state machine
print("\n--- state machine ---")

scene = Scene.zigzag(start=(1.3, 0.0), z=PIPE_Z, floor_z=FLOOR_Z, first_heading_deg=90.0)
sonar = FakeSonar(scene, sub_pos=(0.0, 0.0, 0.0), heading_deg=0.0, seed=3)
driver = Driver(TARGET)

seen_states = []
for step in range(90):
    sweep = sonar.sweep_for_state(driver.sweep_state(), sonar.heading_deg)
    per = perceive(sweep, target=TARGET)
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

# --stop-at parks the sub at a state instead of running the whole mission.
# It has to stop BEFORE the action that leaves that state, or "stop when
# you are overhead" yaws 90 degrees and then stops.
_d2 = Driver(TARGET)
_sonar2 = FakeSonar(Scene.zigzag(start=(1.3, 0.0), z=PIPE_Z, floor_z=FLOOR_Z,
                                 first_heading_deg=90.0),
                    sub_pos=(0.0, 0.0, 0.0), heading_deg=0.0, seed=3)
_sent = []
for _ in range(90):
    _sweep2 = _sonar2.sweep_for_state(_d2.sweep_state(), _sonar2.heading_deg)
    _act = _d2.tick(perceive(_sweep2, target=TARGET), _sonar2.heading_deg)
    if _d2.state == ORIENT:
        break
    _sent.append(_act.kind)
    _sonar2.apply(_act)
check_true("--stop-at ORIENT stops before the turn that leaves APPROACH",
           _d2.state == ORIENT and YAW_BY not in _sent,
           f"state {_d2.state}, sent {sorted(set(_sent))}")

# ------------------------------------------------- Action -> NED conversion
# The geometry that turns a Driver decision into a setpoint. Wrong signs here
# send the sub the opposite way from the one the detector asked for, and that
# is not something you want to discover in the water.
print("\n--- action to NED target ---")
try:
    import math as _m
    from tools.sonar_follow import MAX_STEP_M, action_to_target
    from robotx_graey_2026.api.sonar.driver import Action, FORWARD, HOLD, STRAFE

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

# ----------------------------------------------------------- target resolution
# One function decides what a target argument means, so the viewer, the state
# machine and every tool agree. If these drift apart the radar can show rings
# for one object while the sub swims after another.
print("\n--- target resolution ---")

check_true("no target resolves to nothing", _lib.resolve(None) is None
           and _lib.resolve("") is None and _lib.resolve({}) is None)
check_true("a profile dict passes straight through",
           _lib.resolve(TARGET) is TARGET)
check_true("a name resolves out of the library",
           _lib.resolve("pvc_pipe", path=_path) == _prof)

_typed = _lib.resolve("height_m=1.5,brightness=140", tolerance=0.5)
check_true("typed ideals become a profile",
           _typed["height_m"]["ideal"] == 1.5 and _typed["brightness"]["ideal"] == 140)
check_true("typed ideals get edges either side",
           _typed["height_m"]["min"] == 0.75 and _typed["height_m"]["max"] == 2.25,
           f"{_typed['height_m']}")
check_true("tolerance is what sets how fussy it is",
           _lib.resolve("height_m=1.5", tolerance=0.1)["height_m"]["max"] == 1.65)
check_true("ideals are clamped to what a feature can physically be",
           _lib.resolve("brightness=200,solidity=0.9")["brightness"]["max"] == 255
           and _lib.resolve("solidity=0.9")["solidity"]["max"] == 1.0)

for _bad, _why in (("height_m", "no equals sign"), ("nonsense=3", "unknown feature"),
                   ("height_m=tall", "not a number")):
    try:
        _lib.resolve(_bad)
        check_true(f"a bad target is rejected ({_why})", False)
    except (ValueError, KeyError):
        check_true(f"a bad target is rejected ({_why})", True)

try:
    _lib.resolve("no_such_object", path=_path)
    check_true("an unknown name names what IS in the library", False)
except KeyError as _exc:
    check_true("an unknown name names what IS in the library",
               "pvc_pipe" in str(_exc), str(_exc))

try:
    Driver(None)
    check_true("a Driver refuses to run with no target", False)
except ValueError:
    check_true("a Driver refuses to run with no target", True)
check_true("a Driver resolves a name into a profile",
           Driver("pvc_pipe", library_path=_path).profile == _prof)
os.unlink(_path)

# ------------------------------------------------------- rings follow the score
# The radar must not draw a ring for a candidate nobody asked about, and a ring
# it does draw has to darken with confidence. Both are measured off the pixels
# rather than read off the code, because that is the part that can rot.
print("\n--- rings follow the score ---")

_scene = Scene([((0.8, -4.0, PIPE_Z), (0.8, 4.0, PIPE_Z))], floor_z=FLOOR_Z)
_blind = perceive(sweep_of(_scene))                  # no target at all
_judged = perceive(sweep_of(_scene), target=TARGET)
check_true("without a target, candidates are still measured",
           len(_blind.candidates) > 0 and _blind.candidates[0].score is None,
           f"{len(_blind.candidates)} candidates, all unscored")
check_true("without a target, best is left empty", _blind.best is None)

# Counting black pixels will not do: the range rings, the floor line and the
# blind-zone dashes are black furniture and swamp any ring. Compare against the
# SAME radar with the candidate list emptied, so the only pixels that can differ
# are the ones a candidate put there.
_bare = _Per(sweep=_blind.sweep, floor=_blind.floor, candidates=[], best=None,
             reason=_blind.reason)
_bare_img = _rad(_bare, 520)
check_true("without a target a candidate draws nothing at all",
           _np.array_equal(_rad(_blind, 520), _bare_img))
check_true("with a target it does draw something",
           not _np.array_equal(_rad(_judged, 520), _bare_img))

from robotx_graey_2026.api.sonar.viewer import confidence_colour as _conf
check_true("a sure match is shaded darker than a poor one",
           _conf(1.0)[0] < _conf(0.5)[0] < _conf(0.0)[0],
           f"{_conf(1.0)[0]} < {_conf(0.5)[0]} < {_conf(0.0)[0]}")
check_true("confidence shading is clamped, not wrapped",
           _conf(5.0) == _conf(1.0) and _conf(-2.0) == _conf(0.0))


# and the same thing end to end, on the pixels themselves rather than on the
# colour function: over exactly the pixels the two renders disagree about, the
# sure-match render must be the darker one.
def _canvas(score):
    _p2 = perceive(sweep_of(_scene), target=TARGET)
    for _d in _p2.candidates:
        _d.score = score
    return _rad(_p2, 520).astype(int)


# ------------------------------------------------- scoring and confidence
print("\n--- scoring and confidence ---")
from robotx_graey_2026.api.sonar.detect import rescore as _rescore, score as _score

# A percentage has to mean the same thing however many questions you asked.
# The old plain product did not: four features at 0.9 came out as 0.66.
_two = {"brightness": {"min": 0, "ideal": 100, "max": 200},
        "solidity":   {"min": 0.0, "ideal": 0.8, "max": 1.0}}
_four = dict(_two, height_m={"min": 0.0, "ideal": 2.0, "max": 4.0},
             thickness_m={"min": 0.0, "ideal": 0.2, "max": 0.4})
_ideal = _det(100, 2.0, 0.2, 0.8)
check_true("a perfect match is 100% however many features are asked",
           _score(_ideal, _two)[1] == 1.0 and _score(_ideal, _four)[1] == 1.0)

# halfway along every taper: should read the same with two features or four
_half = _det(50, 1.0, 0.1, 0.4)
_s2, _s4 = _score(_half, _two)[1], _score(_half, _four)[1]
check_true("asking more questions does not lower the score on its own",
           abs(_s2 - _s4) < 0.02 and 0.45 < _s2 < 0.55, f"{_s2:.2f} vs {_s4:.2f}")

# but one outright failure still has to veto, whatever the rest say
_veto = _det(100, 2.0, 0.2, 1.5)          # solidity outside its range
check_true("one failed feature still kills the whole score",
           _score(_veto, _four)[1] == 0.0)

_d1 = _det(141, 1.5, 0.09)
_d1.scores, _d1.score = _score(_d1, _lib.profile_for(_L, "pvc_pipe"))
check_true("confidence is the score as a whole percent",
           _d1.confidence == int(round(_d1.score * 100)), f"{_d1.confidence}%")
check_true("unscored means unscored, not zero", _det(141, 1.5, 0.09).confidence is None)

# a profile asking for height in water with no floor: the feature is dropped
# rather than failed, and the drop is recorded so the viewer can say so
_noheight = _det(141, None, 0.09)
_per_f, _tot = _score(_noheight, _lib.profile_for(_L, "pvc_pipe"))
check_true("a feature this sweep cannot measure is skipped, not failed",
           "height_m" not in _per_f and _tot > 0.0, f"scored on {sorted(_per_f)}")

_p3 = perceive(sweep_of(along), target=TARGET)
check_true("what was dropped is recorded for the viewer to show",
           _p3.best is not None and _p3.best.unscored == (),
           f"unscored: {_p3.best.unscored if _p3.best else '-'}")

# rescore: a new target without a new sweep. This is what makes the target box
# answer at once, since a sweep is 15-25 s and scoring is microseconds.
_live = perceive(sweep_of(along), target=TARGET)
check_true("rescore to an impossible target drops the match",
           _rescore(_live, "brightness=250").best is None)
check_true("rescore back to a good target brings it back",
           _rescore(_live, TARGET).best is not None)
check_true("rescore to no target clears every score",
           _rescore(_live, None).best is None
           and all(d.score is None for d in _live.candidates))

# ------------------------------------------- outlines follow the real size
# A detection is not a point, and drawing a pipeline seen broadside as the same
# little circle as a pole throws away the thing that separates them.
print("\n--- outlines follow the real size ---")
from robotx_graey_2026.api.sonar.viewer import _outline as _out


def _extent(per):
    """Biggest on-screen dimension of the best candidate's drawn outline."""
    size = 520
    centre = size // 2
    radius = centre - int(size * 0.065)
    ppm = radius / (per.sweep.image.shape[1] * per.sweep.metres_per_bin)
    poly = _out(per, per.best, centre, ppm)
    return max(_np.ptp(poly[:, 0]), _np.ptp(poly[:, 1]))


def _predicted(per):
    """How long that blob ought to be drawn, from its own measurements.

    An arc of `span` degrees at `range` metres has a chord of 2*r*sin(span/2),
    and a blob is at least as wide as it is radially thick. Getting the pixels
    per metre wrong, or dropping the range, shows up here and nowhere else.
    """
    size, d = 520, per.best
    ppm = (size // 2 - int(size * 0.065)) / (per.sweep.image.shape[1]
                                             * per.sweep.metres_per_bin)
    return max(2 * d.range_m * _mm.sin(_mm.radians(d.span_deg) / 2),
               d.thickness_m) * ppm


check_true("a pipe across the view is drawn much longer than one end-on",
           _extent(q) > _extent(p) * 2,
           f"{_extent(p)} px along vs {_extent(q)} px across")
for _per, _nm in ((p, "end-on"), (q, "broadside")):
    _ratio = _extent(_per) / _predicted(_per)
    check_true(f"the {_nm} outline is the size the geometry says it should be",
               0.7 < _ratio < 1.4,
               f"drawn {_extent(_per)} px, predicted {_predicted(_per):.0f} px")

_sure_img, _weak_img = _canvas(1.0), _canvas(0.05)
_ring = _np.abs(_sure_img - _weak_img).sum(axis=2) > 30
check_true("the shading reaches the canvas, not just the colour function",
           _ring.any() and _sure_img[_ring].mean() < _weak_img[_ring].mean(),
           f"sure {_sure_img[_ring].mean():.0f} vs weak {_weak_img[_ring].mean():.0f} "
           f"over {int(_ring.sum())} px")

# --------------------------------------------------------- the target box
# The one setting the radar page carries. It is the only way to change what is
# scored without restarting, so it is worth a real socket rather than a mock.
print("\n--- the target box ---")
try:
    import urllib.request as _url

    from robotx_graey_2026.api.sonar import webview as _wv

    _wv.set_target("height_m=1.5")
    _wv.serve(8099)
    _time.sleep(0.5)
    _base = "http://127.0.0.1:8099"

    check_true("the page carries a target box",
               b'id="t"' in _url.urlopen(_base + "/").read())
    check_true("the box starts on whatever --target gave it",
               _json.loads(_url.urlopen(_base + "/state").read())["target"] == "height_m=1.5")

    _url.urlopen(_url.Request(_base + "/target", data=b"  brightness=140  ",
                              method="POST"))
    check_true("typing a target reaches the tool", _wv.target() == "brightness=140")
    check_true("and it resolves to a profile the detector can use",
               _lib.resolve(_wv.target())["brightness"]["ideal"] == 140)

    _url.urlopen(_url.Request(_base + "/target", data=b"", method="POST"))
    check_true("emptying the box means no target at all",
               _wv.target() == "" and _lib.resolve(_wv.target() or None) is None)

    # it listens on 0.0.0.0, so anything on the tether network can post to it
    _url.urlopen(_url.Request(_base + "/target", data=b"x" * 5000, method="POST"))
    check_true("an oversized body is capped, not swallowed whole",
               len(_wv.target()) == 1024)

    # A tool that moves the sub locks the box. The lock has to live on the
    # SERVER: a disabled input is only a suggestion to a browser, and this port
    # is reachable from anything on the tether network.
    _wv.set_target("pvc_pipe")
    _wv.set_target_editable(False)
    _url.urlopen(_url.Request(_base + "/target", data=b"brightness=1", method="POST"))
    check_true("a locked target refuses a POST, not just greys out the box",
               _wv.target() == "pvc_pipe")
    check_true("the page is told the target is locked",
               _json.loads(_url.urlopen(_base + "/state").read())["editable"] is False)
    _wv.set_target_editable(True)
    _url.urlopen(_url.Request(_base + "/target", data=b"brightness=1", method="POST"))
    check_true("unlocking lets it through again", _wv.target() == "brightness=1")
except OSError as _exc:
    print(f"SKIP  target box checks ({_exc}) - port 8099 busy?")

print(f"\n{sum(results)}/{len(results)} checks passed")
sys.exit(0 if all(results) else 1)
