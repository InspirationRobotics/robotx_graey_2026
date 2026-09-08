"""Verify the whole chain against the simulator. No sonar, no water.

Checks the geometry first (floor depth, height above floor, offset, and that
span really does respond to orientation), then runs the state machine to see
that it transitions sensibly.

Needs the package importable, either way round:
    source /root/robotx_ws/install/setup.bash   # on Graey
    PYTHONPATH=. python3 tools/sonar_check.py   # from a plain checkout
"""
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

# ---------------------------------------------------------------- rendering
print("\n--- rendering ---")
img = render(p, driver.memory, state=driver.state)
cv2.imwrite("check_view.png", img)
check_true("viewer rendered", img.size > 0, f"{img.shape[1]}x{img.shape[0]} -> check_view.png")

print(f"\n{sum(results)}/{len(results)} checks passed")
sys.exit(0 if all(results) else 1)
