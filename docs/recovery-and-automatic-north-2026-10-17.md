# DVL recovery and automatic north alignment — 2026-10-17

Scope: `tether_toggle_beichen` only. Local software and isolated simulation work;
no Jetson access, deployment, live parameter writes, arming or physical mission.
Preserve unrelated `docs/nav_lgoic.md` and teammate sonar history.

## Decisions confirmed by the operator

- Navigate to a north/east position target using DVL-based displacement. Retain
  VN orientation to transform body-frame velocity; DVL does not replace yaw.
- Replace the GUI's instruction to manually drive for north alignment with a
  separately started, bounded surface maneuver. Open water is available for
  competition; respect pool dimensions during trials.
- Preserve calibration across brief DVL loss. Consider off-target recovery but
  do not implement a new off-target relocation/re-anchoring policy this session.

## Recovery implementation

`BridgeEstimator` now owns the dropout decision. Previously an invalid report
immediately made output unhealthy and a 0.5 s gap latched a fault. The supervisor
also independently required valid raw DVL on every tick.

The new states are TRACKING, COASTING, REACQUIRING and POSITION_UNCERTAIN.
Explicit invalid reports, gross speed outliers above 5 m/s, or silence over 0.25 s
start a gap. For a bounded gap,
propagate the last trusted **earth-frame** velocity, converting it back to body
velocity for ODOMETRY using the current VN attitude. A turn must not rotate an
old body velocity into fictitious new earth-frame motion. This is a prediction,
not a valid DVL sample or an inertial navigation EKF.

Require at least three consistent valid reports spanning 0.2 s to reacquire;
adjacent recovery velocities must agree within 0.2 m/s per axis. The gap is
integrated once; no second catch-up integration. Preserve north, pressure zero,
Cube origin, reset counter and mission reference. Do not restart the bridge.
Continue odometry during the bounded prediction with quality 40 rather than 80
and growing position covariance. Keep raw DVL validity visible. The supervisor
uses bridge health/uncertainty for an existing dive, while measured DVL remains
necessary for north alignment and pre-dive heading/reference checks.

Provisional envelope: at most 1 s, assumed velocity error 0.2 m/s and unobserved
acceleration 1 m/s². Missing-distance allowance is `0.2*t + 0.5*t²` per gap and
accumulates across gaps; reacquired velocity cannot erase unobserved distance.
At 1 s or accumulated 0.75 m, stop external output and retain the DVL position
fault. These are engineering assumptions, not experimentally proven physical
bounds or a claim of sub-metre accuracy. Repeated losses can still exhaust the
budget. Existing explicit surfaced GPS recovery remains available without VN
magnetic recalibration or north recapture, provided VN continuity is retained.
No automatic underwater relocation or old-mission restart was added.

Predicted samples never enter the time-matched history used to validate GPS or
north alignment. VN loss, Cube frame changes, maintenance interlocks and mission
abort conditions remain distinct; this work does not mask those failures.

## North alignment and GUI

`north_alignment.py` retains accepted samples across rejected observations, with
a 12 s lifetime and a maximum 4 s break. Missing intervals over 0.5 s count toward
neither accepted travel nor duration. Clock regression and inconsistent angles
still clear collection. Existing GPS qualification, tilt, speed agreement and
rotation checks remain: >=0.5 m/s, >=5 s accepted coverage, >=2.5 m travel, >=15
accepted samples. GPS course while stationary does not establish heading.

New pure controller `north_maneuver.py` and ROS executable `north_maneuver`:

1. Operator runs Prepare while stationary, killed/disarmed at the surface.
2. Wait for qualified GPS, valid DVL and fresh VN; no movement command yet.
3. Operator confirms a clear corridor plus stopping room, chooses 3–10 m travel
   limit (default 6 m), enables propulsion and arms in MANUAL using normal controls.
4. Click **Run automatic north alignment** on `/calibration`. The script never
   arms, changes flight mode, releases kill or resets a safety fault.
5. Hold initial **raw relative VN yaw**, not Cube compass or newly corrected yaw.
   Rate-limited manual forward control targets 0.65 m/s. Maximum command 800/1000,
   yaw command +/-150, maximum speed 1 m/s, movement timeout 20 s, total timeout
   30 s, maximum relative yaw excursion 20 degrees, rate limit 15 deg/s. Gains
   are simulation-tested only; wet commissioning is required. A short corridor
   can legitimately abort before collecting enough motion. No obstacle avoidance.
6. Sensor rejection commands neutral forward/yaw; allow up to 5 s for GPS/DVL
   requalification, while retaining bounded accepted alignment history. Count
   unknown motion against travel allowance. Kill/disarm, pilot mode change,
   stale control/VN/pressure, excessive depth/motion, timing or travel limit abort.
7. Once north is accepted, apply bounded forward/sideways DVL velocity braking (maximum +/-300/1000 command), and verify >=3 s with
   valid DVL <=0.05 m/s, yaw rate <=2 deg/s and qualified GPS speed <=0.2 m/s.
   GPS stationary course is ignored. This is **not geographic position hold**;
   a current can prevent settling, in which case the maneuver cannot certify it.
8. Operator disarms/engages kill. The existing Prepare job waits for maneuver
   COMPLETE and verifies Cube fusion before commissioning waypoint permission.

Cancel and System Stop request maneuver abort. Kill, disarm and pilot mode
takeover yield immediately; no mode restoration or re-arm. GUI operation lock,
child-process inventory and autonomy lease exclude simultaneous waypoint and
alignment missions. MAVProxy remains sole Cube serial owner; the maneuver uses
the existing mission UDP endpoint while exclusive with waypoint execution.
Keep pilot joystick neutral during alignment. Exclusive physical joystick input
arbitration and actual firmware pilot-input timeout have not been wet-tested.

`fused_nav_bridge.py` publishes raw VN yaw separately from aligned yaw. TIMESYNC
sampling is increased to 5 Hz without relaxing RTT, future/stale-fix or GPS dwell
checks. Isolated depth fixtures now populate synthetic pressure history so GUI
Prepare executes its actual stationary pressure gate; previously it could not.

## North/east waypoint mapping

Inspection of `waypoint_logic.py` and `waypoint_mission.py` confirmed the mission
already converts geographic target to local north/east/down and repeatedly sends
`Link.goto_ned` position setpoints. It does not implement a timed heading-plus-
distance drive. That architecture is retained. The Cube controls position using
its external-nav-aided estimate, while the bridge derives displacement from DVL
and VN orientation. Neither DVL nor Cube/VN agreement proves absolute accuracy.
The existing mission holds its captured yaw while translating; adding a separate
turn-first maneuver is outside this change.

## Off-target policy — consideration only, no new behavior implemented

- If position is healthy but the vehicle has not reached its north/east target,
  closed-loop control should recompute the remaining position error. Do not reset
  origin or define the current position as success.
- Underwater, ordinary DVL cannot independently discover an absolute position
  offset. Bottom lock returning gives velocity, not the displacement missed
  during loss. Grow uncertainty; stop mission travel when it is too large. A
  future policy might request supervised surfacing or independent visual/acoustic
  localization, accounting for overhead obstacles and available depth/attitude.
- At the surface, compare a time-qualified GPS window with the propagated pose
  and its uncertainty. Reject isolated jumps. A sustained disagreement needs
  explicit revalidation; neither GPS nor DVL is automatically ground truth.
  Preserve the geographic target through any accepted future re-anchor, report
  coordinate changes, and require a deliberate new mission decision rather than
  automatically continuing an old thrust command.
- Do not silently change north offset to explain a position discrepancy. Initial
  north accuracy and later VN drift remain separate problems. This session does
  not diagnose or cure the previously reported 110-degree heading error.

## Validation record

Regression results and final native SITL evidence are recorded below before commit.
Failed tests/runs are preserved and must not be described as successful wet tests.

- Initial command attempts failed because Windows `python` was unavailable,
  direct unittest module paths did not match discovery, and replacing PYTHONPATH
  hid ROS. Corrected commands preserve the ROS environment and use discovery.
- GUI tests required the added process globals in their mocked helper namespace.
- First GUI/native alignment run: Prepare rejected missing synthetic pressure
  history. Fixed the isolated pressure fixture path, retaining the live gate.
- Second native run: GPS timing rejection interrupted qualification; increased
  TIMESYNC measurement cadence without weakening admission limits.
- Third native run: forward control settled below the qualifying speed and
  correctly aborted at 6 m. Revised bounded speed control and reran. Failed
  evidence: `north-maneuver-552e0738`, `north-maneuver-268c313d`,
  `north-maneuver-97ede171` under ignored `sitl/logs/`.
- DVL native test `dvl-recovery-2c0beae6`: short loss automatically reacquired;
  prolonged loss latched; explicit surfaced recovery passed with preserved north.
- Python regression suite: 195 tests passed at intermediate checkpoint. Both
  Node GUI suites passed; final counts/results follow.

Final software regressions: **202 Python tests passed**, both Node GUI suites
passed, changed new Python entrypoints compile, and `git diff --check` passed.

Native GUI-integrated waypoint test `waypoint-388892ffec` passed all stages from
WAIT_SURFACE through COMPLETE with a 0.5 m dive, injected invalid DVL reports
during underwater transit, GPS outage underwater, resurface GPS recovery and
disarm. One dropout recovered automatically with the same reference. Final
simulated target error 0.153 m; retained missing-distance allowance 0.154 m.
This is simulator behavior, not a measured physical accuracy claim.

Native GUI automatic alignment `north-maneuver-6ef46c81` passed Prepare,
MOVE/SETTLE/COMPLETE and commission with a synthetic +40-degree VN yaw reference;
accepted offset -40 degrees, 27 samples, 5.006 m traveled within the 6 m limit.
A follow-up run `north-maneuver-6dfe9b54` exposed an overly short first-status
deadline during ROS CLI startup, before motion began. Split initial startup
grace (10 s, no motion before fresh interlocks) from active status freshness
(0.7 s). Retain completed verification while waiting for operator disarm/kill,
so child-process exit does not force a new calibration. Final rerun recorded in
the evidence summary.

All tests used localhost ROS domain 73 and disabled QGC forwarding. The native
alignment fixture synthesizes control leases from observed simulator arm state;
it does not validate the real kill wiring, RF/tether input arbitration, live
firmware input timeout, hydrodynamics or EMI. Wet commissioning of maneuver
gains, actual stopping room, dropout acceleration assumptions and heading
stability remains required. The new software has **not been deployed**.

Final native alignment rerun `north-maneuver-e691860d` passed with the final
bounded DVL braking controller: MOVE -> SETTLE -> COMPLETE -> GUI commission,
-40-degree accepted offset for the injected +40-degree reference, 27 samples,
5.080 m travel within the unchanged 6 m allowance. Settling includes bounded
forward/sideways velocity feedback and zero commands at completion. It does not
hold a GPS coordinate or claim exact position hold in current.
Additional failed runs were retained: `north-maneuver-46b22eb2` stopped on a
transient freshness failure (a later snapshot alone did not identify its exact
cause); aligned maneuver VN/bridge freshness with the existing 0.5 s bridge gate
and added explicit age/control diagnostics. `north-maneuver-2915f59e` reached the
travel limit while passively coasting during SETTLE, motivating DVL braking
rather than increasing clearance limits or bypassing the guard.

Committed evidence summary: `docs/evidence/recovery-north-2026-10-17.json`, including
all six failed alignment runs, successful recovery/alignment/waypoint runs, and
SHA-256 hashes of the navigation sources. Native and pure tests passed, but this
is not certification for unattended physical operation. Future deployment must
verify source AND installed Python/HTML versions; prior successful colcon builds
left stale installed artifacts. Deploy/restart only with new live authorization
and verified maintenance conditions. No deployment is authorized by this note.
