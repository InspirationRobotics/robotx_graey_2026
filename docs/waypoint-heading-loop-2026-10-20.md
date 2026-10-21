# Surface waypoint heading loop — 2026-10-20

Scope: `tether_toggle_beichen`, local development only. No deployment, vehicle
parameters, compass calibration profile, main branch, or kill/failsafe changes.
User authorized waypoint implementation after the earlier polling-only fix.

## Runtime sequence

1. The existing GUI target becomes a frozen geographic waypoint. Existing
   commissioning, sensor/reference health, pressure depth, vertical profile and
   control permission checks remain in `waypoint_logic.py` / `waypoint_mission.py`.
2. The existing five accepted seconds of surface validation run in position hold.
   GPS must be admitted; the target is converted to Cube local north/east once.
3. `SurfaceLeg` in `surface_control.py` turns toward a lookahead point on the
   start-to-target line. Yaw commands slew at at most 15 degrees/second. Forward
   travel waits for heading error <=15 degrees continuously for 0.7 seconds.
   Error >40 degrees returns to alignment. Alignment uses velocity damping;
   it is not a separate absolute position lock.
4. Travel uses north/east position error, cross-track correction and measured
   Cube velocity. Cube receives bridge-aided GPS/DVL/VN odometry. A 1.5 m
   lookahead guides heading, cross-track gain is 0.18/s, velocity damping 0.35,
   and horizontal speed is capped at 0.20 m/s. Near or beyond the endpoint,
   the point controller brakes/returns rather than stalling after overshoot.
5. In parallel, `BridgeEstimator.correct_motion()` calls `CourseHeading` only
   for GPS fixes that already passed freshness, pressure, admission, residual
   and time-alignment gates. It compares actual GPS motion against actual DVL
   velocity rotated by VN. Desired waypoint bearing is never a measurement.
6. Good GPS corrects bridge position through the existing bounded correction
   path. Missing/rejected GPS uses the existing bounded DVL/VN continuation:
   at most 10 seconds or 3 m before pilot takeover. Arrival still requires
   current admitted GPS. Existing DVL recovery and takeover logic are retained.
7. The GUI shows alignment/tracking, cross-track error, and the reason north
   correction is waiting or active. Trace logs include those same states.

## Heading evidence conditions

- Requires a previously validated north reference; this is not initial alignment.
- Measured horizontal motion >=0.08 m/s, yaw rate <=3 degrees/s, roll/pitch
  rates <=10 degrees/s, and vertical speed <=0.15 m/s.
- Doppler mode: GPS speed >=0.08 m/s; reported speed uncertainty implies <=12
  degrees of direction uncertainty; GPS/DVL speeds agree within the larger of
  0.06 m/s or three speed uncertainties. At least eight samples span three
  seconds; direction spread must stay within five degrees.
- Position-baseline alternative: at least five seconds, >=2 m GPS displacement,
  >=1 m DVL displacement, endpoint uncertainty implies <=12 degrees, path
  straightness >=95%, and travel distances agree within the larger of 0.3 m or
  twice combined endpoint uncertainty. A short pool leg may not qualify.
- GPS/DVL direction disagreement >25 degrees is rejected for review. Accepted
  changes are <=0.5 degrees, separated by >=3 seconds, with accumulated learned
  correction bounded to +/-30 degrees. Initial reference is retained across
  rejected GPS, stationary periods, turns, submergence and DVL gaps.
- GPS course is not nose heading: lateral and reverse DVL motion are supported.
  The same evidence no longer simultaneously learns DVL velocity bias, which
  could confuse a yaw error with a lateral sensor bias.
- No averaging claim removes systematic/correlated GPS error. GPS reported
  accuracy is a gate, not proof of true geographic accuracy.

## Surface vertical control

Unchanged: zero down velocity inside the 3 cm pressure deadband, bounded upward
velocity outside it, never a positive/downward surface velocity command. Pressure
depth >0.08 m still aborts. A velocity setpoint is closed-loop control, not neutral
thruster output; this change does not prove the real vehicle cannot rise or sink.

## Verification

- 289 Python regressions pass, including closed-loop pure-controller legs with
  opposite initial headings, cross-current and endpoint overshoot; slow heading
  correction, lateral/reverse motion, uncertainty/turn/rejection gates and GPS
  interruption tests. Both waypoint and calibration JavaScript suites pass.
- Native ArduSub + ROS + GUI API run `waypoint-2cd1f939ca` completed with a two
  second injected GPS outage; final Cube target error 0.104 m. The short leg
  did not accumulate qualifying heading-correction evidence (zero updates).
  Heading convergence is demonstrated by synthetic measured-motion tests,
  not by this native run or by a real vehicle test.
- Earlier run `waypoint-d6007e830d` aborted at the 0.08 m surface-depth guard
  before transit (pressure depth 0.09 m). No guard was weakened. An older pass
  `waypoint-cd2fb13511` used stale build imports and does not validate this code.
- SITL launcher now verifies current source imports and refreshes installed GUI
  HTML. QGC forwarding remains disabled unless explicitly requested separately.
- A targeted test invocation using `test.test_*` failed import resolution; full
  discovery is the working invocation and passed afterward.
- The added live guidance GUI fixture exposed a formatter scope error that
  would have interrupted rendering once mission status arrived. The formatter
  is now defined in render scope; both GUI suites pass with active guidance.

- Native final-code run `waypoint-f2879e1a3a` passed the expected-failure test:
  a 12-second injected GPS outage caused the existing bounded-continuation abort,
  and Cube confirmed MANUAL takeover. No unintended dive stage occurred.

Physical heading accuracy, EMI, propulsion behavior and real sensor recovery
still need controlled commissioning with Graey available. Before deployment,
retain the existing external-navigation profile and a verified initial north
reference; this waypoint commit does not finish the deferred temporary GPS
Compass Learn calibration workflow.
