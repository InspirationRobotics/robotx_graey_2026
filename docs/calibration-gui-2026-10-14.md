# Calibration GUI and remaining live checks — October 14, 2026

The new `/calibration` page is reachable from the main GUI's **Calibrate / Prepare**
button and the Waypoint page. This is a standalone page with detailed instructions,
two confirmations, asynchronous progress, coverage counts, pressure baseline/depth,
north-alignment status, odometry status, Cube/supervisor evidence, log path and Cancel.
No deployment or real-vehicle enablement has occurred in this offline session.

## Sensor authority

User requests VN-100 as the primary external attitude source. The bridge uses VN
orientation/angular rates, DVL body velocity, pressure depth and qualified surface
GPS; it does not substitute Cube yaw as a north measurement. Cube EKF receives the
combined odometry and uses its own inertial measurements for estimation/control.
Cube/VN agreement is a downstream consistency check, not an independent compass
calibration. No speculative covariance/weighting change was made. Actual VN mounting,
magnetic performance and uncertainty must be validated before tightening weights.

## Calibrate VN button

1. Keep the assembled vehicle powered, disarmed, killed and ESC-off, with VN mounted.
   Make space/tether slack to rotate safely away from external metal/interference.
2. Read and tick the calibration confirmation. Press **Calibrate VN** and wait for
   the confirmed live instruction to start rotation.
3. The GUI suspends supervisor permission, pauses odometry, and invokes the existing
   VN reader's calibration service. It never stops the GUI/core or starts another
   VN process. The reader backs up register values, uses relative heading and slow
   onboard HSI collection, withholds IMU/heading publication during collection, and
   records six-face orientation coverage.
4. Rotate yaw/pitch/roll for120s, covering upright/inverted, both sides and nose-up/down.
   Keep kill/disarm/ESC-off throughout. Every face needs >=5 samples; this is coverage,
   not a certificate of fit or heading accuracy.
5. The reader freezes learning and checks readback. Failure/incomplete coverage blocks
   attitude publication until calibration is repeated. Calibration remains volatile;
   no flash save or fixed compass offset is performed. Cancel stops learning and
   leaves the run unaccepted. Lost serial/control or failed freeze ACK is reported.

On serial reconnection the reader freezes any interrupted HSI run, retaining relative
heading. A hard process/vehicle interruption still requires checking the sensor on
reconnection; no automatic readiness is inferred from a restart.

## Prepare Navigation button

1. Float stationary at the water surface, killed/disarmed with ESC power off. Wait
   several seconds, then tick the explicit floating-surface confirmation. Never use
   that confirmation on land or underwater.
2. Press **Prepare Navigation**. Existing mission permission is suspended, old mission
   reference/preview display session invalidated and odometry paused. Fresh stable
   pressure history establishes the session baseline (>=15 samples spanning1.5s,
   newest <=0.35s, <=8hPa range in the last3s). The external-only Cube profile must match.
3. The bridge resets its estimator before output, preserving geographic origin and
   incrementing the ODOMETRY reset counter. Output is permitted only after the new
   GPS/DVL north alignment and all existing health/profile checks succeed. This is
   an explicit killed/disarmed revalidation, not automatic recovery mid-mission.
4. Follow live instructions: after GPS/DVL/VN are available, deliberately use normal
   pilot controls for a steady, level MANUAL surface movement. Required accepted
   motion >=0.5m/s for >=5s and >=2.5m, with valid DVL bottom lock and exposed GPS.
   Allow additional travel/stopping clearance. Do not blindly use these minimums as
   a guarantee of sufficient room. The button never releases kill, enables ESCs,
   arms, selects a mode or drives this movement.
5. When north aligns, stop, disarm and engage kill again. The preparation job waits
   for Cube health, local position, aligned transmitting odometry, heading consistency,
   qualified surface GPS and near-zero depth. The checked commissioning service then
   enables supervisor/session waypoint permission. No bridge restart is needed.
6. Preparation expires after five minutes if incomplete. Cancellation/timeouts do
   not grant GUI waypoint permission. During manual movement Cancel does not drive,
   stop thrusters or change mode: the pilot remains responsible for controls/kill.

The page does not manufacture a successful heading from a phone or GPS position.
GPS and DVL velocity vectors establish an estimated north reference. Alignment is
session-only; underwater propagation retains that reference and rejects GPS using
the existing pressure gate. Mounting/body axes and velocity quality remain prerequisites.

## What must still be checked on Graey

- Deploy/build the toggle branch, preserving Jetson-only sonar work; verify one VN,
  one DVL and one fused bridge, the external-only Cube profile, relative mode and
  zero unreviewed yaw offsets. Main is not part of this deployment.
- Bench-test GUI Calibrate start/progress/Cancel/safety-loss and freeze readback with
  real serial hardware. Verify no odometry is transmitted during calibration and
  that failed coverage cannot be presented as completed preparation.
- Verify VN/DVL mounting and forward/right/down axes, including pitch/roll signs and
  rotation response. GPS north alignment cannot certify physical sensor-axis correctness.
- At the real surface, verify pressure zero and bottom lock, GPS location against
  physical surroundings, fix timing and GPS velocity uncertainty. Prior live readings
  included approximately0.39m/s speed uncertainty; current alignment screens require
  <=0.2m/s and a direction-uncertainty screen <=10deg. This may block the earlier
  conditions. Check fresh data and antenna/environment before considering threshold
  changes; do not claim synthetic GPS quality represents the receiver in the pool.
- Perform the manual surface alignment with adequate clearance. Return to kill/disarm;
  check stable Cube position/depth/heading and the positive commissioning-service path.
- Perform a short disarmed submerge/surface continuity test: GPS rejected when wet,
  DVL/VN/Cube pose remain healthy, surface depth recovers, no unexpected frame reset.
- Verify target registration/clearance and controller connection failsafe selection,
  deliberate ESC permission/recovery, pilot takeover and Abort. Then run a short,
  supervised0.5m dive/transit/surface mission while recording Cube BIN and Jetson logs.
  Approximate pool-image live targets remain blocked; use a verified geographic target.
- Independent mission-process-death protection and quantitative drift/covariance
  validation remain outstanding before unattended missions. Normal Abort is tested.

## Implementation and evidence

- `vn_calibration.py`, `maintenance_safety.py`: onboard calibration/safety policy.
- `vn100_node.py`: services/status and the sole sensor reader.
- `fused_nav_bridge.py`: checked pause/prepare, pressure history and reset counter.
- `navigation_supervisor.py`: checked suspension and existing commissioning service.
- `calibration_jobs.py`: GUI/ROS asynchronous orchestration, stale-run/cancel protection.
- `calibration.html`, GUI routes/navigation links: instructions and controls.

Final validation results are appended below. Physical magnetic calibration cannot
be validated by deterministic SITL; hardware cases are mocked in software tests.

### Validation results

- 142 Python regression tests pass, including calibration incomplete/complete
  coverage and freeze failures, stale prior-run rejection, cancellation races,
  preparation timeout/control gating, surface confirmation/pressure stability,
  reset-counter and frame preservation, commissioning gates and None/nonfinite
  heading sample timestamps. Hardware serial and positive maintenance responses
  are mocked in these tests; no physical calibration result is inferred.
- Three browser-script tests pass: calibration page confirmations/stale/armed
  controls/progress, Waypoint rendering and supervisor rendering. Actual GUI HTTP
  checks in local SITL returned200 for /calibration and /api/calibration and409
  for calibration without fresh safety evidence. The interactive browser could
  not reach either local preview server, so visual screenshot review was not
  completed; the read-only preview helper was stopped afterward.
- Final native ROS/GUI/ArduSub nominal run waypoint-a8f102d31f completed the entire
  waypoint sequence after waiting for qualified GPS and learning -40deg against
  injected +40deg VN reference error. GPS was absent underwater and Cube pose
  remained available. Synthetic final endpoint error0.064m is not a real accuracy
  claim. Preparation movement/settling consumed substantially more travel than the
  accepted2.5m minimum: use adequate clearance and pilot control in physical tests.
- Native Abort run waypoint-fe15670c00 passed after alignment, entering TRANSIT
  with GPS absent and confirming MANUAL takeover. An earlier nominal run
  waypoint-65879b62bc also passed and served the HTTP checks above.
- Failed run waypoint-b4dbee4bda is preserved. Supervisor crashed when a missing
  VN/DVL received timestamp was None and heading freshness subtracted it from
  monotonic time. DiveHeadingCheck now rejects missing/nonfinite/nonnumeric
  timestamps; regression covers VN, Cube, DVL and GPS. The simulator harness now
  stops/disarms its owned synthetic vehicle during cleanup, rejects lost supervisor
  telemetry during alignment, and bounds manual alignment movement to25s rather
  than driving throughout the full scenario deadline. Real vehicle controls were
  never involved in this failure or cleanup.
- Initial adapter tests needed new maintained-state fields in their test fixtures;
  fixtures were updated and pressure/reset failure tests added. One browser-test
  command used the wrong working directory and was rerun successfully. These are
  not recorded as successful first attempts.
- Evidence JSONs: docs/evidence/calibration-gui-2026-10-14. No vehicle deployment,
  physical calibration, odometry enablement, arming or real mission in this session.
  The positive GUI calibration/preparation service path still requires hardware
  bench/wet validation; SITL does not emulate physical magnetic calibration.
