# GPS coordinate track north alignment

User requested replacing Compass Learn with an explicitly started autonomous
approximately 10 m forward surface run. Work is on `tether_toggle_beichen`.
This document describes local code, not a verified vehicle deployment.

## Operator sequence

1. Calibrate VN if required. Keep VN powered continuously afterward.
2. Float stationary, killed/disarmed with ESC power off. On Calibration, confirm
   surface and select Prepare Navigation. This establishes pressure zero and
   checks the normal ExternalNav profile and sensors; it does not move Graey.
3. Wait for fresh VN, DVL bottom lock and admitted surface GPS. Point the nose
   down a clear 10 m corridor with at least 2 m additional stopping clearance.
4. Enable propulsion normally and arm in MANUAL. Keep controls neutral, confirm
   clearance and click Run 10 m north alignment. The script stays in MANUAL.
5. Supervise the run and stop. After completion, disarm and engage kill. The
   running Prepare job checks Cube fusion before granting waypoint permission.

No automatic arming, flight-mode change, kill recovery, ESC restoration or
obstacle avoidance is added. Physical kill, disarm, mode takeover and Cancel
remain effective. Neutral vertical MANUAL_CONTROL input is always 500; this
script does not command depth hold or a dive. Surface pressure must remain
between -0.10 and +0.15 m. Physical behavior still requires wet verification.

## Data and control path

GUI `CalibrationJobs` checks surface sensors and clearance, calls
`begin_motion_north`, then starts `north_maneuver`. The bridge independently
rechecks fresh armed MANUAL control, GPS, DVL, VN and the complete profile.

`NorthManeuver` targets 0.4 m/s forward using bounded PI velocity feedback,
relative VN heading hold and lateral DVL velocity damping. It limits forward
command to 450/1000, yaw and lateral movement corrections to 150/1000. The
braking phase limits horizontal corrections to 300/1000. Heading error over
20 degrees, yaw rate over 15 degrees/s, speed over 1 m/s, stale telemetry,
depth violation or takeover aborts. These limits are not a guarantee of
physical stopping distance under current or a propulsion failure.

`TrackNorth` receives only admitted GPS coordinates with advancing Cube fix
timestamps, paired with DVL/VN history using the existing clock mapping. It
fits a planar rotation and translation between relative DVL positions and
GPS north/east positions. It does not equate GPS course with nose heading;
sideways motion is represented in both tracks. The resulting rotation is the
north offset applied to VN orientation in the bridge, not a VN firmware write.

Acceptance requires at least 20 samples, 9 m usable displacement, scale ratio
0.75–1.25, RMS residual at most 0.75 m, and the conservative heading uncertainty
indicator `atan2(2 * maximum_reported_GPS_accuracy, baseline)` at most 12 degrees.
GPS reported accuracy is not divided by sqrt(sample count). The indicator is
not a certified confidence bound; correlated drift and sensor mounting errors
can still bias alignment. Rotation during capture is limited to 5 degrees/s
and tilt to 15 degrees. Existing GPS freshness, quality, jump and pressure
admission checks remain upstream.

At 9.5 m integrated travel the maneuver begins braking if a candidate exists;
otherwise it stops without accepting north. A three-second stationary check
requires DVL speed <=0.05 m/s, yaw rate <=2 degrees/s and GPS speed <=0.2 m/s.
Only COMPLETE plus a subsequent admitted GPS fix applies the offset. Raw track
records and fit evidence are logged as GPS_TRACK_NORTH; live status carries a
compact summary, avoiding re-broadcasting the full coordinate history at 20 Hz.

Sensor loss neutralizes movement immediately and aborts after five seconds.
Travel during loss is charged conservatively. Movement times out after 45 s;
the whole operation after 60 s. Total travel including braking is capped at
12 m. Failure does not retry or extend distance automatically. Ten metres can
be insufficient for poor GPS; rejected data never force a north reference.

## Removed and retained

Removed Compass Learn GUI, services, state machine, GPS-learning watchdog and
temporary native-GPS/barometer/compass profile writes, plus obsolete learner
tests and its native validator. Historical documentation remains as evidence.
VN calibration remains. The ordinary waypoint GPS/DVL correction loop remains.
Supervisor supplies health/commissioning and does not run compass learning.

Polling retains the batching fix. It now requests the 24 navigation fields and
COMPASS_LEARN=0 readback. An interrupted legacy learning/profile state blocks
navigation visibly; this change does not silently repair a live Cube. Before
deployment, verify/restore the normal profile and learning off while maintained.
No compass parameter or vehicle change has been made by this local task.

## Verification record

Python/GUI regression results and final native simulation result are recorded
below after validation. Deterministic tests are not a substitute for vehicle
dynamics or hardware kill tests.

- Native attempt `north-maneuver-40915357`: failed because bridge's initial
  three-second motion watchdog expired during ROS process discovery. Fixed
  startup grace to ten seconds; active status expiry remains 0.7 s. Maneuver
  also aborts if the bridge ends the alignment session.
- `north-maneuver-d4e6cb35` and `north-maneuver-c9677cc4`: aborted on stale
  synthetic controller status (0.719 / 0.853 s). Safety limits retained.
  Fixture used wall-clock timers with host time adjustments in logs; changed
  fixture timer to STEADY_TIME to match production maneuver scheduling.
- `north-maneuver-9d42734e`: HTTP 409 start rejection; fixture saw Cube armed
  before its synthetic controller state reached GUI. Fixed fixture to wait
  for confirmed controller arm/ESC/kill state and retain HTTP error text.
- Deterministic closed-loop tests cover successful 0, 73 and -170 degree
  offsets, GPS distortion, sustained dropout and cancellation; all pass.

Final integrated native run `north-maneuver-896be1b3` PASS:
MOVE -> SETTLE -> COMPLETE -> Prepare ready. Total integrated travel 9.896 m;
GPS/DVL baseline 9.885 m; recovered offset -40.000 degrees for injected VN
+40 degrees; fit RMS residual 0.0157 m; 226 samples; uncertainty indicator
3.474 degrees. Cube fusion commissioning passed after simulated disarm/kill.
QGC forwarding disabled, hardware interlocks not tested. Compact evidence:
`docs/evidence/gps-track-north-2026-10-21.json`. Full ignored logs remain in
`sitl/logs/north-maneuver-896be1b3/`.

Final regression: 282 Python tests PASS; calibration GUI and waypoint GUI
JavaScript suites PASS. Native parameter polling PASS in both enabled/paused
output states: all 25 readbacks refreshed within 2.005 seconds. Evidence:
`sitl/logs/parameter-polling-20261021-233854/result.json`.
No live deployment was performed. First physical run must verify mounting,
forward/lateral/yaw signs, neutral vertical behavior and stopping clearance.

## Five-metre revision

The user requested 5 m instead of 10 m, with only targeted checks. The current
run target is 5 m: brake at 4.5 m, require at least 4 m usable GPS/DVL baseline,
and retain 2 m stopping clearance (7 m hard travel limit). Speed, feedback,
freshness and quality thresholds are unchanged. A shorter baseline can reject
GPS that would have passed the longer run; no accuracy gate was weakened.
Targeted maneuver/transport checks (13), closed-loop checks (2) and GUI checks
passed. This supersedes the 10 m distances above; that native run remains
historical evidence and was not rerun as a 5 m native mission.
