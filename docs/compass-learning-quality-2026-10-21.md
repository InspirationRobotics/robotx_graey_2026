# Initial north learning: lane coverage and GPS quality

User approved implementation on 2026-10-21. This workflow establishes an initial
Cube compass north offset for VN relative orientation; it does not replace VN
as the ongoing external heading source or change waypoint heading control.

## Evidence motivating the change

The inspected approximately 225-second live attempt acknowledged mode 3 and
GPS source readbacks, but never emitted `CompassLearn: Finished`. Cube used
three IMU cores (`EK3_IMU_MASK=7`) while `EK3_GSF_RUN_MASK=3` excluded core 2,
which became primary. Its 1,927 GPS samples had median speed 0.16 m/s and
90th percentile 0.41 m/s; 59 samples exceeded 2 m/s, with maximum 31.98 m/s.
Cancel restored the bridge-only profile. Exact GSF variance/clip rejection is
not available in this telemetry; the Cube DataFlash log is needed for that.

## Implementation

- Learning requires GSF coverage for every active core, including after a
  readback change during an attempt. GSF bits refer to core indices, not the
  physical IMU bit numbers. For the verified three-core setup, RUN mask 7 is
  required. The one-time deployment changes RUN 3 to 7 and reboots for allocation;
  USE mask stays 3. No broader automatic yaw-reset permission is added.
- `CompassGPSQuality` monitors Cube GPS messages during learning/capture only.
  It cannot filter native GPS inside Cube. Serious contamination invalidates
  this attempt and prevents north transfer; restoration is requested through
  the existing fresh MANUAL/no-mission gate. No arm/disarm/mode/motion command
  is added and attempts never restart automatically.
- Immediate rejection: reported speed above 5 m/s, GPS clock reversal, or a
  position step exceeding the larger of 5 m and
  `3*(previous_accuracy + current_accuracy) + 3*elapsed_seconds`, compared over
  intervals shorter than 1.5 seconds. This is a generous jump bound, not a claim
  of navigation accuracy.
- Unsuitable fix (<3D), invalid metadata, position accuracy >3 m, or velocity
  uncertainty >0.5 m/s pause capture. One continuous second rejects the attempt.
  Missing/nonadvancing measurements first become stale after 1.5 seconds and
  reject after a further second. Duplicated packets do not refresh this clock.
  Brief interruptions can recover without restarting the firmware learner.
- Low speed compared to velocity uncertainty is motion guidance, not rejection.
  The instructions request controlled forward acceleration/deceleration and
  turns within available clearance, then a level steady capture. Turning in
  place or waiting longer is not itself convergence evidence.
- GUI shows GPS speed, velocity/position uncertainty, measurement age,
  interruptions, last reported active lane, lane-mask readbacks and firmware/
  capture/restoration stages. GSF yaw confidence is explicitly unavailable;
  elapsed time is not a progress percentage.
- Firmware completion, mode-0 readback and stable paired Cube/VN attitudes remain
  necessary. Failed attempts cannot publish a captured reference, including a
  GPS fault arriving immediately after capture but before restoration.

Native GPS already consumed by Cube before detection cannot be undone by this
monitor. Firmware-written compass calibration offsets are not automatically
rolled back. A rejected attempt produces no accepted VN north reference; it
still needs a clean repeat and physical heading verification.

## Verification

- Full Python suite and both GUI suites cover masks, brief/sustained outages,
  duplicates, clock reversal, jump/speed corruption, tainted capture restoration,
  explicit retry and preserving an already completed reference during later GPS
  disturbances. An initial test failed because its fake node lacked a logger;
  the test fixture was fixed, with production behavior unchanged.
- Three-core native SITL completed actual learning and full profile restoration:
  `sitl/logs/compass-learning-20261021-223707/result.json`, 122 paired samples over
  2.026 seconds, offset -2.57 degrees, about 13.82 wall seconds at simulation
  speedup 3. This does not establish live compass accuracy or duration.
- Native GPS-jump injection passed rejection and complete restoration with no
  north reference: `sitl/logs/compass-learning-20261021-223916/result.json`.
- Native unarmed polling refreshed all 31 compass fields within 2.007 seconds:
  `sitl/logs/parameter-polling-20261021-223821/result.json`.
- All simulations use loopback-only isolated firmware; QGC forwarding disabled.
