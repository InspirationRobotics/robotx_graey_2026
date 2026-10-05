# GPS telemetry review — October 3 session

User supplied `graey_2026-10-03_session_telemetry.tlog` (37,655,367 bytes).
Offline analysis and local changes only; vehicle unavailable. Branch:
`tether_toggle_beichen`. Existing SITL edits were preserved. No parameters,
services, main branch, commits or remote repositories were changed.

## Evidence

The log spans 2026-10-03 07:22:18 UTC to October 4 03:39:33 UTC with gaps;
it is not a continuous 20-hour test. Firmware identifies ArduSub 4.5.7,
commit 30257f01. There are 18,592 GPS reports, 15,557 global-position reports,
2,136 local-position reports and 17,603 ODOMETRY reports.

Consecutive reports less than two seconds apart show:

| Stream | Steps over 1 m | Largest horizontal step |
|---|---:|---:|
| Raw GPS | 460 | 174.78 m |
| Reported global position | 285 | 119.60 m |
| Reported local position | 0 | 0.238 m |

These are observed coordinate steps, not proof the vehicle was stationary.
284 of the 285 global steps occurred with the latest EKF flags equal to 167:
constant-position mode, without horizontal-position-valid bits. Only one
occurred with flags 831. The local-position stream ends at 02:42:02 UTC,
immediately before flags change to 167. ODOMETRY is recorded only in the early
07:22–07:29 UTC segment. Its later absence from this QGC log does not prove
that the DVL itself or its Jetson publisher stopped; obtain ROS/bridge logs.

469 GPS reports lack a 3D fix. Another 1,022 report a 3D fix but fail the
known-positive <=3 m horizontal-accuracy check. Some reacquired 3D fixes report
71.127 m and 110.910 m uncertainty; the maximum 3D reported uncertainty is
170.737 m. A 3D flag alone is clearly insufficient.

Recorded parameters consistently show source 1 horizontal position/velocity
ExternalNav (6), source 2 horizontal position/velocity disabled (0), and
EK3_SRC_OPTIONS=1. Source 2 yaw is also disabled. This does **not** match the
repository candidate Surface GPS profile. There are no source-selection
commands (42007) and no NAV_SRC reports in the log. Firmware emits 784
`No ap_message for mavlink id (251)` messages, consistent with an older deployed
supervisor requesting an unsupported stream. The existing local code already
removes that request; source feedback requires the separate reporter.

No timestamp regressions were found in the GPS/global/local streams. That
does not categorically exclude a duplicate vehicle sender, but this log does
not reproduce the earlier obvious real/SITL time alternation.

Conclusion: the data does not establish GPS overpowering a healthy DVL-aided
position EKF. It establishes unreliable GPS and mostly invalid horizontal
estimation during the large global-coordinate changes. Healthy external aiding
and the deployed source configuration must be verified before noise tuning.

## Changes and verification

- GPS admission now checks displacement against a fresh atomic DVL motion
  history. Rejected coordinates do not become trusted anchors.
- Rejected GPS receives no grace; sustained missing GPS also requests external
  aiding, subject to the existing health, alignment and commissioning gates.
  Surface mission permission stays blocked until qualified GPS recovers.
- GUI marker/origin admission rejects the demonstrated unhealthy cases while
  keeping raw coordinates visible. This does not change QGC.
- `tools/audit_navigation_tlog.py` provides reproducible offline analysis and
  production GUI-cache replay; requires pymavlink. It outputs aggregate
  evidence without coordinates and never opens a vehicle connection.
- Replay withheld the global marker for all 284 steps with invalid horizontal
  EKF status; it retained the one step with valid status. This is GUI replay,
  **not an EKF re-simulation or proof that physical navigation is fixed**.
- Regression coverage includes stationary GPS drift, actual DVL-supported
  movement, bad/unknown accuracy, stale/replayed/invalid DVL, rejected GPS,
  prolonged dropout, recovery, source ACK failure, and display health.
  All 57 unit/regression tests passed; both navigation pages passed JavaScript
  syntax checks, edited Python compiled, and git diff whitespace checks passed.
  No new full ArduSub SITL or physical-vehicle trial was performed in this pass.

## What is needed before numerical EKF tuning

1. Cube DataFlash `.BIN` covering the event and a full current parameter export.
   This tlog contains source parameters but not GPS noise/gate settings or the
   full innovation history. Do not infer current values from old repo snapshots.
2. Concurrent Jetson DVL atomic samples, VN-100, bridge status and supervisor
   status, including validity, timestamps and restart information.
3. A disarmed stationary recording with antenna dry, at the waterline and
   briefly wet, with noted event times; maintain a known bottom lock. Confirm
   pressure calibration and sensor/antenna geometry before depth thresholds.
4. Restore/verify healthy external aiding first. Confirm installed code matches
   source, source profiles match readback, and NAV_SRC feedback is present.
5. Compare GPS innovations, reported accuracy, rejection ratios and local-pose
   continuity. Only then trial GPS position-noise and innovation-gate settings
   in isolated SITL/replay and controlled vehicle tests. Do not loosen failsafes
   or shrink glitch-reset thresholds blindly. No numerical EKF parameter change
   is justified as a validated tune from this tlog alone.

Direct GPS-to-Cube wiring means supervisor rejection cannot pre-filter each
receiver packet. It can withdraw GPS as a source, with latency and only when
the fallback is healthy. Proper Cube rejection and handling of lost external
aiding remain required. Never auto-arm or restore ESC permission during this
commissioning.
