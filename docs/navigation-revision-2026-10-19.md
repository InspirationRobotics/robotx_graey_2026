# Surface mission revision and compass-learning decision

User reports lake GPS was reliable, heading sensitive to waves, and upward thrust
after initial mission validation. Requests GPS-primary navigation with DVL/VN
support, straight waypoint legs, ongoing heading correction, and GUI steps:
VN calibration -> ArduSub Compass Learn/manual driving -> supervisor health check.
Deployment remains explicitly on hold. No vehicle access or live changes here.

## Implemented independently: remove continuous surface ascent

Prior helper requested upward speed even at zero or negative calibrated depth:
`-min(.10,.03+max(0,depth)*.8)`. Both initial surface hold and transit used it.
This can drive vertical control against the air/water boundary. IMU calibration
does not remove this request. Code behavior is proven; exact physical output and
all causes of the reported event have not been recovered from a live mission log.

New helper requests zero vertical speed for pressure depth <=0.03 m, then smooth
upward correction `-min(.05,max(0,depth-.03)*.5)`. It never requests descent.
Retains 0.08 m pressure abort, vertical-frame checks, horizontal limit and takeover.
Zero speed still uses Cube Z control; it is NOT a neutral-thrust guarantee. Actual
motor polarity, buoyancy/trim, vertical integrator and response require live review
before declaring physical upward thrust fixed. No parameter/gain changes made.

255 Python tests passed, including deadband and monotonic bounded correction.
Both existing GUI JS suites passed. Isolated GUI/native ArduSub SITL run
waypoint-637a1d879b completed surface hold/transit/disarm with 2s GPS outage and
simulated final horizontal error 0.2476 m. See accompanying validation and command
trace summary under docs/evidence/surface-deadband-*2026-10-19.json. QGC forwarding
disabled, ROS domain73 localhost. This is not a physical vehicle test.

## Proposed mission sequence (not implemented by this fix)

1. Establish a validated independent true-north reference and initial position.
2. Freeze a geographic start and waypoint; convert to a fixed north/east line.
3. Turn toward the leg within bounded yaw rate while holding horizontal position;
   require actual turn completion, not merely issuing a yaw target.
4. Track along the leg, correcting cross-track error from qualified GPS-aided
   position; use DVL velocity feedback to brake and bridge GPS interruptions.
5. At matched timestamps, compare GPS displacement/velocity with body DVL rotated
   by full VN attitude. Learn only a slow, bounded yaw-offset correction after
   sufficient displacement versus GPS uncertainty and sustained consistency.
6. Freeze learning when stationary, GPS rejected/stale, DVL lost, turns/tilts are
   excessive or the discrepancy is ambiguous. Account for sideways drift; GPS
   course is not body yaw. Retain uncertainty and bounded outage continuation.
7. Slow near target, confirm arrival, stop/disarm per mission policy. Underwater,
   GPS has no correction authority; retain last trusted reference with DVL/VN.

GPS is the primary *absolute position* measurement when qualified. It cannot
replace attitude needed to rotate body DVL and allocate world-frame movement.
Do not use the GPS-aided estimator to independently validate the same GPS fix,
or Cube ExternalNav yaw to independently validate its own VN source.

Existing correction requires >=0.5 m/s almost-forward travel for >=3 s, whereas
surface mission requests <=0.2 m/s, so current heading correction normally does
not activate on a leg. Merely reducing that threshold can learn GPS noise as yaw.
Wave tilt is addressed through full attitude/time alignment and quality gating;
repeated calibration alone does not eliminate real yaw drift or magnetic EMI.

## Cube north workflow — user clarified and implemented locally

ArduSub Compass Learn calibrates Cube-connected magnetometers, not the VN-100
connected to Jetson's serial/ROS reader. It does not directly write VN calibration
or assign a north offset to a relative VN orientation. The selected physical
compass, its mounting/calibration/EMI and reference transfer must be defined first.

User clarified: learn north on Cube and transfer that reference to VN. The local implementation temporarily selects compass yaw only during manual commissioning, then restores ExternalNav yaw after capture. It does not enable an unidentified compass. The selected compass and firmware support remain live commissioning prerequisites.
Last historical configuration disabled M9N compass and selected ExternalNav yaw;
that is not a fresh live parameter verification. Prior mounting uncertainty remains.

Official advanced setup documents COMPASS_LEARN=3 with GPS lock, arming/manual
movement and turns, completion status and return to0. It says not to use position
control during learning. Firmware build and estimator support must be verified
before promising it works on this ExternalNav-only configuration. Latest upstream
coverage shows learning depends on a usable GSF yaw estimate; that source is not
proof of the exact installed firmware's behavior.

https://ardupilot.org/sub/docs/common-compass-setup-advanced.html
https://firmware.ardupilot.org/coverage/AP_Compass/Compass_learn.cpp.gcov.html

Implemented GUI sequence:
1. Calibrate VN, with current safe rotation workflow.
2. Compass learning: explicit start/abort, actual parameter/readback and firmware
   completion evidence; manual supervised drive only, never mission auto-start.
   Transfer independent accepted north to VN only under defined safe conditions.
3. Check navigation health: supervisor-derived per-sensor freshness, GPS admission,
   pressure baseline, DVL lock/continuity, VN continuity/reference, Cube profile
   and estimate consistency. Show explicit blockers and separate control permission.

Supervisor remains a runtime monitor even if the operator only uses its GUI
health summary. Removing runtime health/reference gates would permit a later
sensor failure to go unnoticed. Compass learning does not auto-arm, clear a kill,
restore ESC power or imply mission permission. GUI workflow is implemented; the full straight-line guidance and slower GPS/DVL heading-correction redesign remain proposals, not silently included in this fix.

## Guarded Cube-north implementation details

- New CubeNorth state machine and three Trigger services on the existing bridge:
  compass_start, compass_capture, compass_cancel. Existing MAVProxy serial ownership
  and UDP routing remain. No extra MAVLink node or serial reader.
- Start requires maintenance (fresh kill/disarm/ESC-off, no mission), odometry paused,
  completed VN calibration state, fresh healthy Cube magnetometer/GPS, compass
  acquisition enabled, primary COMPASS_USE=1, nonzero selected device ID, fresh
  parameter readbacks, learning off and all yaw slots6. It does not guess compass
  identity/mounting or enable compass acquisition; setup may require a separate
  verified reboot. A disabled/unidentified compass produces an explicit blocker.
- Start requests yaw1 in all three slots plus COMPASS_LEARN3. This is temporary
  maintenance, not mission source switching. Bridge output and waypoint permission
  remain suspended. Operator manually controls propulsion and MANUAL movement;
  no service sends arm, motor, ESC-power or mode commands.
- Capture requires firmware CompassLearn: Finished observed for this process/session,
  learn0 readback, compass yaw1 readbacks, healthy magnetometer and GPS reception,
  fresh Cube attitude paired with VN (<0.15s receipt separation), moderate tilt
  <=30degrees and angular rate<=10deg/s. At least20 pairs spanning2seconds, latest
  <=0.2s old, and circular offset spread<=5degrees are mandatory. Paired relative
  rotation can remain consistent through gentle wave motion; a tilted raw magnetic
  heading or GPS course is never substituted for true Cube attitude.
- Capture stores only Cube-minus-VN north offset, then requests all yaw slots6 and
  learn0. It waits for exact fresh readbacks before marking the transfer ready.
  Surface Prepare must still establish real pressure baseline, GPS/DVL position,
  Cube fusion and supervisor health. Heading transfer does not set position.
- No north accepted merely from enabling learning or seeing parameter0. Unsupported
  firmware/GSF configuration or nonconvergence can prevent completion; after600s
  learning reports timeout and requires killed/disarmed Cancel. Cancel also recovers
  a recognizable interrupted learning profile after node restart, returning yaw6
  without claiming north. Kill/ESC-off required for all parameter writes.
- VN calibration/data gaps/orientation discontinuities invalidate pending capture;
  normal DVL continuity recovery does not trigger new compass calibration.
- Supervisor remains active during missions, independent of RF/tether/kill. GUI step3
  uses supervisor state/reason/profile/GPS/depth and bridge VN/DVL/odometry evidence.
- Legacy manual north controls are hidden from the normal three-step workflow;
  retained backend compatibility is not the operator's default preparation path.

Initial expanded-bridge SITL run waypoint-5cc75b0fa4 aborted before transit because
navigation configuration permission became unavailable. This failure is preserved
under docs/evidence/compass-integration-first-sitl-failure-2026-10-19.json. Compass
polling was restricted to paused maintenance/learning and ordinary navigation's
ordered profile polling retained; added regression checks. The failure does not
prove a physical vehicle fault or a specific telemetry-loss cause.

## Final validation and delivery boundary

Final Python regression suite:265tests. Both calibration and waypoint GUI JavaScript
suites pass, including actual confirmation payloads for learning and health prep.
The first new GUI test caught missing compass-confirmation forwarding; corrected
before delivery. Tests cover circular source rejection, actual active-learning
readback before accepting completion, failed/partial restoration, safe restart
Cancel, stale/invalid pairs, changing offsets, VN loss, maintenance-only writes,
no propulsion commands and normal telemetry polling unchanged.

Rerun native GUI/SITL waypoint-68b7289639 passed CHECK_SURFACE, transit with2sGPS
outage, disarm and completion; final simulated horizontal error0.0782m. See
cube-north-integration-sitl-2026-10-19.json. It validates ordinary mission/bridge
integration, NOT physical compass calibration or native Compass Learn convergence.
Actual learning/heading transfer has unit/adapter simulation coverage only; installed
firmware and selected compass commissioning remain required. End-to-end learning
has not been claimed successful on Graey or the native simulator.

No SSH/deployment/live parameter changes in this session. The user explicitly
paused deployment. Last recorded vehicle revision dddf46d belongs to a teammate's
sonar merge; preserve that work during any later authorized integration. The
previous EK3_SRC1_POSZ repair was a separate authorized action on October18.
