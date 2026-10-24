# GUIDED waypoint navigation and one Jetson north/east map

Local implementation on `tether_toggle_beichen`, October 22, 2026. Not deployed to Graey. This replaces the separate surface north-alignment maneuver in the normal GUI workflow. The user approved an initial autonomous straight movement, a clear-corridor confirmation, no separate turn stage, and estimated arrival within 0.5 m for a brief dwell.

## Coordinate contract

- Jetson's geographic map is horizontal metres: **x = north, y = east**. GPS latitude/longitude projects into this plane; GPS altitude does not command depth.
- Cube and VN/DVL odometry retain a **fixed local frame** throughout the bridge session. Its x/y are not advertised as north/east before alignment.
- `geographic_frame.py` maps local XY to north/east using a rotation and translation: `map_NE = R(angle) * local_XY + translation`. The inverse converts geographic waypoint targets to Cube-local control targets.
- `relative_estimator.py` owns the transform. GPS updates this registration; it does not rotate Cube's current odometry or send GPS directly into Cube EKF. VN rotates body DVL velocity into the fixed local frame. Cube still fuses external odometry and its inertial measurements.
- GUI `cube_estimate.map_ne` and supervisor `map_xy` are north/east. `mission_ned` uses the same axes relative to captured surface start once aligned; `mission_local_xyz` preserves the explicitly local diagnostic. Pressure/down remains separate from the 2D map.
- Cube's raw `GLOBAL_POSITION_INT` and yaw are not authoritative geographic displays in this arbitrary-local-frame profile. The waypoint GUI projects **Cube LOCAL_POSITION_NED + ATTITUDE** through the Jetson transform. QGC has not been modified to apply that transform. Do not compare its raw geographic projection as if it shared the GUI map frame.

## Workflow and ownership

1. **VN calibration**, while killed/disarmed/ESC-off. The VN reader remains the serial owner. This does not establish geographic north.
2. **Prepare Navigation**, independently at the stationary water surface. `calibration_jobs.py` and `commissioning.py` validate sensors, pressure zero, bridge profile and Cube/local attitude consistency. `fused_nav_bridge.py` resets the local estimation session and permits output only when healthy. Prepare does not move the vehicle.
3. **Select geographic waypoint and confirm corridor** on `/waypoint`: 5 m ahead plus 2 m stopping clearance. `waypoint_api.py` checks fresh local navigation, reference/session and target bounds. Preview is not propulsion authority; Start explicitly requests mission mode and arming.
4. `GuidedSurfaceRun` requests GUIDED, arms, then holds the captured start for five accepted seconds of surface measurements. `SurfaceMeasurement` tolerates pauses; fixed-local limits are 0.12 m/s motion, 5 degrees/s yaw rate, 3 m GPS scatter and 4 m GPS/DVL displacement disagreement. Those are admission checks, not surveyed accuracy claims.
5. If no geographic transform exists, **ALIGN_MOVE** commands 0.2 m/s straight in the initial Cube-local heading. GUIDED yaw control holds that heading. No separate turn stage occurs. Brake at 4.5 m; hard limits are 7 m total displacement, 65 seconds including alignment stop, and 20 degrees heading error. GPS loss commands zero velocity while waiting; beyond five seconds the mission yields MANUAL.
6. The bridge collects only admitted GPS fixes paired to DVL/VN history. `TrackNorth` fits a rotation/translation between GPS coordinates and integrated relative DVL displacement, rather than treating GPS course as nose heading. Requirements: at least 20 samples, at least 4 m baseline, fit scale 0.5–1.5, residual at most 2 m, conservative heading uncertainty at most 35 degrees. DVL defines metric distance; no GPS scale is applied to DVL.
7. **ALIGN_SETTLE** brakes. After one second at speed at most 0.08 m/s and yaw rate at most 3 degrees/s, a valid candidate is required. **ALIGN_ACCEPT** authorizes applying the transform only after another fresh admitted GPS fix. Missing evidence aborts instead of guessing north. Short GPS requalification may retain the track up to five seconds while local continuity remains healthy; invalid GPS never enters the fit.
8. **TRANSIT** converts the unchanged latitude/longitude target to Cube-local XY each cycle. `SurfaceLeg` uses along-track and cross-track position feedback plus velocity damping, capped at 0.2 m/s. Translation and yaw happen together. Yaw slew is bounded and decreases near the target; lookahead shrinks from 1.5 m toward 0.25 m. Within 1 m, commanded yaw is held.
9. GPS admission requires surface pressure eligibility, fresh VN, advancing timestamps, bounded timing, reported horizontal accuracy at most 1.5 m, and displacement consistency. A gross map disagreement exceeding max(6 m, 4 times reported accuracy) is rejected. Good fixes adjust map translation slowly (at most 0.1 m per fix); stationary DVL prevents GPS from dragging the position. GPS/DVL motion can refine geographic orientation, capped at 0.5 degrees/update and further reduced near the target. This correction freezes within 2 m. The evidence baseline is not shortened below GPS's resolvable scale merely to get more corrections.
10. **Arrival** requires estimated distance at most 0.5 m, speed at most 0.15 m/s and current admitted GPS for one second. Stop, verify speed at most 0.08 m/s for one second, disarm, finish in MANUAL. Starting close to the target after alignment is handled without a zero-length-leg exception.

`navigation_supervisor.py` remains the health/reference/permission observer. The mission owns maneuver decisions, the bridge owns sensor admission and registration, and Cube owns GUIDED stabilization. RF/tether arbitration, pilot takeover, Abort and hardware kill remain independent. No source switching or compass-learning sequence is added.

## Depth and recovery

Surface control never requests positive/down vertical velocity. During alignment and braking it requests zero vertical velocity; normal surface feedback allows only a small bounded ascent when pressure indicates depth beyond 3 cm. This is closed-loop vertical velocity, not guaranteed neutral thruster output. A depth greater than 8 cm or a Cube/pressure vertical-reference mismatch greater than 15 cm aborts.

Brief DVL interruptions use the existing bounded recovery/hold only with independently healthy current navigation. VN continuity loss invalidates even pre-north local navigation. Local frame discontinuity, lost permission, kill, pilot mode/disarm, watchdog failure or extended missing GPS stop the autonomous mission. There is no automatic rearm or kill reset.

## Timing correction discovered in native testing

The old Cube clock uncertainty covered only the latest TIMESYNC RTT/2. Native SITL showed several-millisecond apparent future GPS fixes as clock estimates varied, causing repeated three-second requalification. `CubeClock` now carries the recent three-second offset envelope into uncertainty. The 0.35-second maximum fix age, 50 ms maximum TIMESYNC RTT, regression rejection and clock-reset rejection remain. Larger uncertainty makes old fixes less eligible, not more. Receiver internal delay remains unknown: these are Cube last-fix timestamps, not a claimed synchronized GNSS measurement epoch. Diagnostic logs include signed age, offset and uncertainty.

## Verification

- Python regression: 292 passed, two skipped at the initial completion checkpoint; subsequent final result recorded below.
- Browser JavaScript harnesses: calibration and waypoint page passed, including mandatory corridor confirmation and map controls.
- Deterministic closed-loop simulation: initial map angles 0, 73 and -120 degrees; reported GPS accuracy 1.2 m; alignment, target travel and arrival pass. GPS jump, DVL loss and Abort do not falsely accept north. These deterministic coordinates do **not** represent realistic 1.2 m correlated GPS errors.
- Native ArduSub + ROS bridge/supervisor + GUI API: `sitl/logs/waypoint-7e3471430d/validation.json` **PASS**. Injected VN offset +40 degrees; recovered -40.027 degrees over 4.743 m; completed target transit, stop and disarm. Duplicate Start was idempotent. QGC forwarding disabled, ROS domain 73 localhost only; no physical vehicle or hardware interlocks tested.
- Earlier failed native runs retained: `waypoint-2463aefd41` aborted on GPS qualification/freshness; `waypoint-187f0f0c82` and `waypoint-a60c37a9fd` stopped without accepting an insufficient track. They motivated clock-envelope diagnostics and bounded track retention. No failed run is counted as a pass.
- Native GUI Abort: `waypoint-22747b075b` **PASS**; operator abort during armed surface hold yielded MANUAL. Synthetic control feedback is opt-in and restricted to localhost SITL domain 73; it does not validate physical kill hardware.
- A subsequent regression found the isolated sensor-test fixture did not initialize the new optional control-fixture flag. Defaulting that optional flag to disabled fixed the fixture compatibility issue; final regression result follows.
- Final Python regression: **293 tests run, two skipped, zero failures/errors**. Both JavaScript GUI harnesses pass. Native success/Abort evidence above is separate from these software tests.

## Remaining real-vehicle checks

Deployment requires a fresh killed/disarmed/ESC-off check. Confirm DVL body axes and upside-down VN mounting transform, pressure zero, and forward movement. Run a short supervised open-water mission with the initial corridor clear. Inspect geographic heading/position and logs after alignment. A receiver reporting 1.2 m accuracy cannot guarantee physical 0.5 m arrival; a 5 m alignment may still be too short in multipath or correlated GPS bias. The 35-degree conservative fit bound is permissive and explicitly not precision heading certification. Fixed-frame continuity and smooth control do not eliminate sensor bias or environmental error.
