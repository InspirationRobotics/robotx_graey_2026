# navigation_new — 2026-10-24 local implementation and validation

## Scope and status

Created navigation_new from tether_toggle_beichen at 27b3dc6, preserving the existing uncommitted DVL/10 m work. No vehicle connection, deployment, live parameter writes, main changes or push occurred in this session. Unrelated docs/nav_lgoic.md is preserved and excluded from the commit.

The active surface-waypoint path is simplified. Other vehicle functions (hardware kill, tether/RF arbitration, sonar and manual controls) were not removed. Historical mission/estimator modules remain for compatibility and comparison; this is not a claim that every old source file was deleted. Legacy supervisor-driven competition missions have NOT been validated against the new read-only supervisor; do not use them as tested entry points on this branch.

## Active sequence and responsibilities

1. `api/gui/calibration_jobs.py` exposes VN calibration, cancel and navigation startup only. `tools/calibration.html` presents Calibrate VN and Start Navigation. Startup enables the bridge data path and establishes pressure zero from surface-confirmed samples; it does not wait for a sensor qualification dwell, stationary motion, north alignment or supervisor permission. Maintenance/physical-state checks protect calibration/restarts. Startup never arms or restores ESC power.
2. `api/navigation/fused_nav_bridge.py` remains the single external-odometry producer, consuming VN orientation, DVL velocity, pressure and timestamped Cube GPS telemetry. MAVProxy remains the physical Cube serial owner. Existing separate UDP Link ownership remains.
3. `continuous_estimator.py` / `horizontal_ekf.py` implement a Jetson horizontal EKF: fixed-local x/y, vx/vy and relative yaw. VN yaw/rate, body DVL velocity and admitted GPS position are independent updates. Joseph covariance update and normalized innovations reject bad measurements. GPS position covariance retains a conservative floor for correlated GNSS bias. Cube position is not fed back as a sensor.
4. GPS admission retains finite/range/fix/accuracy, pressure-surface eligibility, receiver timestamp freshness and jump checks. Reported accuracy must be positive and at most 1.5 m. Jump admission uses 4 m + 2 m/s times elapsed since the trusted fix, then the EKF checks innovation. No DVL-agreement gate is required to admit GPS. Invalid measurements are skipped, never replaced by a measured zero.
5. Accepted sensor timestamps form AIDED / DEGRADED / LOST health (2 s per sensor). Individual dropouts retain prediction and use available measurements; valid returns update automatically without Prepare or a continuity latch. Initial north not yet known is shown as ALIGNING, and GPS is labelled admitted rather than position-aiding until registration exists.
6. `navigation_supervisor.py` only subscribes and publishes status. It has no MAVLink Link, service clients/servers, source switches, reference capture, reset or mission permissions. Malformed mission messages cannot replace its dictionary or crash publication.
7. `api/gui/waypoint_api.py` validates the geographic target/session and starts the existing mission process through GUI ownership. Start requires initialized navigation, fresh Cube control telemetry, a geographic anchor and explicit clear 10 m corridor confirmation. Surface waypoint range remains 0.3–20 m. These are not repeated sensor qualification windows.
8. `waypoint_mission.py` adapts ROS/MAVLink/control ownership; `guided_surface_mission.py` contains the surface state machine. Despite its historical class name, runtime mode is STABILIZE. It acquires control, arms if necessary, and sends neutral vertical input 500 throughout surface movement.
9. With no existing geographic alignment, `straight_alignment.py` holds the starting relative heading and uses lateral velocity/cross-track feedback for approximately 10 m. A separate DVL-only accumulated track supplies heading evidence so GPS cannot validate its own fused track. `track_north.py` fits GPS coordinates against that track. At 9.9 m estimated along-track travel, stop forward travel and accept qualified evidence; if evidence is insufficient, retain the mission and wait rather than drive farther blindly.
10. The fixed Cube frame never rotates or resets. `geographic_frame.py` maps it to Jetson x=north/y=east. Alignment updates that map transform. GPS altitude does not enter horizontal mapping; pressure supplies vertical odometry.
11. After registration, continuously navigate toward the geographic target. Every admitted fix is offered to rolling heading estimation in `course_heading.py`; sufficient motion/evidence still matters. Correction is capped at 0.25 degrees per accepted update and frozen within 2 m of the target. There is no one-metre mission stop/check loop. Existing statistical sampling/update-rate rules prevent a single noisy fix from rotating north.
12. Estimated distance within 0.5 m plus low speed for a brief dwell leads to STOP, disarm, MANUAL and COMPLETE. All GPS/DVL/VN accepted measurements lost together abort. Operator abort, kill/control ownership, mode takeover, broken coordinate frame and command-loop interruption remain independent stop/takeover reasons. Missing sensor data alone does not reset the mission.

## GUI

`tools/waypoint.html` retains map, pose/coordinates, live estimator/sensor health, coordinate entry/GPS recording and Start/Abort. Optional coordinates, imagery and Cube statistics are collapsed. Startup has two clear actions; retired standalone north/compass/recovery actions are not offered. World imagery defaults online; saved Miramar and pool archive options remain. World tile requests are capped at zoom 19 because browser inspection reproduced provider placeholder tiles at zoom 21. Zoom enlarges existing imagery rather than requesting missing detail. Satellite placement is still not a surveyed shoreline.

## Validation and failures encountered

- 298 Python regression tests pass under ROS Humble/Python 3.10 in WSL.
- Waypoint and calibration JavaScript suites pass. Headless Edge showed no script errors; Start/Abort visible at 1440 x 950. Final satellite screenshot verified actual imagery after fixing excessive tile zoom.
- Three isolated native ROS + GUI API + ArduSub 4.7.2 SITL runs passed. QGC forwarding disabled, localhost simulation only. See navigation-new-validation-2026-10-24.json for exact run IDs and sampled vertical inputs.
- Final completion run 97d2b40a77: 9.9035 m alignment; fitted -40.075 degrees for injected +40 degree VN offset; survived separate 5 s DVL and VN outages; reached estimated target and disarmed.
- Failure run a8227d87ef: GPS loss observed without immediate abort, then simultaneous GPS/DVL/VN loss correctly aborted. Abort requests MANUAL takeover; it is not an automatic return or proof of physical stopping.
- Unit coverage includes individual/two-sensor loss, all-sensor loss/recovery, DVL gaps, GPS jump rejection, cardinal body frames and no repeated segment stop.
- Early launches failed on incorrect WSL ROS import paths before exercising the mission. Regression initially exposed outdated supervisor/prepare/transport fixtures, which were updated to assert the new contracts. Also fixed a real malformed-status crash and removed the obsolete stopped-segment correction callback. A longer pure simulated path required extending test duration; it did not justify changing mission completion criteria.

## Limits and remaining vehicle verification

Simulation uses synthetic sensor truth; it does not establish lake-current, EMI, GNSS multipath, DVL scale/mount or real vehicle control performance. Heading is still essential to rotate body DVL into a fixed frame. A single surviving sensor cannot bound every state indefinitely; DEGRADED permits continuation as requested, not a guarantee of accuracy. Physical arrival within 0.5 m is not guaranteed by metre-level GPS.

The Jetson EKF is a modest horizontal filter, not a complete copy of EKF3 fault diagnosis. Its covariances/noise constants require real data review. Cube still performs inertial fusion and attitude control. Correlations between external fused odometry channels and Cube fusion are not fully modelled. GPS timing uses Cube timestamp/clock alignment and bounded latency compensation, not hardware PPS synchronization.

Before deployment: review the new branch/profile and backup the vehicle; verify disarm/restart state, single bridge ownership, parameter profile, timestamps and physical frame signs. Then test bounded surface motion in a clear corridor with takeover available. No live readiness is asserted here.


## Follow-up: distance controls and current scenarios

Every new mission now performs its initial approximately 10 m straight leg, including when a map registration already exists. First commanded movement is forward in STABILIZE; mode acknowledgement, control ownership and arming still precede thrust. No stationary GPS qualification dwell was added. A known map is retained rather than resetting Cube coordinates; new GPS observations continue the normal heading correction.

Waypoint GUI retains manual latitude/longitude entry and live coordinate output. Record GPS is visible in the main sidebar. The last recorded coordinate is saved in browser localStorage and displayed outside collapsed controls, with a Navigate to recorded point action that selects it without starting propulsion. Persistence is browser/origin-specific, not cross-device. Distance updates from fresh mapped Cube position, falling back to explicitly labelled raw GPS before registration. Move target applies the entered 0.3–20 m distance along the existing bearing from the current vehicle position. Editing does not change an active mission; stale/missing position disables distance editing.

Current simulations: `python -m sitl.validate_current_scenarios`. These are software closed-loop sensitivity tests of the production estimator, mission, line guidance and STABILIZE velocity controller, NOT native ArduSub hydrodynamics or measured Miramar conditions. Assumptions: constant lateral current, synthetic GPS/DVL/VN, first-order horizontal response with 1 s time constant and pilot command/1000 through-water speed. No waves, heave, multipath or motor asymmetry. User approved assumed scenarios after no measured current was supplied.

Results: 0.05 m/s completed with 0.378 m final physical-model error; 0.10 m/s completed with 0.438 m error; 0.20 m/s did NOT complete by the 700 s horizon and drifted about 40.1 m from the target. The existing gentle thrust cap cannot reject that modelled disturbance. This is an exposed operating limitation, not a successful lake validation; thrust limits were not silently raised. See `current-scenarios-2026-10-24.json`. Existing native no-current SITL evidence remains in the previous section.

Follow-up checks: 14 focused Python tests and waypoint GUI suite pass, including retained-map initial leg, saved GPS reload and distance/bearing editing. First current-test harness invocation failed due to a test-local variable shadowing the current vector; fixed and all three scenarios rerun. No vehicle access, deployment or push.
