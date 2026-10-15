# VN relative heading and pre-test north alignment — October 14, 2026

## What changed

User selected GPS/DVL surface movement rather than a manually entered heading.
Every new production bridge instance now starts without a north alignment.
The VN driver verifies relative-heading mode on serial connection/reconnection,
preserving the other VPE settings and writing no flash. It rejects invalid ASCII
checksums and nonfinite measurements. No fixed yaw offset was added.

GPS/DVL alignment and magnetic calibration are separate operations. The former
is required each navigation session. Repeating full magnetic calibration requires
handling the assembled vehicle in three dimensions; it cannot happen automatically
while it is sitting still or during an arbitrary mission. The script below can
be used before each water test if the team chooses to recalibrate each time.

## Guided magnetic calibration

With the assembled vehicle powered, disarmed, physical kill engaged and ESC power
off, stop the ROS core service. Keep MAVProxy running as the Cube serial owner.
Inside the graey container, source the ROS/workspace environment and run:

```sh
cd /root/robotx_ws/src/robotx_graey_2026
python3 tools/vn_calibrate.py --calibrate --killed-disarmed --seconds 120
```

The script uses the same process-ownership lock as vn100_node and refuses to steal
its serial port. It checks a disarmed Cube heartbeat initially and throughout,
backs up registers before writes, sets relative mode, resets and runs slow onboard
HSI calibration, logs coefficients and attitude samples, and stops learning even
on interruption. Rotate yaw/pitch/roll to cover all six faces away from external
metal and varying magnetic sources, keeping the sensor mounted. Maintain kill
throughout: heartbeat proves disarm, not physical kill. The flag is the operator's
confirmation of kill; the script does not independently measure the power chain.

Coverage requires at least five samples near each gravity-axis face. Coverage is
not a residual-quality or absolute-heading certificate. Outputs remain volatile;
the script does not save flash, accept calibration as valid, change mounting
rotation, or introduce a compass-derived north offset. A failed/incomplete run is
reported and logged; do not treat it as a successful calibration.

Use the same command without --calibrate to back up/read settings only. Core
must still be stopped to release serial ownership. After calibration restart the
core with a newly confirmed surface pressure baseline. Any previous odometry or
mission reference is invalid after this maintenance restart.

## Pre-test north alignment

1. Start the core killed/disarmed at the confirmed water surface with the measured
   `surface_pressure_hpa`, `odometry_enabled:=true`, `waypoint_enabled:=1`, and
   `mission_commissioned:=false`. These are launch arguments; no source file edits
   are needed. Existing systemd overrides must be updated deliberately at deployment
   rather than starting a second core. Defaults still disable output/execution.
2. The bridge observes its first Cube heartbeat. An armed startup latches a fault;
   missing heartbeat withholds output. Wait for VN/DVL, pressure and GPS timing.
3. The operator deliberately releases kill/arms MANUAL for a controlled surface
   movement, using the normal controller procedures. The script never does this.
   Move steadily at >=0.5m/s where valid DVL bottom lock and exposed GPS are possible.
   Allow adequate clearance for the movement and stopping; this is not stationary
   compass alignment. Return to disarmed/kill after the movement.
4. The bridge aligns vectors automatically and freezes the learned reference for
   this bridge session. Before alignment it never initializes/publishes external
   position. The first accepted fix anchors corrected north/east; the Cube origin
   is not reset. GUI Waypoint page shows progress/rejection reasons.
5. Confirm Cube position/heading/depth are stable and agree with the intended
   physical/map frame. Then request checked supervisor activation without restarting
   the bridge or losing the alignment:

```sh
python3 tools/prepare_navigation.py --watch
# Once stationary, killed/disarmed, and Cube fusion has been checked:
python3 tools/prepare_navigation.py --commission
```

The commissioning service requires fresh healthy odometry/profile, acquired north
alignment, admitted GPS, near-zero surface depth, healthy Cube position and heading
consistency, disarm/kill/ESC-off, no active mission, and no supervisor fault. It
activates supervisor permission for this session. It does not arm, restore ESC
power, clear faults, start a mission or remove waypoint health checks.

6. Preview the intended geographic waypoint, verify its physical clearance and
   Abort availability, then deliberately start the short supervised mission. The
   existing mission captures surface start, dives to the selected depth, transits,
   surfaces, disarms and recovers qualified GPS. Approximate pool-image live targets
   remain disabled. Restarting the bridge requires a new alignment and reference.

## Alignment conditions and limitations

- Existing GPS admission must pass: pressure-exposed surface dwell, 3D fix,
  horizontal accuracy <=3m, DVL motion consistency, fresh VN and Cube-fix timing
  upper estimate <=0.35s with valid TIMESYNC. Receiver processing delay remains
  unknown; these are approximate time-matched measurements.
- Each fix is matched to VN/DVL history using the mapped Cube fix timestamp.
- GPS and horizontal DVL speeds >=0.5m/s; reported GPS speed uncertainty >0 and
  <=0.2m/s, with velocity-direction uncertainty screen <=10deg.
- Tilt <=15deg, angular rates <=5deg/s, vertical velocity <=0.15m/s, speed agreement
  within max(0.15m/s, 3 reported speed sigmas).
- At least 15 distinct fixes, >=5 continuous seconds, >=2.5m integrated travel,
  max 0.5s between accepted samples and <=5deg spread about circular mean.
- North offset equals GPS velocity direction minus VN-rotated DVL velocity direction.
  Sideways and reverse movement are supported mathematically; GPS course is never
  assigned directly as nose heading. Correct DVL mounting/body axes and VN mounting
  transforms remain prerequisites that GPS cannot independently certify.
- Qualification loss, inconsistent samples, inadequate movement or timing gaps
  discard the unfinished window. No automatic realignment occurs mid-mission.
  Underwater, the existing pressure gate rejects GPS and aligned DVL/VN continue.
- Relative-heading drift and dynamic magnetic interference remain possible. A
  learned reference is an estimate, not guaranteed surveyed heading. The provisional
  covariance model and independent mission-process-death safeguard remain review
  items before unattended operation. Normal operator Abort has been simulated.

## Verification and deployment status

Results and evidence are recorded below after final validation. Graey is offline
per user clarification. Tether SSH attempt timed out; no deployment, sensor writes,
odometry enablement, mission activation or vehicle mission occurred in this work.
Historical October13 VN RAM settings and calibration must be read again when online;
the teammate reports returning it to relative mode, not independently verified today.

### Results

- 130 current Python tests passed in ROS Humble/WSL, including lateral/reverse
  velocity alignment, stationary/uncertain/turning/duplicate/gapped GPS rejection,
  estimator initialization, armed-start output inhibition, commissioning rejection
  conditions, serial checksums/ACKs, calibration interruption cleanup and coverage.
- Waypoint and supervisor browser-script rendering tests passed. Python compilation
  and shell syntax checks passed before the final documentation update.
- Native ArduSub + production ROS bridge/supervisor + GUI/API known-north mission
  completed (waypoint-e57cfb6de6). This fixture starts with known synthetic north.
- Automatic north fixture injects +40deg into synthetic VN attitude while DVL body
  velocity and GPS are independently derived from simulator truth. Manual surface
  movement acquired -40deg correction with26 accepted samples in the final nominal
  run (waypoint-9c9e2890ab), then GUI/API mission completed all phases, rejected GPS
  underwater and recovered it after surfacing. Final synthetic endpoint error0.132m.
  This is deterministic fixture behavior, not a physical accuracy promise. Surface
  preparation itself traveled several metres; clearance is required beyond the
  minimum accepted2.5m collection threshold, including deceleration.
- Automatic alignment followed by operator Abort in TRANSIT passed with MANUAL
  verified, GPS absent and Cube pose available (waypoint-5718b6ee96).
- First automatic alignment run waypoint-c27cf69feb also completed. Evidence JSONs
  are under docs/evidence/vn-north-2026-10-14. They include harmless initial connection
  retries while the owned stack starts. UTC timestamps are recorded by the execution
  environment; this report uses the client's October14 local session date.
- One intermediate regression run had three errors because an adapter test fixture
  omitted the new armed-start field. The fixture was corrected and an armed/missing
  heartbeat test added; the full suite now passes. A mistakenly requested nonexistent
  browser test was replaced by the actual supervisor browser test; both actual
  browser tests pass. No failed vehicle calibration/deployment is represented as a
  successful live run. The hardware calibration procedure still needs bench testing;
  serial protocol/cleanup are mocked in its offline tests.
- The new activation service is exercised by pure gate tests and loads in the final
  native stack. Its positive live path depends on the real controller kill/disarm API,
  absent from this SITL fixture; it must be verified during deployment. Default SITL
  still precommissions synthetic hardware; the simulator does not prove that live
  commissioning has occurred.
