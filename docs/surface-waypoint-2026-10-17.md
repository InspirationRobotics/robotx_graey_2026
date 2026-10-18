# Surface waypoint mission — 2026-10-17

User changed the mission to surface travel and stopped compass alignment work.
This change is repository software only; no deployment, parameter change,
arming or physical mission was performed for this request.

## Behavior

The live waypoint page submits `mode=surface`, depth=0. Missing mode also defaults
to surface, including old clients supplying a dive depth. The previous underwater
controller remains available only through explicit `mode=dive` for regression;
the GUI has no dive control. The geographic point selected by the user is retained.

1. Operator prepares pressure baseline, healthy DVL/VN, north reference and Cube
   external-navigation estimate. RF/tether, kill and ESC permission remain separate.
2. GUI Start launches the mission. It measures 20 seconds of accepted stationary
   surface samples. GPS-only gaps up to four seconds retain earlier valid exposure,
   but rejected time is excluded; the window is limited to 30 seconds. Movement,
   DVL/VN failure, poor accuracy or excessive scatter still invalidate the window.
3. Supervisor receives SURFACE_NAV intent, captures a per-run surface reference
   only with admitted GPS and validated heading, and verifies the continuous
   estimate. This is not EKF source switching or an EKF-origin reset.
4. Mission freezes the Cube/pressure vertical offset at surface depth zero,
   transforms the selected latitude/longitude to Cube north/east, and confirms
   GUIDED/arm under existing operator authorization and control safety checks.
   Current admitted GPS is required before departure.
5. It enters TRANSIT directly: no DIVE or ascent stage. Cube controls toward the
   fixed north/east target and surface down target, retaining the starting yaw.
6. Existing bridge admission rejects stale timestamps, missing 3D fix, excessive
   reported horizontal uncertainty, GPS velocity inconsistent with DVL, and GPS
   displacement beyond independent DVL path bounds. The bridge continues VN/DVL
   odometry during GPS rejection. Rejected GPS never moves the geographic target.
7. Healthy DVL/VN can continue for at most ten seconds or three metres of cumulative
   estimated horizontal travel without admitted GPS. Exceeding either requests
   MANUAL pilot takeover. Returning GPS must pass normal qualification again.
   These initial conservative limits are not a guarantee of ten safe seconds.
8. Arrival requires Cube distance <=0.35 m, horizontal speed <=0.15 m/s, pressure
   depth -0.10..0.12 m, current admitted GPS within 1.5 m of the selected target
   and reported GPS accuracy <=2 m, held for one second. Then disarm, MANUAL and
   surface GPS confirmation complete the run.

All real sensor/frame faults, Cube jumps, pressure overshoot/reference changes,
pilot mode/disarm, control-link inhibit, kill and Abort remain effective. Brief DVL
recovery uses the existing bounded hold; this request does not make IMU alone an
unlimited position source. No compass alignment was enabled.

## Code ownership

- `tools/waypoint.html`: map selection, explicit surface plan, instructions/status.
- `api/gui/waypoint_api.py`: existing single-use plan/frame validation, unchanged.
- `api/navigation/waypoint_logic.py`: default surface plan, direct transit,
  GPS-outage travel/time bounds, confirmed surface arrival.
- `api/navigation/waypoint_mission.py`: telemetry/bridge admission evidence,
  pressure target mapping, MAVLink commands, diagnostics.
- `api/navigation/continuous_supervisor.py` and `navigation_supervisor.py`:
  SURFACE_NAV reference capture and permission without per-packet GPS resets.
- `api/navigation/surface_measurement.py`: bounded preflight GPS-gap retention.
- Existing `fused_nav_bridge.py`, `bridge_estimator.py`, `gps_admission.py` and
  `gps_quality.py`: sensor propagation and GPS rejection, unchanged by this request.

## Validation and limitations

232 Python tests pass, including surface-only sequence, GPS-loss time/path bounds,
GPS-confirmed arrival, reference health gates and GPS-jump regressions. Node GUI
checks pass and assert that Start sends an explicit surface plan with depth zero.

Isolated native ArduSub/ROS/GUI API SITL, localhost domain 73, QGC forwarding off:

- Initial `waypoint-5361137bfe`: FAILED safely in WAIT_SURFACE, never armed.
  Intermittent fix-timing rejections repeatedly erased its whole stationary
  window. This demonstrated the preparation brittleness addressed above.
- `waypoint-d64a3831ec`: PASS. Two-second GPS outage, automatic qualified recovery,
  surface TRANSIT -> DISARM -> COMPLETE, no DIVE/ascent. Final simulated Cube
  horizontal error 0.194 m. This run predates the final cumulative-path bound and
  <=2 m GPS-arrival-accuracy refinement; final regression tests and the following
  runtime test include those refinements.
- `waypoint-c493141974`: PASS for the expected abort. Twelve-second injected GPS
  loss triggered the ten-second/travel bound and confirmed Cube MANUAL takeover.
  It did not falsely declare arrival despite being near the target. Sim teardown
  disarmed the simulator; on a live abort, operator takes control and must manage
  propulsion. No autonomous return is promised.

Evidence JSON is stored under `docs/evidence/surface-waypoint-*2026-10-17.json`;
full runtime logs remain under `sitl/logs/`. These are simulated estimate errors,
not surveyed physical accuracy. No live heading, GPS, map, buoyancy or motor
response validation was completed. Low GPS scatter cannot detect a stable bias,
and a 1.5 m GPS arrival radius cannot guarantee sub-metre placement in a pool.

## Next live test

Deploy only when requested and verify one bridge, fresh ExternalNav profile and
kill/disarm/ESC-off before restart. Float stationary with DVL bottom lock; Prepare
and verify the existing north reference. Confirm mapped physical position and
heading independently. Select a short clear surface target, enable propulsion
deliberately, supervise Start and physical kill. Save mission trace and Cube log.
Stop on wrong-direction movement; do not add arbitrary offsets to force agreement.

2026-10-17 surface waypoint DEPLOYED via tether, code439f5c8, backup
/home/graey/deployment-backups/20261017-surface-waypoint-220725. Build/232installed
Python tests passed; installedtrackedruntime byte-equal; served surface GUI and one
bridge verified. User requested noM9Ncompass: COMPASS_ENABLE1->0,rebootedCube;
allCOMPASS_USE*=0 and completeExternalNavprofile verified. M9NGPS stays enabled.
No compass alignment code deployed. Saved calibration/orientation6 retained dormant.
Postrestartkilled/disarmedESCoff, freshVN/DVL, VNflip180true/yawoffset0unchanged;
bridgeenabledfalse/baseline0/northunaligned, supervisorObservation: Prepare+existing
north alignment/commissioning needed. No arming/mission performed. PreservedJetson
untracked files. Evidence docs/evidence/surface-waypoint-postdeploy-2026-10-17.jsonl.

2026-10-17 user requests shorter/less brittle surface check. Surface mission now
requires5s ACCEPTED exposure (not20),min10distinctfixsamples,15srecentwindow.
Brief nonfatal DVL/motion/accuracy glitches pause up to1.5s;GPS-only requalifying
up to4s. Rejected intervals nevercount; segmentIDs prevent bridging across short
invalidsamples. Currentvalidinput needed to pass; realbridgefault/staleVN/lost
surface state clearhistory. GPS/velocity/rate/scatter safety thresholds unchanged.
SITLwaypoint8e53981a9d passedGUIproductionmission+2sGPSgap,qualification5.201s,
surfacecomplete. Userconfirmedkilled/disarmedESCoff fordeployment;freshcheckrequired.
