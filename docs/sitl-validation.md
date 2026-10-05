# Graey ArduSub software-in-the-loop

## Purpose and limits

This local-only scenario boots ArduSub SITL with Graey's EKF source parameters,
runs the repository's real `nav_ekf_bridge`, active `navigation_supervisor` and
navigation GUI, and supplies deterministic synthetic VN-100, DVL and depth
topics. It starts surfaced with GPS selected, saves the Jetson-owned dive
reference, switches to Underwater sources before descending to 0.5 m, disables
simulated GPS, navigates one GPS-defined target using external navigation, then
surfaces, restores simulated GPS and returns to Surface GPS sources.

This does not simulate real water drag, pool walls, tether effects, DVL bottom
lock, IMU bias/noise, GNSS multipath, or the physical sensor drivers. The
synthetic IMU and DVL are derived from SITL telemetry to exercise the production
ROS/MAVLink interfaces; they are not independent sensor truth. No hardware,
Jetson service, or physical Cube is used. `sitl_graey.parm` is SITL-only and must
not be loaded on a vehicle.

## Pool/map reference

The street name was geocoded with the user's permission through OpenStreetMap
Nominatim. Its road result is approximately **32.9240586, -117.0385389**. This
is the road's reference point, not a verified lab, backyard, or pool coordinate.
The default waypoint is a demonstration point 1.5 m north and 1.0 m east of that
reference. Set the launch home and waypoint offsets to surveyed pool values
before treating the map as the backyard pool. The waypoint is shown in purple on
the GUI's offline Grid and online Streets layers; the live Cube global position
and raw GPS are shown alongside it. MAVLink does not upload the demonstration
target as a QGC mission.

## Runtime setup

The supported local host for this workspace is the `Ubuntu-22.04` WSL2 distribution.
Use `wsl.exe -d Ubuntu-22.04`; the default `Ubuntu` distribution is different.
Inspect `wsl.exe --list --verbose` before diagnosing simulator processes. Install
ROS 2 Humble, colcon, `pymavlink`, `MAVProxy`, and `pyserial` there. The existing
Jetson `docker/Dockerfile` is ARM64 and is not the x86 WSL simulator image. Docker
is not required when the WSL ROS 2 packages are available. The simulator
executable can be built from ArduPilot or supplied through `SITL_BIN`; by
default the launcher uses the existing local ArduSub 4.5.7 artifact at
`output/gps-sitl-2026-10-02/ardusub`.

From the repository directory in WSL:

```bash
cd robotx_graey_2026
./sitl/run.sh
```

Open `http://localhost:8090/navigation`. Choose **Streets (online)** to see the
configured waypoint and simulated position over online OpenStreetMap tiles.
QGC output is disabled by default. The physical Cube and SITL both use MAVLink
system ID 1, so sending both to one QGC instance can make its single vehicle
marker alternate between the real location and the SITL home. For a separate,
SITL-only QGC instance, explicitly set `QGC_HOST=<Windows-host-address>` before
running; do not point it at the QGC instance connected to Graey. Stop the stack
with Ctrl-C. `./sitl/stop.sh` is a fallback for stale process IDs.

Configure a known pool home and one target offset before launching:

```bash
HOME_LAT=32.9240000 HOME_LON=-117.0385000 \
WAYPOINT_NORTH_M=2.0 WAYPOINT_EAST_M=1.0 ./sitl/run.sh
```

To exercise a different GPS target point, change its local N/E offset. The
runner converts that target into latitude/longitude and publishes it with the
mission intent so the GUI can display the same point.

## What to inspect

- GUI navigation map: raw GPS, Cube global pose, dive-start reference, and
  configured mission waypoint.
- Navigation Supervisor page: state, reason, requested and confirmed EKF source,
  saved dive reference, and readiness.
- `sitl/logs/ros.log`: scenario phase transitions and ROS node errors.
- `sitl/logs/mavproxy.log`: ArduPilot messages and MAVLink routing.
- `sitl/logs/ardusub.log`: simulator startup, Lua source reporter, and EKF.
- `sitl/run/`: isolated SITL EEPROM and Lua script directory.

## Regression / bug log

### Setup and implementation — 2026-10-03

- Initial package install attempt named `python3-pymavlink` and `python3-pyserial`,
  which are not Ubuntu 22.04 package names. Fixed by installing `python3-pip`
  and the MAVLink tools from pip; `pyserial` was already available through apt.
- First geocoder command used a malformed User-Agent header and was rejected by
  PowerShell. Fixed the header and obtained the street-level road coordinate
  above. The returned point is explicitly treated as approximate.
- First launch stopped before starting SITL because `set -u` conflicted with an
  unset `AMENT_TRACE_SETUP_FILES` read by ROS Humble's setup script. Removed only
  unset-variable strictness from the wrapper; shell syntax and package build
  passed after the fix.
- The first complete launch returned before its nodes were ready. The wrapper
  had no child-exit diagnostics and MAVProxy loaded modules unnecessary for this
  telemetry-only test. Added synthetic SITL clock mode, minimized MAVProxy's
  modules, and made the wrapper print each component log on exit; retest follows.
- The GUI returned 404 for `/navigation` in WSL because its static asset root was
  hard-coded to the Jetson container path. Changed it to resolve `tools/` beside
  the installed package, which preserves the Jetson path and supports local SITL.
  SITL also showed repeated `SET_MESSAGE_INTERVAL` denials for message 251; that
  ID is Lua-pushed `NAMED_VALUE_FLOAT`, not an interval-configurable stream, so it
  was removed from the request list.
- The first mission run reached its 90-second `WAIT_SURFACE_GPS` timeout. Added
  change-triggered supervisor-state diagnostics to the SITL runner so the next
  attempt can identify which readiness gate is missing instead of timing out
  without a reason.
- Live diagnostics found a circular startup dependency: the synthetic DVL
  waited for EKF local position, while the EKF needed DVL ODOMETRY before it
  would publish local position. The sensor driver now seeds attitude and
  horizontal velocity from ArduPilot's SITL-only `SIMSTATE` ground truth, then
  uses local NED only for the simulated depth/vertical channel after startup.
  The navigation page route and target pin API both returned HTTP 200 after the
  asset-path fix; end-to-end mission retest follows.
- A second successful end-to-end pass reached `COMPLETE` with Surface GPS source
  and a final raw GPS fix at 32.9240747, -117.0385300, about 0.3 m from the
  configured demo waypoint. A separate five-second `SIM_GPS_DISABLE=1` injection
  produced GPS fix type 1 while `SIMSTATE` and `LOCAL_POSITION_NED` continued;
  the supervisor retained Surface GPS and readiness returned after GPS was
  restored. Pausing the synthetic sensor publisher made the supervisor report
  `Unavailable: DVL velocity, DVL validity, VN-100, Navigation bridge` and
  `navigation_ready=false`; resuming it restored Surface GPS/readiness.
- Test suite caught a regression in the pressure-message callback because its
  mocked settings lacked the new `depth_topic` key. Changed the callback to use
  a safe default for legacy/test instances and added an isolated depth-topic
  regression case. The final WSL build and all 47 unit tests then passed.

### Final end-to-end run — 2026-10-03

- Result: **PASS**. The run recorded `Underwater → Surface GPS → Underwater →
  Surface GPS`, captured the surface fix/reference, and changed source sets only
  after the supervisor reported a healthy estimate.
- Dive depth reached **0.502 m** for a **0.500 m** target. The run verified
  `SIM_GPS_DISABLE=1` and GPS fix type **1** before advancing to the waypoint.
- The underwater waypoint was reached with **0.248 m** horizontal error at
  **0.490 m** depth. GPS was restored; the surfaced supervisor returned to
  **Surface GPS** with fix type **6**. Final GPS was 32.9240728, -117.0385269,
  about **0.2 m** from the configured demonstration waypoint.
- A separate five-second controlled GPS outage observed fix type 1 while SITL
  continued emitting `SIMSTATE` and `LOCAL_POSITION_NED`. Pausing the synthetic
  sensor process caused `navigation_ready=false` with explicit stale DVL/IMU/
  bridge reasons; resuming it restored readiness. Neither probe touched vehicle
  hardware.
- `/navigation` and `/api/navigation` returned HTTP 200 from WSL and Windows.
  The GUI API exposed raw GPS, Cube global/local pose, supervisor status and the
  configured waypoint. The local GUI is at `http://localhost:8090/navigation`.
- Machine-readable results and full logs are in the ignored runtime folder
  `sitl/logs/` and can be regenerated with `./sitl/run.sh`. The final run leaves
  local SITL and the GUI running; use `./sitl/stop.sh` or Ctrl-C to stop them.

### QGC vehicle-marker isolation — 2026-10-03

- During diagnosis, the live GUI's fresh raw GPS and Cube global-position
  samples matched, while the supervisor was in Observation mode and not
  requesting source changes. The recorded road geocode used as SITL home is
  approximately 49 m from that live position, matching the reported second
  marker location.
- The initial process check incorrectly inspected the default `Ubuntu`
  distribution. A subsequent check explicitly targeting `Ubuntu-22.04` found
  ArduSub SITL running with system ID 1 and the road home coordinate, plus
  MAVProxy PID 2103 actively forwarding to Windows QGC at 172.31.16.1:14550.
  Sent SIGTERM to that verified simulator MAVProxy process; its launch wrapper
  cleaned up the SITL/ROS stack. Follow-up process and UDP-socket checks found
  no remaining SITL processes or 14551–14559 listeners in that distribution.
  Graey's live HTTP telemetry remained fresh with matching GPS/global positions.
  The QGC screen itself was not independently observed after cleanup.
- Changed the local runner so QGC telemetry is opt-in via `QGC_HOST`; it no
  longer automatically sends same-system-ID SITL packets to the normal QGC UDP
  14550 listener. This prevents the local SITL runner from creating the
  duplicate marker on future runs. It does not change live Cube parameters or
  the Jetson.

Pool accuracy remains limited: geocoding returned a road reference, not a
verified lab or pool coordinate. The default waypoint is a demonstration offset
of 1.5 m north and 1.0 m east. Supply surveyed pool values before interpreting
the map as an exact backyard track.

Bug closure: every bug found during testing must be fixed or recorded here with its status before the run is considered complete.
