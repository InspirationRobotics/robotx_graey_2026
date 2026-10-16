# Satellite map and waypoint control — 2026-10-15

## Scope and deployment status

Work is local on `tether_toggle_beichen`. User requested finishing locally and
deploying later after tether SSH did not respond. No vehicle connection, parameter
write, arming, mission, service restart or deployment followed that instruction.
Main and the unrelated untracked `docs/nav_lgoic.md` are untouched.

## Satellite map

The waypoint page uses geographic coordinates for both map clicks and Cube/GPS
overlays. It does not translate the old pool screenshot into navigation commands.
The old screenshot remains view-only on a live vehicle.

Current World Imagery has considerable tree cover over this pool. Visually
inspected an official Esri Wayback tile with a clearer visible pool outline and
made that dated release the default. Added an imagery selector, Find pool and
Reload imagery. Find pool changes the viewport only, not the navigation origin,
heading, GPS reference or selected geographic target. Changing imagery clears
the selected target. Map/depth/imagery inputs are locked while a Start request or
mission is active. Offline geographic grid remains available.

Official source configuration:
https://s3-us-west-2.amazonaws.com/config.maptiles.arcgis.com/waybackconfig.json

Selected entry: World Imagery (Wayback 2023-08-31), tile release 64776:
`https://wayback.maptiles.arcgis.com/arcgis/rest/services/World_Imagery/WMTS/1.0.0/default028mm/MapServer/tile/64776/{level}/{row}/{col}`.
The pool tile redirects to canonical archive 47963. Release date is not photo
acquisition date. Current imagery remains selectable. Browser internet is needed.
Imagery registration and GPS errors remain independent; this is not a survey and
does not establish safe pool-wall clearance by itself.

## Demonstrated pilot command conflict

`kill_switch.py` runs pilot arm handling for fresh RF channels under both RF and
TETHER watchdog selections. Its previous three-second arm/disarm retry could
continue after a waypoint mission claimed control. An arm retry also sent MANUAL
or STABILIZE before ARM, potentially undoing GUIDED. The mode-switch callback
already protected mission ownership; the arm callback did not.

Now a new mission claim cancels old retries and adopts the current SB level.
While GUIDED/autonomous or a mission lease is active, static SB levels cannot arm,
disarm or force a flight mode. A NEW SB high-to-low edge still retries DISARM.
SA kill, manual-operation connection-loss inhibition, GUI Abort and externally
requested pilot takeover remain effective. No firmware failsafes were disabled.
This fixes a reproducible software conflict; it does not prove this was the sole
cause of the last live failure. Firmware RC/GCS failsafes, QGC commands and source
changes still need correlated live evidence if an interruption recurs.

Status now includes fresh arm/mode switch PWM and pending retry duration.
Waypoint stage logs retain those safety fields, controller source, inhibit,
kill/ESC state and Cube mode/armed state. Missing safety telemetry remains null;
it is not invented from simulator data.

## Validation

- 173 Python regression tests passed. Six new tests execute production pilot
  callbacks with mocked transport: static disarm under RF/TETHER, stale arm retry,
  claim before GUIDED, deliberate disarm edge, SA kill and manual connection loss.
- `node test/test_waypoint_page.cjs` passed. Includes projection round-trip,
  unloaded tiles, geographic target submission, archive/current URL choice,
  Find pool, reload, selection reset, readiness and idempotent Start retry.
- Isolated native ArduSub SITL, production navigation/mission nodes and actual GUI
  API completed WAIT_SURFACE → PREPARE → ARM → DIVE → TRANSIT → SURFACE → DISARM
  → RECOVER_GPS → COMPLETE. Requested depth 0.5 m; target 1.5 m north/1 m east.
  GPS outage and valid underwater Cube display were observed; repeated Start was
  idempotent. Final simulated horizontal error 0.153 m. QGC forwarding disabled.
  Evidence: `docs/evidence/waypoint-map-nominal-2026-10-15.json`.
- Simulation recorded wall-clock adjustments, but no independent monotonic
  scheduling gaps above 0.3 s. Startup connection-refused retries preceded API
  readiness; shutdown MAVProxy reconnect output followed simulator termination.
- The native fixture does not run the hardware pilot/kill node. RF/tether
  arbitration coverage is the production-callback regression, not a claim of
  complete hardware-in-loop validation.
- Full-page browser preview at localhost:18190 timed out in the available browser.
  Satellite tiles were visually inspected directly; complete page visual QA is
  still pending. The page JavaScript and real GUI API were exercised separately.
- The first Abort-test launch was blocked by an automatic approval-review timeout;
  the one permitted retry launched successfully. Result recorded below.
- Abort regression passed in isolated native SITL through the GUI API. During
  underwater TRANSIT with GPS unavailable, Abort produced ABORTED and Cube mode
  19 (MANUAL), preserving arming for pilot takeover as designed. It does not
  automatically surface or disarm. Evidence:
  `docs/evidence/waypoint-map-abort-2026-10-15.json`.

## Next vehicle session

Deploy this branch with Graey killed/disarmed, preserving unrelated Jetson work.
Restart the pilot control node as well as the waypoint process; a static page
refresh alone does not install the control fix. Re-Prepare if navigation restarts.
Confirm selected watchdog and kill/ESC status, surface depth baseline, DVL lock,
relative VN north alignment, Cube position/heading freshness and geographic target.
Keep RF/TETHER selection stable during a mission. First verify no unsolicited
MANUAL/disarm with stationary SB; separately verify deliberate disarm, SA kill and
Abort. Then perform the bounded wet waypoint test with clearance and capture
Cube BIN, pilot status and waypoint stage logs. Do not remove blockers merely to
obtain a clickable Start button.
