# Underwater mission MANUAL transition — 2026-10-17

Operator reports mission stopping in MANUAL underwater. Authorized local debugging
and fixes on `tether_toggle_beichen`; subsequent explicit answer: finish locally,
deploy later. No vehicle writes, restarts, parameter changes or physical testing.

## Evidence and limits

- `graey.local` did not resolve; permitted read-only tether SSH at 192.168.2.2
  timed out. No deployed version, current abort reason, DataFlash log, firmware
  failsafe or current RF/tether state was obtained. This session cannot identify
  the cause of the reported physical event conclusively.
- Production `WaypointRun.step()` requested MANUAL on **the first** unavailable
  navigation permission or freshness observation during DIVE/TRANSIT/SURFACE.
  This can abort on a transient asynchronous status interruption even when local
  Cube pose, EKF and bridge remain healthy. Reproduced in the decision regression.
- Explicit MANUAL mode requests can also come from real mission aborts, operator
  control, hardware kill handling and Cube firmware. Do not attribute every mode
  change to RF, GPS or a pilot without logs. No kill/failsafe suppression added.
- Earlier DVL dropout recovery changes are already committed locally/online,
  but were not deployed. An older installed runtime can still have the prior
  immediate DVL fault. Deployment must verify actual installed modules and GUI,
  not just a Git checkout or successful build.

## Fix

The waypoint adapter now independently observes bridge status. A moving mission
may pause for **at most 0.7 seconds** on a permission/status interruption only if:

- Cube heartbeat, local position/velocity, attitude and origin remain fresh;
  local data are finite and the Cube EKF position-health check passes.
- Independent bridge publication is <0.3 seconds old, healthy, transmitting,
  configuration verified and navigation-valid, with no bridge fault.
- Control watchdog/ESC permission remain valid (the isolated SITL fixture retains
  its existing simulation-only control checks).
- Same run ID, continuous fusion, same current reference, same horizontal Cube
  origin; no position jump, operator Abort, safety failure or stage timeout.

Capture a fixed current Cube north/east/down position as a temporary hold target;
replace the travel target once, retain GUIDED, do not advance stage/arrival during
the pause. This is supported by a currently healthy Cube/bridge, not a blind hold
using stale position. On recovery, immediately restore the original stage target
and reset arrival dwell. On deadline exhaustion, abort and request MANUAL as
before. A different run/reference/policy, real bridge/EKF fault, lost watchdog,
kill, disarm or observed pilot mode change does not receive this grace period.
Unknown independent evidence defaults to no pause authorization.

The GUI shows the temporary hold reason. Aborts now preserve supervisor state and
reason, readiness/reference state, bridge fault/DVL recovery details, watchdog
failure, Cube armed/mode state and recent Cube STATUSTEXT messages. Every mission
mode request logs `MISSION_MODE_REQUEST` with run, stage and reason. Observed
mode changes are labeled 'Vehicle left GUIDED or disarmed; yielding control',
since a heartbeat alone does not prove who changed mode. A real failure still
ends the mission; this is not permission to continue indefinitely on bad data.

## Tests

- 205 Python regressions passed at initial verification, including transient hold,
  original-target resumption, fixed hold position, expiration, frame/reference/
  watchdog/estimator failures and pilot takeover.
- Node waypoint GUI regression passed with pause display.
- Native GUI-integrated ArduSub test `waypoint-ed1ef46685` passed all stages through
  COMPLETE with a 0.5m dive, GPS outage, a 0.35s injected permission interruption
  and brief invalid DVL reports underwater. Pause was observed, original mission
  resumed, surfaced/disarmed and GPS recovered. Final simulated target error
  0.097m; not a physical accuracy claim. QGC forwarding disabled, ROS localhost
  domain73. Native fixture does not test real hardware kill/RF arbitration.
- A separate native operator-Abort run is recorded in the final evidence.

Final operator-Abort run `waypoint-aa10d56615` passed: operator request while
underwater still changed GUIDED to MANUAL and emitted the new abort context,
including Cube STATUSTEXT. 205 Python tests and Node waypoint GUI checks passed.
The pre-change/post-change pure replay gave identical healthy pose/reference
inputs and one false permission publication: committed old logic returned
`ABORTED + manual`; new logic returned `TRANSIT + fixed current-position target`.
Summary and native results: `docs/evidence/underwater-manual-2026-10-17.json`.

## Deployment / next physical event

Not deployed. While killed/disarmed, later synchronize the toggle checkout, build
and verify actual installed waypoint/bridge/supervisor modules and served HTML.
Existing navigation preparation may need renewing after a restart.

If MANUAL occurs again, retrieve the run's `waypoint-*.log`/JSON and Cube .BIN:
`MISSION_MODE_REQUEST` proves the waypoint node requested a change and states why;
absence does not prove a pilot did it. Compare FC STATUSTEXT, mode changes,
watchdog kill/inhibit and `WAYPOINT_STAGE.abort_context`, then inspect firmware
failsafe evidence. Do not disable firmware failsafes or automatically force
GUIDED/arming to conceal the mode change.
