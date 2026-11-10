# Competition waypoint and pipeline route — 2026-11-08

Local implementation moved to newly merged `main` (base `26e9bd4`) on 2026-11-08 at the user's explicit request. Ruth's GUI routing, mission and sonar changes are retained. No vehicle deployment, live parameter
changes, arming, or motion were performed in this session.

## Approved behavior

- Keep the existing navigation GUI, satellite map and coordinate entry/recording.
  Center on the vehicle. The current `cube_ned` profile displays Cube
  `GLOBAL_POSITION_INT` and `ATTITUDE`, matching the vehicle telemetry QGC uses.
  This is not screen sharing or copying QGC's imagery cache. World satellite
  tiles still require internet; saved Miramar/pool options remain available.
- Plot and reorder waypoints, pipeline starts, pipeline ends and reference buoys.
  Buoys never command motion. A start/end pair must be consecutive executable
  points; reference buoys may appear between them.
- Start uploads the entire route to the existing onboard waypoint process.
  Browser/Wi-Fi loss does not cancel it. Points and their order are saved in
  browser storage; the executing route is also included in onboard status/logs.
- Surface legs use the newest GUIDED implementation: hold/orient within 3 degrees,
  then position-only travel. There is no continuous Jetson yaw P-controller or
  speed taper on ordinary surface legs. Existing 1.5 m estimated arrival radius
  and 1 s settling dwell remain.
- At a pipeline start, request recording in the background; hold the planned start
  and face the planned end. Descend to 4 m pressure depth, then command the end
  position while holding that depth and fixed start-to-end yaw. Surface at the
  end before continuing. Final route completion stops and disarms as before.
- The latest user correction supersedes the earlier recording-failure choice:
  **recording failure is logged, never a reason to stop the segment or surface
  early. Continue to the end at 4 m, then surface.** Failed camera processes retry
  every 5 s into new video parts; sonar connection/setup attempts retry.
- Individual GPS/DVL/VN loss does not independently abort. Existing estimator
  prediction and sensor admission remain. Total sensor loss, bridge faults,
  loss of onboard control, physical kill, or deliberate pilot takeover remain
  distinct from recording faults or loss of the operator network.

## Code path

1. `tools/waypoint.html`: map editor, point types, ordering, coordinate/GPS tools,
   persisted route, Start route, status and Abort.
2. `api/gui/waypoint_api.py` and `api/navigation/route_plan.py`: normalize/validate
   the ordered plan. Ordered competition routes are not constrained by the old
   single-target 20 m limit. Single-target behavior is retained.
3. Existing `api/gui/gui_node.py` waypoint launcher: starts one onboard mission
   process with the full JSON plan. It does not stream flight commands from the
   browser. Existing run IDs and duplicate-start handling remain.
4. `api/navigation/competition_route.py`: sequences surface legs and pipeline
   orient/descent/transit/ascent. Uses `guided_surface_mission.py` for surface legs.
5. `api/navigation/waypoint_mission.py`: actual MAVLink transport and status/logs.
   Position-only packets use mask 3576. Position plus absolute yaw packets use
   mask 2552 for orientation and pipeline traversal. Cube GUIDED owns control.
6. `api/navigation/pipeline_recording.py`: background recording worker. Uses the
   existing host sonar launcher (8096) and sonar GUI settings/record API (8095),
   not a competing sonar serial owner. During pipeline acquisition it sets range
   6 m and sector 225–315 degrees in that API's existing convention. Previous
   range/sector are restored when segment recording stops.
7. `api/vision/bottom_recorder.py`: identifies DOWN camera by repository-established
   MxId `14442C10C1B6BED200`, uses USB HIGH as `cam_pair.py` does, records MJPEG AVI
   and per-frame host/device timestamps. It never silently chooses the front camera.

Four metres means depth below the pressure surface baseline, not DVL altitude.
The local target is `Cube local down - pressure depth + 4`. If pressure drops out,
use Cube local down against the last pressure-derived surface reference and keep
travelling. Without any established depth reference, hold the last vertical target
rather than invent zero depth. `depth_source` reports which source is in use.
During pipeline control, GPS admission remains pressure-aware.

## Recording and connection behavior

Camera files: under the waypoint log directory,
`survey-<run-id>/segment-NNN/camera-part-NNN/`: `bottom.avi`, `frames.jsonl`,
`camera-status.json`. Segment folder also contains `camera.log` and recording
status. Sonar sweeps/nav/settings remain in the existing host `sonar_data/rec_*`
folders with the mission-ID prefix and segment number. `PIPELINE_RECORDING` events
in the waypoint log record recorder health transitions even when the GUI is offline.

Worker HTTP/camera operations never block the 10 Hz mission control loop.
Readiness checks describe recording health (fresh frame heartbeat, advancing sonar
sweeps and settings readback); they do not gate mission progression.

Existing watchdog `autonomous()` exempts GUIDED from RF/tether loss. Its Cube-link
freshness and physical-kill paths are retained. This is verified locally against
the actual watchdog methods. Cube firmware failsafe parameters and RC receiver
behavior are **not freshly verified on Graey**; inspect these during deployment.
Jetson mission heartbeat and neutral pilot-input transmissions remain onboard, so
normal mission operation does not require a laptop heartbeat or joystick stream.

## Validation and limitations

- 89 focused Python tests and the waypoint JavaScript suite passed, including
  ordered route behavior, recording failure continuation, sensor loss policy,
  RF/tether loss in GUIDED, physical kill, recorder configuration/restoration and
  surface mission regressions.
- Native ArduSub 4.7.2, ROS, GUI API and actual mission process:
  `sitl/logs/competition-b125ab5a3c/validation.json` passed complete pipeline plus
  final surface waypoint, stopped/disarmed. Maximum pressure depth 3.94 m; final
  pressure depth approximately 0.01 m. This run preceded the final non-fatal
  recorder policy and planned-start refinement; subsequent final-policy test is
  recorded below.
- Final non-fatal recorder-policy native test:
  `sitl/logs/competition-8197f21e64/validation.json` passed with recording failure
  injected throughout: full pipeline at maximum depth 3.95 m, surface at its end,
  final waypoint, stop and disarm. The failure remained in the final status/log.
  Subsequent display-order metadata and pressure-dropout fallback changes were
  covered by the final Python/JavaScript regression run; the pressure-dropout
  branch itself has not had a native sensor-fault injection test.
- Initial `competition-7f04169c5e` reached the pipeline and was surfacing but hit
  the 360 s test deadline at about 0.60 m depth. It is a failed validation, not a
  completed mission. The follow-up test allows 600 s. `competition-2a1847a52b`
  was a setup failure: temporary SITL executable missing. It did not run a mission.
- QGC forwarding disabled and ROS isolated to localhost/domain 73. Native tests
  explicitly stub recording; no real camera or sonar acquisition is proved.
- Browser visual inspection through in-app localhost failed with connection
  refused. JavaScript/DOM fixture tests passed; do not claim a screenshot review.
- Before live deployment: verify current code/branch, Cube failsafes/profile,
  pressure baseline/depth tracking, DOWN camera availability (including competing
  camera processes), host sonar service and write permissions, and actual saved
  recordings. No service restart or parameter change was performed here.

## Bugs corrected while implementing

- Orientation hold target/yaw now follow an accepted heading-frame reset along
  with other cached targets; previously the newly added orientation helper did
  not participate in that transformation.
- Recorder job identifiers fit the existing sonar recorder's 40-character slug,
  so acknowledgement can match the correct run instead of waiting forever.
- Route execution consumes paired ends and skips buoy annotations without
  disarming between points; only the final completion uses the old disarm stage.
- Pipeline target/status uses the end coordinate during survey, and displayed
  route numbering accounts for reference-only buoy entries.

Fix demonstrated bugs found during testing and record failed runs honestly on
every navigation change, as requested by the user.
