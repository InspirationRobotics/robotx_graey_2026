# Waypoint GUI and continuous mission audit — October 12, 2026

Local work only on tether_toggle_beichen. No Jetson access, deployment, live
parameter changes or main changes in this session. Prior live readiness is not
assumed. Supplied sonar notes imported with provenance in the consolidated history.

## Visual changes

Waypoint page has clearer layout, status/blocker explanations beside Start,
zoom controls, metre scale, Cube track, start-to-target route and distinct marker
label positions. Approximate pool registration is explicitly labeled. Removed the
old dark icon-cover label; the static screenshot icon area is covered with adjacent
image texture, a display treatment only, not new survey data or geographic evidence.
Native SITL's old start was about54m north of the supplied pool anchor, making the
fit include a tiny pool and a distant vehicle. Configurable default simulation
home is now the dated Oct11 raw-fix location near the pool, not the old street point.
Browser inspected the actual running page in geographic and pool views, including
vehicle marker, surface start, target and mission stage during transit/ascent.

## Start blockers audited

Start requires a current preview (30s), fresh telemetry, healthy Cube location and
heading, active commissioned supervisor/profile, surface state and no supervisor
fault. The API checks global/local agreement, origin, distance0.3–20m, preview
session/frame/movement and single-mission ownership. Live execution additionally
requires the explicit server commissioning setting and control/ESC permission.
Mission then captures the reference and checks expiring readiness throughout.

These checks do not inherently prevent a valid mission: integrated continuous
SITL completed through the production GUI HTTP API and production waypoint node.
Preview occurs before a mission run ID exists, so navigation_ready must not be
required there: supervisor waits for mission intent at that stage. UI likewise
does not introduce that circular requirement. Duplicate Start remains idempotent.

Live **pool-image** starts remain explicitly blocked because its registration is
approximate; geographic coordinate targets are the commissioned live route. This
is now visible in the UI rather than unexplained disablement. This audit does not
authorize labeling approximate registration surveyed or bypassing that check.

## Demonstrated problems and fixes

- Existing integrated SITL exercised the legacy source-switch bridge, not the new
  implementation. Added continuous mode with exact external-only profile and
  mutually exclusive bridge launch. QGC forwarding stays disabled.
- Simulator fixture now uses independent SIM_STATE attitude/velocity and depth;
  it does not recycle Cube estimated position/velocity into the new bridge.
  Synthetic depth is restricted to explicit isolated SITL scope. Production
  Bar30/SCALED_PRESSURE2 handling is unchanged.
- Native simulated pressure noise repeatedly broke shallow qualification in
  diagnostic runs. Independent truth-depth fixture avoids confusing simulator
  barometer defaults with tested physical pressure accuracy. It explicitly
  references the owned simulation's initial disarmed surface altitude.
- WSL UTC steps caused synthetic ROS stamps to regress and a real freshness
  fault during ascent. Isolated synthetic sensors and their freshness check now
  use the same monotonic clock. Vehicle timestamps remain unchanged.
- 50ms bridge receive polling conflicted with the50ms TIMESYNC RTT gate. Receive
  servicing now runs at10ms; odometry and status retain20Hz rate limits. No RTT
  or GPS-quality threshold was weakened.
- MAVLink receive drain had no work bound. Added256-message/20ms budget so busy
  streams yield to ROS timers; regression uses an endless mock producer.
- Shared node wrapper previously forced exit0 in finally, hiding callback
  exceptions as clean exits. Unexpected exceptions now print traceback and exit1;
  KeyboardInterrupt stays normal shutdown. A failing callback test verifies this.
  An earlier supervisor clean exit could not retrospectively be diagnosed from
  its suppressed traceback; do not claim a specific exception cause was proved.

## Validation

159 Python regression tests pass. Node waypoint rendering/readiness/zoom and
supervisor rendering tests pass. ROS package built during every integrated run.
Legacy baseline completes. Continuous run9 completes with every expected phase:
WAIT_SURFACE, PREPARE, GUIDED, ARM, DIVE, TRANSIT, SURFACE, DISARM, RECOVER_GPS,
COMPLETE. GPS outage observed underwater; Cube estimate/heading remained available;
surface GPS recovered; duplicate Start returned the original run.
Final repeat on the finished code also passes with simulated final horizontal
target error0.11136m. This is deterministic simulation, not a physical accuracy
claim. Evidence:waypoint-6b2fec3cda.json.
Continuous Abort run passes: GUI API abort during underwater TRANSIT is recorded
as operator abort and Cube enters MANUAL. This verifies takeover mode, not an
automatic return, surface or disarm guarantee.

An earlier continuous run also completed after extended GPS recovery; a prior
run aborted on synthetic DVL/VN freshness loss. Failed/interrupted runs and an
occupied-port attempt are retained in committed validation JSON, not discarded.
Tests arm only owned local native SITL, never hardware. Browser page was visually
inspected against these running simulations; automation starts missions via HTTP,
not by claiming a physical operator drove the UI.

## Limits before live testing

Synthetic sensors are deterministic, noiseless truth aids; these results establish
runtime/mission reachability, not physical navigation accuracy. Post-disarm drift
can change final position during GPS recovery (one earlier run drifted noticeably).
Real heading, mounting/frame alignment, actual pressure calibration, firmware
compatibility and map survey still need vehicle commissioning. No live mission
flags were enabled by this session. Readiness guards remain intact.
