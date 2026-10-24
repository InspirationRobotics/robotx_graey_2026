# Surface mission startup investigation — 2026-10-22

## Observed vehicle evidence

Read tether logs for run `75ba17d8bf4948848a9847c7f41694f5`. It entered
GUIDED and CHECK_SURFACE, then aborted with `Surface GPS unavailable for five
seconds`. It never reached ALIGN_MOVE or TRANSIT. GPS reported accuracy
degraded from about 1.192 m to 11.282 m; rejecting that fix was intentional.

Pressure depth rose from zero to a maximum sampled 0.051 m, ending at 0.038 m.
Logged vertical setpoints were zero or negative (upward in NED), never a dive
request. Cube local Z remained near zero while pressure indicated settling.
Zero velocity is closed-loop control, not neutral thrust. These logs do not
prove whether buoyancy, thrust/controller response, or estimator vertical
behavior caused the physical settling. A physical retest remains necessary.

## Changes

- Require the existing five-second accepted surface measurement before entering
  GUIDED. A single good GPS sample no longer starts closed-loop control.
- Keep updating this measurement through mode/arming transitions instead of
  throwing away its accepted history at CHECK_SURFACE. Fresh checks and the
  position hold still precede alignment travel.
- Send a zero-horizontal surface command on first confirmed GUIDED, and a
  start-position hold immediately when armed state is confirmed.
- Apply the same bounded pressure-based upward correction during alignment and
  braking as during hold/transit. Previously those paths hard-coded zero Z.
  Invalid telemetry still allows an abort stop without using invalid depth.
- Preserve GPS admission, depth limits, kill, takeover, and bounded aborts.
  No GPS accuracy relaxation or speculative gain tuning.

## Validation

26 focused tests passed, including closed-loop geographic missions at three
frame offsets and a regression proving an unqualified preflight cannot enter
GUIDED. Initial test invocation failed due to Python test-package resolution;
rerunning with the test directory on PYTHONPATH passed. Initial native runner
invocation lacked repository PYTHONPATH and failed before starting simulation.
Native retry and deployment outcome recorded below when verified.

Full regression: 294 tests, two skipped, no failures. Native run
`waypoint-7579920726` passed startup/alignment (-39.946 degrees recovered for
injected +40 degrees), then aborted on DVL recovery. Native clean retry
`waypoint-df067a7e40` reproduced settling: 0.09 m pressure depth during ALIGN_MOVE,
triggering the unchanged 0.08 m guard. Neither native run is a successful mission.
Deployment withheld: the startup changes are not a demonstrated descent fix.

## Firmware finding

Read-only AUTOPILOT_VERSION from the live Cube reports flight_sw_version
67438591 (4.5.7), flight_custom_version ASCII `30257f01`, matching released
revision `30257f01185471ab4c1ac544e47d1b4437e44c98` inspected here.

At that revision, `ArduSub/mode_manual.cpp` writes throttle_out=0 when disarmed;
armed MANUAL writes motors directly without updating that attitude-controller
value. `ArduSub/mode_guided.cpp` initializes Z control from
`AC_PosControl::init_z_controller()`, whose acceleration integrator uses the
previous attitude throttle. GUIDED's disarmed path also writes zero. For Sub,
neutral is 0.5, not 0. This is a credible descent mechanism, not proof of the
specific live event without its controller log.

Upstream issue: https://github.com/ArduPilot/ardupilot/issues/32056
Upstream neutral-throttle fix: https://github.com/ArduPilot/ardupilot/pull/32057
The upstream report concerns ALT_HOLD; the inspected GUIDED code shares the
initialization path. Do not infer a verified live GUIDED fix from that report.

No Cube firmware, parameters, mode, arm state, or installed navigation was changed
during this investigation. User confirmed safe state and fresh controller data
independently showed killed/disarmed/ESC-off. A targeted firmware fix needs local
validation and explicit approval before flashing. Do not compensate by blindly
increasing upward commands or relaxing the depth abort.
