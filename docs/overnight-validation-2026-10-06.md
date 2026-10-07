# Overnight repository validation — October 6–7, 2026

## Outcome

Repository hardening and architecture/recovery documentation are complete for
review. Full ArduSub mission validation is **not passed**. This branch is not
commissioned for vehicle deployment or autonomous pool navigation.

Work stayed on tether_toggle_beichen. Main, live Jetson services, Cube parameters,
and physical vehicle state were not changed in this session.

## Implemented

- Exclusive process ownership for DVL, VN-100, bridge and supervisor before node
  construction; real competing Linux subprocesses cannot acquire the same owner.
- GUI navigation lifecycle controls disabled at both frontend and API; executable
  counts distinguish wrappers from actual duplicate nodes.
- Optional prequal core startup defaults off; stateful bridge no longer respawns
  automatically inside core launch.
- Bridge identity overlap/restart latches a supervisor fault.
- Missing/nonfinite depth remains Unknown and cannot grant readiness.
- Distinct LED/supervisor MAVLink component IDs.
- VN reconnect closes the failed serial handle and discards partial data.
- DVL measurement validation rejects malformed data before partial publication.
- Installed map vendor assets and parameter profiles included in packaging.
- Corrected four invalid Windows-1252 punctuation bytes embedded in otherwise
  UTF-8 navigation/planner HTML; all inline scripts pass syntax checks.
- SITL ROS scope/domain separation, loopback port preflight, fresh EEPROM,
  archived prior result, stale synthetic-input rejection, waypoint resend
  throttling, fixed leg heading, and explicit failed-result recording.
- SITL motion stages abort on lost health/stale supervisor/wrong run; this
  simulated disarm policy is not a change to the live mission abort policy.

## Tests and observed runtime evidence

| Check | Result |
|---|---|
| Python regression suite under Linux/WSL | 79 tests passed, no skips |
| Same suite under Windows | 79 ran, 3 Linux-only lock tests skipped |
| Linux owner contention | 20 competing starts rejected while owner held lock |
| Owner crash | SIGKILL released lock without deleting its inode |
| Real second ROS bridge during SITL | Rejected before node construction with OwnershipError |
| Running GUI start-navigation API | HTTP 409, managed by core launch |
| Colcon build | Passed on all three bounded runs |
| HTML inline JavaScript syntax | All 5 script blocks passed Node --check |
| Git whitespace/diff checks | Passed |
| Full ArduSub surfaced/dive/transit/surface scenario | Not passed |

Existing deterministic tests exercise surface bobbing, stale sensors/intent,
GPS loss/rejection/recovery, missing depth, configuration gates, ACK-only and
source-only failures, alignment timeouts, position discontinuity, mission takeover
latching, and 20,000 seeded state-machine health/observation steps. These are
software logic tests, not hydrodynamic or sensor-accuracy validation.

Three bounded isolated ArduSub runs were attempted with QGC output disabled.
The first remained unqualified amid freshness interruptions. It also exposed a
runner diagnostic bug (obsolete multi-file tail syntax), which was corrected.
The second reached measured depth 0.550 m and verified GPS fix loss, but did not
complete the waypoint mission before the bounded run ended. Audit then corrected
the harness's continuous unchanged waypoint commands and missing health abort.

The third run switched Underwater → Surface GPS → Underwater, but the supervisor
latched “Source change not confirmed before timeout.” The test timed out at
WAIT_DIVE_REFERENCE and wrote passed=false. Run ID:
43aa21c0-11bb-495b-858a-05343ffdcf87. Result timestamp:
2026-10-07T06:13:41.429995+00:00 (October 6, 11:13 PM Pacific).
This is a failed-test timestamp, not the completion time of all repository work.

MAVProxy logs contain repeated “time moved backwards” warnings and link outages.
A separate 20 s host monotonic-clock probe sampled 398 intervals, min 0.050105 s,
max 0.050550 s, with no backwards or >0.5 s interval. Therefore a persistent
host-clock fault is not established. Simulator/MAVProxy timing, delivery latency,
and command/source feedback correlation still need diagnosis; do not blame the
hardware or relax safety timeouts based on these results.

All simulator processes were absent in the final process inspection. Runtime
logs/results are in ignored sitl/logs, not deployed to the vehicle. The old success
JSON from October 3 is historical and is not evidence that this revision passed.

## Open work and deployment conditions

Resolve the end-to-end source confirmation failure on a stable simulator runtime
before commissioning active vehicle switching. Acquire exact firmware and full
logs, verify actual frame/mounting/source configuration, and review the kill-switch
channel/policy issues listed in the software pipeline and recovery plan.

Locks only cover cooperating processes sharing their directory/scope. Legacy
processes and whole-service restarts need controlled disarmed migration. The
standalone PID-only stop script and untracked vehicle helpers remain audit items.

Testing agreement: every discovered bug must be fixed and regression-tested when
its behavior is established and within scope; otherwise record its evidence,
impact, uncertainty and required decision. Never conceal a failing simulation,
weaken a readiness gate to obtain a pass, or treat this agreement as permission
for unrequested live hardware changes.
