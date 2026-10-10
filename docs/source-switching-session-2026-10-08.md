# Source switching session — October 8–9, 2026

Status: source-switching patch implemented and final local validation passed.
Intermittent runtime scheduling delay remains a documented limitation below.
User authorized source-switching code changes, regression
tests and local SITL. Branch tether_toggle_beichen, starting commit 98d167b.
No physical Graey access, deployment, live parameters or main changes.
Preserve unrelated untracked docs/nav_bridge_ownership.md.

The hourly follow-up `resume-graey-source-switching-work` was used for resumption
and was removed after this validation pass. The October 9 resumption followed
explicit user confirmation of Astra light. Changes remain uncommitted locally;
there was no push, main change, live parameter write or Jetson deployment.

## Implemented changes

- Source confirmation only accepts ACK and selected-source evidence received
  after the request and within its three-second deadline. Tests demonstrated
  that the old logic could accept late evidence before checking timeout.
- Preserve timely evidence across subsequent periodic source reports, while
  still requiring the current source to match and remain fresh. Fault injection
  exposed an intermediate diagnostic bug where a later report erased timely
  evidence; a regression reproduces it and the implementation now passes.
- Ignore unsolicited ACKs and ACKs addressed to another component; IN_PROGRESS
  is not success. Keep rejection, discontinuity and clock rollback faults latched.
- Check position continuity in both switch directions. Log SOURCE_REQUEST,
  SOURCE_ACK and frozen SOURCE_RESULT records, also exposed as source_transaction
  in existing supervisor status. Internal transaction IDs are diagnostics only:
  MAVLink ACK has no transaction ID and cannot distinguish every delayed ACK.
- SITL aborts now distinguish stale supervisor status, mismatched mission run,
  and actual supervisor faults. No timing or health gate was weakened.
- Add bounded localhost-only validation with dropped ACK/source feedback cases,
  separate evidence directories and process-group cleanup. Optional native WSL
  snapshots retain source/binary hashes and copy evidence back to the repository.
- TCP listener preflight permits TIME_WAIT reuse, still rejecting an occupied
  listener and naming its port. QGC forwarding is disabled in validation.

## Validation checkpoint

93 Linux unit tests passed. Several full ArduSub missions passed (surface GPS,
qualified reference, underwater source, 0.5 m dive, confirmed GPS loss, waypoint,
surface GPS recovery). Both fault injections pass strict checks: no navigation
readiness and simulator remains disarmed, with the correct missing-evidence reason.

Final-latch mounted-workspace evidence:
`sitl/logs/switch-drop-ack-c3a7c46a6c` and
`sitl/logs/switch-drop-source-c816afd24d` both pass.
`sitl/logs/switch-nominal-be4297db71` confirms all three source changes and reaches
the waypoint, but aborts on supervisor status age 1.170 seconds (limit 1 second).

Native snapshot `sitl/logs/graey-switch-i3gzc0gl/manifest.json` records four runs:
nominal pass, missing-ACK pass, missing-source pass, nominal failure at supervisor
status age 1.115 seconds. Moving to native storage did NOT eliminate the failure.
Do not select only successful runs or claim overall validation is complete.

The earlier October 6 source timeout has not been reproduced deterministically;
its original cause remains unproven. Current work adds diagnostics and fixes
demonstrated logic bugs; it is not numeric GPS/heading EKF tuning.

## October 9 continuation

The user confirmed the current model is Astra light and explicitly resumed work.
Recovered the three already-started runs in
`sitl/logs/graey-switch-67h9hzd1`; all three completed the full mission successfully.
Two runs logged supervisor callback gaps of 1.028 and 1.004 seconds, with MAVLink
drains of only 0.015 and 0.014 seconds. This identifies delayed callback execution
in those samples, but does not prove whether host scheduling, ROS scheduling or
another callback caused it. No shared MAVLink transport rewrite or relaxed timing
threshold is justified by this evidence. Earlier failing runs remain retained.

Added a regression proving that stale status which still says "Navigation inputs
healthy" aborts the simulated mission without issuing a new target. Scenario JSON
now preserves the actual failure reason, and source trace timestamps use the same
request timestamp as the state machine. The final software suite passes 94 tests.
Final full simulation batch: `sitl/logs/graey-switch-fyvz37yd`; all four passed:

| Scenario | Evidence subdirectory | Result |
| --- | --- | --- |
| Full mission | switch-nominal-041e7191f0 | Dive, GPS loss, underwater waypoint, surface GPS recovery |
| Missing ACK | switch-drop-ack-ab7af2ba08 | Specific ACK timeout; readiness false, disarmed |
| Missing source report | switch-drop-source-4757d2af87 | Specific source timeout; readiness false, disarmed |
| Full mission repeat | switch-nominal-0936845528 | Dive, GPS loss, underwater waypoint, surface GPS recovery |

The runner exited successfully. Shell syntax validation and Git whitespace checks
passed. Evidence includes all final result JSON files, simulator/ROS logs and the
snapshot manifest. Ignored simulation logs remain local rather than being added
to Git. Unrelated `docs/nav_bridge_ownership.md` was preserved unchanged.

## Reproduction and interpretation

Under Ubuntu/WSL with the existing ROS Humble, colcon, pymavlink and MAVProxy setup,
from the repository root:

```sh
python3 -m unittest discover -s test
bash -n sitl/run.sh
python3 sitl/validate_native.py
```

The last command copies a source snapshot and the configured SITL binary to native
Linux storage, runs the suite plus nominal/drop-ACK/drop-source/nominal scenarios,
and retains all logs and source hashes under `sitl/logs/graey-switch-*`.
Set SITL_BIN if the existing default executable path is unavailable. The tested
binary SHA256 is `f272e76ef10b8e9314f3efa4c8fa0ec30bcf2731280e85a73ef61edaeb7a971`.
Failure cases must withhold readiness and keep the simulator disarmed; a detected
fault is the expected passing result for those injected failures.

These changes affect Jetson supervisor confirmation and diagnostics, not Cube
firmware or numeric EKF fusion weights/source parameters. Physical GPS accuracy,
heading calibration, live switching and vehicle behavior remain unvalidated.
Before deployment, validate live source reporting, configured source parameters,
frame alignment and sensor health. The intermittent host/runtime status-age abort
is still an open limitation; passing repeats do not establish its resolution.

Execution: default shell helper currently fails setup refresh; escalated shell
works. On October 8 automatic approval review became unavailable due to usage
limits; work paused. No approval bypass was attempted. Required context and
OpenAI Docs skill were read.
