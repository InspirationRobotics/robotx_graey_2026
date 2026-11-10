# GUIDED waypoint heading feedback — local only

User requested per-loop heading correction to favour forward travel.
GuidedSurfaceRun computes atan2(delta East, delta North) in the same Cube-local
NED frame as the position command and subtracts Cube yaw using shortest-angle
wrapping. Each scheduled transit command carries a proportional yaw rate:
sign(error) * min(20 deg/s, 0.5 * max(abs(error)-3 deg, 0)).

Above 45 degrees error the XY destination is the current position while turning;
travel resumes below 25 degrees. This hysteresis avoids repeated toggling.
The real geographic waypoint remains unchanged. This favours forward travel but
does not prohibit Cube from using lateral thrust to oppose current or brake.

Transport uses SET_POSITION_TARGET_LOCAL_NED mask1528 (position and yaw rate),
with radians/s on the wire. Three-coordinate hold commands explicitly send zero
yaw rate so arrival, stopping and telemetry waits cancel the previous rotation.
Existing surface Down target, 1.5m arrival radius, dwell, takeover and disarm
sequence are unchanged. No heading calibration or EKF state is altered.

Compatibility checked against upstream Sub-4.7.2 GCS_MAVLink_Sub.cpp local-NED
handler, which passes yaw-rate use/value to guided_set_destination. Hardware
turn response and surface-depth behaviour remain unverified.

58 focused mission/bridge/estimator tests passed, including bearing direction,
wraparound, deadband, proportional/capped correction, hysteresis, hold rate
cancellation and a simple kinematic heading exercise. Initial run had two old
position-only assertions fail; updated expectations for intentional yaw-rate
behaviour and reran successfully. No native SITL or vehicle test performed.
No deployment, commit or push; preserve unrelated workspace changes.

## Subsequent deployment and native SITL validation

The preceding status describes the original local implementation. Commit 7000358
was subsequently pushed and deployed; see GRAEY_CONTEXT.md for that deployment.
The following tests were local only, with no vehicle commands or new deployment.

On November 1, 58 focused mission/bridge/estimator unit tests passed.
Actual ArduSub 4.7.2 SITL, ROS bridge, production waypoint node and GUI API were
then exercised using `sitl/validate_waypoint.py`. QGC forwarding was disabled;
MAVProxy endpoints were loopback only and ROS used isolated domain 73.

| Target local N/E | Result | Final estimated target error | Transit yaw first/last |
| --- | --- | --- | --- |
| 0 / 6 m | COMPLETE, stopped and disarmed | 1.404 m | 0.13 / 85.19 degrees |
| -4 / 0 m | COMPLETE, stopped and disarmed | 1.425 m | 0.15 / 175.02 degrees |

Both arrived inside the configured 1.5 m radius. Maximum sampled pressure depth
was 0.020 m in both runs. The east run reached the commanded 20 deg/s rate cap.
The approximately 5-degree final heading error is observed simulator response,
not a claim of exact heading convergence or physical accuracy.

Evidence: `sitl/logs/waypoint-abd66f8317/validation.json` and
`sitl/logs/waypoint-cfcb0c47a2/validation.json`, with corresponding mission traces,
ROS logs and MAVLink recordings. No native controller bug was demonstrated.
Unit coverage includes negative turns, wraparound, deadband, proportional gain,
rate cap, turn/translation hysteresis, and cancellation of yaw rate on hold.
Native runs covered east and behind-target turns, not current/wave disturbances
or sensor fault injection. Existing unrelated local bridge edits were present;
these were working-tree integration tests, not an exact deployed-image replay.

Setup failures retained: initial import invocation failed before recording;
waypoint-1f28d7b773 and waypoint-7413a369f1 failed before launch due to binary
path/space quoting. waypoint-a3155e0dd4 failed because the temporary binary was
absent on the next WSL invocation. Copying the binary to a space-free /tmp path
in each invocation resolved setup. None are counted as successful missions.
Both successful harness runs cleaned up; no SITL/MAVProxy/launch processes
remained at the final process check. No production code was changed by testing.

## Distance-based horizontal speed — subsequent local change

Requested proportional travel speed now accompanies each transit target:
`0.15 + 0.25 * clamp((distance - 1.5) / 3.5, 0, 1)` m/s.
Thus 5 m and farther requests 0.4 m/s, 3.25 m requests 0.275 m/s,
and approach at 1.5 m requests 0.15 m/s. The near speed is an explicit
chosen default, not a freshly measured vehicle parameter. Turning-in-place
and hold commands use the 0.15 m/s limit; existing position hold stops motion.
DO_CHANGE_SPEED (178), ground-speed type 1, sets a runtime WPNav horizontal
limit; no persistent parameter or vertical-control changes. The snapshot exposes
`transit_speed_limit_mps`. Heading gain/thresholds and arrival are unchanged.

59 focused tests passed. Native ArduSub 4.7.2/GUI/ROS run
`sitl/logs/waypoint-9c40404eeb` completed a 6 m east target, stopped/disarmed,
final estimated distance 1.459 m. MAVProxy recorded DO_CHANGE_SPEED ACCEPTED.
Measured sampled horizontal DVL speed peaked at 0.121 m/s despite 0.4 m/s
requested limit: this verifies command acceptance and arrival, not achieving
0.4 m/s physical speed or proportional measured speed. Repeated position-target
updates can reinitialize WPNav legs (see pinned AC_WPNav destination handler);
that is a candidate explanation, not yet isolated by this test. No speed tuning
or vehicle deployment was performed. These speed changes remain local.
