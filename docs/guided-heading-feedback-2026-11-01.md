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
