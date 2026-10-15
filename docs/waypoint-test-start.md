# Fast waypoint-test preparation

Use the current checkout inside the Jetson ROS container. This passive check
collects 12 seconds of bridge data and current GUI/controller status without
opening another MAVLink endpoint or changing parameters:

```sh
docker exec graey bash -lc 'source /opt/ros/humble/setup.bash; source /root/robotx_ws/install/setup.bash; cd /root/robotx_ws/src/robotx_graey_2026; python3 tools/waypoint_preflight.py'
```

Save its JSON alongside the Cube log. Missing output/active flags are expected
before commissioning; do not bypass them to make the report green. Surface
pressure median is an observation, not permission to calibrate while submerged.

Order: verify startup/one bridge; disarmed kill/ESC off; fresh surface baseline;
valid DVL/VN/GPS; physical zero-offset heading; fresh Cube profile readback;
enable odometry and verify healthy stable Cube position/depth/heading; disarmed
submerged/surfaced continuity check; enable commissioned supervisor/mission;
validate map target and abort; short dive/transit/surface mission with logs.

Native ROS/GUI nominal and Abort moving missions passed October 13; see continuous-only-2026-10-13.md. Still pending: physical heading and map validation, live Cube odometry commissioning and submerged continuity checks. Temporary runtime launch overrides expire at reboot;
Cube external-only parameters persist, so never assume reboot restores the old
working navigation path. No output must be mistaken for a ready Cube estimate.



October14 update: use vn-pretest-workflow-2026-10-14.md for the current relative-VN north-alignment and checked activation workflow. Graey remains undeployed/offline in this session.
