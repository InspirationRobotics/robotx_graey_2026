# Orient once, then original GUIDED travel

Supersedes the November 1 continuous steering, local speed taper and local
automatic north operator proposals. User approved GUIDED position hold and
orientation to within 3 degrees before position-only transit.

`guided_orientation.py` computes a fixed initial bearing from Cube-local position
to the selected waypoint. `guided_surface_mission.py` enters ORIENT after arming.
The transport holds captured XYZ and sends absolute yaw (mask 2552). Once the
latest estimated yaw is within 3 degrees, it enters TRANSIT and sends the original
position-only targets (mask 3576). There is no recurring Jetson heading controller,
45/25-degree hysteresis or speed command. Arrival remains 1.5 m. Existing
takeover, abort, telemetry and estimator behavior are retained; no new timeout,
dwell or sensor qualification was introduced. Orientation uses the existing
heading reference and does not learn true north.

The uncommitted automatic north operator and proportional speed were removed.
Manual north adjustment remains. Historical November 1 notes describe previous
work, not the current design.

Validation: 71 focused tests passed. Native ArduSub 4.7.2 SITL through the GUI API
passed STARTING -> WAIT_SURFACE -> GUIDED -> ARM -> ORIENT -> TRANSIT -> STOP ->
DISARM -> COMPLETE for a 6 m east waypoint. QGC forwarding disabled. This tests
the orientation/handoff, not recovery from a wrong heading reference. No vehicle
deployment or live parameter changes were performed.
