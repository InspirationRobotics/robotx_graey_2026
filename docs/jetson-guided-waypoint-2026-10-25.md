# Jetson waypoint mission uses Cube GUIDED control

Deployed 2026-10-25 after the user reported that the restored Jetson mission appeared to hold position and requested the same waypoint/EKF concept as working QGC navigation. QGC itself remains untouched.

## Evidence and change

Read-only live inspection found run `2a55109347a64f84a29dbf9aaf7e1320` had a real geographic target and local target approximately [-1.122, -6.818] m, with about 6.016 m remaining at its last moving sample. It used the Jetson STABILIZE SurfaceLeg/StabilizeSurfaceControl path, with last pilot command [81,150,500,100], roughly 0.585 m estimated along-track progress and 77.9 degrees heading error. It subsequently reported pilot/failsafe takeover with Cube MANUAL/disarmed. This does not prove zero motion or an internally issued position hold; it establishes that this path differed from the working native Cube waypoint controller.

Jetson now owns the mission lifecycle and target, converts geographic coordinates into registered Cube local NED, selects GUIDED, and sends the actual waypoint as SET_POSITION_TARGET_LOCAL_NED. The mask is position-only 3576: velocity, acceleration, yaw and yaw rate are ignored so Cube controls travel. The current local down is captured and retained. No custom STABILIZE horizontal joystick steering or lookahead controller is used in this path. Neutral pilot packets maintain the existing mission-input arbitration while Cube owns position control.

Pinned ArduSub 4.7.2 `GCS_MAVLink_Sub.cpp` lines 680–681 route position-only local targets to `sub.mode_guided.guided_set_destination`, the same waypoint-controller entry used by DO_REPOSITION. Jetson is still the mission process; the GUI again uses preview/start/abort, not its removed direct native Go-to adapter. No QGC command interception was restored.

Startup uses an initialized usable EKF and an available geographic transform, rather than requiring independent fresh GPS/DVL qualification or a north maneuver. The GUI startup wait also uses EKF readiness instead of a GPS/DVL/VN vote. The existing manual heading reference is retained; automatic heading alignment remains disabled. No new abort/readiness checker was added. Existing operator/kill/connection authority and estimator-loss handling remain. GUI preview can start from GUIDED as well as MANUAL/STABILIZE, consistent with the chosen mission mode.

At the existing estimated arrival radius, Jetson requests a GUIDED brake/hold, then retains its existing stop/disarm completion sequence. Explicit abort still requests MANUAL through the existing path. There is no alignment travel, turn-only phase, new travel cap or repeated short waypoint/check maneuver. The GUI reports Cube GUIDED waypoint control rather than the inactive custom SurfaceLeg's waiting message.

## Verification and deployment

69 affected local Python tests plus GUI JavaScript passed. Onboard: 3 new Jetson GUIDED tests, 2 manual-only navigation, 7 navigation-new, 5 heading correction, 1 waypoint transport, 7 GUI operation helpers, 1 native QGC forwarding, 3 geographic frame, 17 navigation view, 23 bridge transport and 10 calibration jobs passed: **79 total**. Tests cover target conversion distinct from current position, startup with GPS/DVL absent but an available EKF, repeated real target commands, current-depth retention, no alignment stage, arrival hold/disarm, and exact position-only packet arguments. An in-memory pymavlink serialization check confirmed the actual local-NED target/mask without a socket or vehicle command.

Package: `output/jetson-guided-waypoint-20261025.zip`. Four runtime files changed: guided_surface_mission.py, waypoint_mission.py, waypoint_api.py and waypoint.html. Source/installed equality and Python compilation verified. Backup: `/home/graey/deployment-backups/jetson-guided-waypoint-20261024-041634` (vehicle clock), including before/after snapshots and unchanged hashes. An additional GUI-only wording fix was incorporated into the final archive/manifest and installed asset, with GUI JavaScript rechecked.

Deployment observed fresh disarmed MANUAL/no mission/no autonomy before replacing mission files. Only the GUI process was restarted to reload the preview helper; the stateful bridge, ROS core and MAVProxy were not restarted. Before/after SHA-256 comparisons confirmed QGC/MAVProxy scripts and continuous_estimator/fused_nav_bridge unchanged. Bridge instance `0aa75d8c-9d64-46b7-8e05-94b71adf6f10`, manual offset -92.9341797927508, cube_ned registration, enabled/transmitting state and verified profile were retained. Final controller remained disarmed MANUAL/no mission/no autonomy. No physical mission, arming or movement was initiated by deployment verification; actual travel under the new path still needs observation.

This supersedes prior reports describing Jetson surface missions as STABILIZE control, while preserving the working QGC and shared geographic-frame changes.
