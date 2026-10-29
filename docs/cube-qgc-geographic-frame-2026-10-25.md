# Shared Cube, QGC and GUI geographic coordinates

Deployed 2026-10-25 at the user's request after they observed the GUI was accurate while QGC was offset.

## Cause and correction

The waypoint GUI previously transformed Cube LOCAL_POSITION_NED through a private Jetson GeographicFrame. QGC used Cube GLOBAL_POSITION_INT. Live inspection found Cube origin at 32.9235833, -117.0385061 while the private Jetson map origin was 32.9235839, -117.0385754, separated by about 6.5 m. Jetson sent private local coordinates directly as external odometry, so Cube interpreted them relative to its different origin. The GUI transform concealed that disagreement. Heading already came from Cube ATTITUDE plus the map rotation (zero in the manual-only path).

ContinuousEstimator now converts its private position through its geographic map and back to north/east metres relative to Cube GPS_GLOBAL_ORIGIN before bridge ODOMETRY transmission. Pressure down is unchanged. This is a position registration fix in the navigation input, not a QGC-only map-marker override, GPS spoof or origin reset. Cube's geographic origin is read, not changed. No QGC application modification, parameter/source-selection change, firmware modification or second serial owner was introduced.

Once registered, bridge status identifies `coordinate_frame: cube_ned`, exports the same Cube-relative position and an identity map at Cube origin for Jetson waypoint target conversion. Private estimator coordinates/map remain available as `estimator_position_ned` and `estimator_map_transform`. Manual heading-reset anchors are converted to the same Cube frame. Entering the registered frame increments the existing odometry reset counter once; repeated status updates do not. Existing manual heading correction and independent GPS/DVL/VN EKF updates remain.

The GUI then displays Cube GLOBAL_POSITION_INT and ATTITUDE directly, the same native telemetry used by QGC. Its fixed-local transform remains a compatibility path for an unregistered/older bridge; it is not applied again to registered Cube global coordinates. QGC Go-to still passes natively to Cube, whose geographic conversion now uses correctly registered external position. Automatic north alignment remains disabled; no new readiness/abort check was added.

ArduPilot documents the distinction between home and EKF origin, and that an established origin cannot be moved: [Setting Home and/or EKF origin](https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html). Local inspection of pinned ArduSub 4.7.2 GCS_Common.cpp confirmed GLOBAL_POSITION_INT uses AHRS location and NED velocity. The fix therefore registers external input to the existing origin rather than trying to move it.

## Validation and deployment

60 affected local Python tests and waypoint JavaScript tests passed. Onboard: 3 geographic registration, 17 GUI navigation view, 23 bridge transport, 5 heading correction, 2 manual-only navigation, 7 navigation-new, 2 fusion-policy, 1 native QGC forwarding and 10 calibration job tests passed: **70 total**. Coverage includes different origins, geographic-position preservation, late-origin registration without changing private EKF state, translated manual reset anchor, target inverse conversion, native GUI coordinates and one reset-counter update.

Archive: `output/cube-qgc-geographic-20261025.zip`. Source/installed runtime equality and Python compilation verified. Backup: `/home/graey/deployment-backups/cube-qgc-geographic-20261024-033413` (vehicle clock). Installed-tree backup: `/root/robotx_ws/logs/installed-before-cube-qgc-geographic-20261025.tgz`. Only graey-ros restarted; MAVProxy stayed running. Initial inspection was armed; activation waited until fresh disarmed MANUAL/no mission/no autonomy was observed.

Prior navigation/manual state was captured in `/home/graey/cube-qgc-geographic-before.json`. Following restart, the prior VN yaw offset was restored through the existing manual heading service using the current VN-derived yaw, then the previously enabled surface navigation path was restored through Prepare. Restored offset was -112.015755 degrees versus -112.021386 previously (about 0.006 degrees difference from asynchronous samples). No arming, motion, source selection, ESC recovery or parameter command was sent.

Live postdeployment native GLOBAL_POSITION_INT and GUI both reported **32.9236029, -117.0385565**, with zero coordinate difference at MAVLink integer resolution. Cube local minus outgoing registered odometry was approximately +0.018 m north, -0.041 m east. Profile verified, transmitting true, cube_origin_registered true, reset_counter 2 (manual reference plus registration). Controller was disarmed MANUAL/STARTUP/ESC off/no autonomy. Evidence saved in container `/tmp/cube-qgc-geographic-postdeploy.json` and copied to host `/home/graey/cube-qgc-geographic-postdeploy.json`.

This verifies matching telemetry sources and coordinates at deployment time. QGC's actual screen and physical native waypoint performance were not inspected/tested. GPS accuracy and physical heading accuracy remain independent of source agreement.
