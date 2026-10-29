# QGC Go-to starts alignment — October 25, 2026

User requested that QGroundControl **Go to location** start the alignment workflow, with no added navigation checks. Implemented and deployed an ingress adapter in the existing sole-owner MAVProxy wrapper. This does not start a second serial connection or replace the Cube firmware.

QGC `MAV_CMD_DO_REPOSITION` in COMMAND_INT/COMMAND_LONG and legacy NAV_WAYPOINT MISSION_ITEM/MISSION_ITEM_INT with `current=2` are consumed before they reach Cube. The selected latitude/longitude becomes a normal Jetson surface mission: initialize surface navigation when needed, run the existing 10 m current-heading GUIDED alignment, apply accepted Cube heading correction, then use the existing STABILIZE surface waypoint transit and arrival/stop/disarm behavior. QGC is the initiating interface; this is not a raw/native Cube Go-to after alignment. Coordinate altitude is not used to request a dive.

The adapter briefly selects MANUAL to release the previous target while the existing mission starts. Installed ArduSub 4.7.2 GUIDED initialization code was inspected: on entry it initializes its position target to a stopping point. Incoming Go-to is withheld from the Cube throughout routing; no original destination packet is independently replayed. Duplicate Go-to retries reuse the in-flight transaction. Legacy and modern acknowledgements and stage STATUSTEXT go back to QGC from MAVProxy's periodic task. HTTP work runs on a worker thread, leaving the serial forwarding loop responsive.

Stop/disarm/non-GUIDED mode changes and existing joystick-button takeover cancel the pending operation; an asynchronous start that completes after cancellation is aborted by run ID. QGC-triggered missions do not issue an arm command after disarm. Existing kill, source/watchdog and mission rules remain. No new sensor-health gates or covariance aborts were added. The old GUI distance range is not imposed on QGC destinations; API source tagging bypasses that GUI-specific range admission, while normal GUI behavior is unchanged.

Both current QGC protocol forms were checked against primary source:

- https://github.com/mavlink/qgroundcontrol/blob/master/src/FirmwarePlugin/APM/APMFirmwarePlugin.cc (`guidedModeGotoLocation`, modern reposition and legacy fallback).
- https://github.com/mavlink/qgroundcontrol/blob/master/src/MissionManager/MissionManager.cc (`writeArduPilotGuidedMissionItem`, current=2 Go-to and current=3 altitude-only).

This verifies supported formats in source, not the exact installed QGC version's on-wire selection. Pause/altitude-only commands with NaN coordinates and ordinary mission uploads are not classified as Go-to.

25 focused onboard tests passed: protocol formats, duplicate handling, acknowledgements, source IDs, cancellation including an in-flight-start race, no QGC auto-arm, no added GUI distance cap, source-hook ordering, heading rebase and DVL prediction. Waypoint JavaScript tests passed locally. The actual installed MAVProxy source was injected and compiled without executing a second MAVProxy instance. The first staged test run lacked the unchanged `mission_input_gate` dependency; copying that dependency into the test staging directory resolved it. No physical QGC Go-to/motion test was performed.

Deployment installed five changed/new files plus a targeted two-line GUI handler patch preserving other GUI source edits. Runtime Python source/installed copies verified. ROS and MAVProxy stopped only after fresh disarmed/no-autonomy observation. One surviving old wrapper process was stopped before starting the new sole owner. Both services are active, exactly one serial owner is verified, Cube telemetry and bridge status are fresh, and profile has no mismatches. Final observed disarmed MANUAL, STARTUP inhibit, ESC off; bridge disabled pending surface startup. Normal deliberate control recovery/arming is required before QGC Go-to can move the vehicle.

Backup: `/home/graey/deployment-backups/qgc-goto-20261024-030230` (vehicle clock); installed-tree backup `/root/robotx_ws/logs/installed-before-qgc-goto-20261025.tgz`. Stage `/home/graey/qgc-goto-alignment-20261025`. No vehicle parameters, firmware, Git branch, commit or push changed. No motion command was launched as a deployment test.
