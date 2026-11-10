# Crusader alignment — November 9, 2026

**Current OCS:** [Shared team OCS](shared-team-ocs.md) supersedes the former patched OCS/skip-fleet instructions below. Native GRAEY reporting now works with unchanged team main; no OCS patch is required.

This is the current operator implementation; earlier custom laptop flows are superseded. No hardware deployment, commit or push occurred.

## Sources

Crusader main abf7d8214a7ef3194d4ec59ca5025795436adae8 and communications 86d5d8a913e751115b9a8860146120a45282959c; Ekko main fb913f40d2bbc09082a031771a4d182d02d671a5 and communications 2fc255e67c61691574c362eea8620b01ac997aa2; OCS main 9ccbb8c2538d1d7636b05852161ab7c4b5366ef7. Remote branch pointers were rechecked this session. Source archives/checkouts and prealignment backup are retained in the parent competition-comms folder.

## Copy/adapt audit

| Component | Classification | Basis and reason |
| --- | --- | --- |
| Laptop server, page, map/editor, wire, XML, parameter client, recorder, system info, log buffer | COPIED | Byte-identical communications-branch modules; SHA256 manifest accompanies validation. |
| Vehicle Connection and chunk writer | ADAPTED | Unchanged Crusader transport block with GRAEY package imports; separate bounded control/bulk lanes preserved. |
| Laptop launcher | ADAPTED | Crusader BOAT_GUI.cmd with GRAEY namespace/default host and explicitly approved8152. Same .venv fallback/curl/open/CLI flow. |
| OCS | ADAPTED | Current OCS main plus documented retained UUV run-tag capability patch. Root wrapper skips deferred fleet radio. |
| Common GUI operations/state | ADAPTED | FleetAdapter maps existing GRAEY navigation/controller/comms data and checked node APIs into Crusader contracts; no Rover BT or payload controls. |
| Missions and map | ADAPTED | Crusader course/mission workflow mapped to requested Waypoints, plotted submerged Pipeline Survey and 5–2–5; pose from Cube estimate. Shared OCS Declare/End and verified competition-only gate retained. |
| Sensors | ADAPTED | Existing DVL A50, VN100, Bar30, Cube and camera/sonar owners replace Rover sensors. Physical kill remains unsensed/independent. |
| Recording | ADAPTED | Copied Recorder/BagRecorder with native camera/sonar ownership and session paths; persistent mount and disk guard retained. |
| Parameters | ADAPTED | Copied ROS ParamBridge and page; GRAEY live callbacks validate state and change actual scale/density/watchdog objects. Wiring/listener/identity lifecycle fields require restart. |
| ROS/FCU/navigation/control | ADAPTED | Existing submarine interfaces and previously authorized gateway migration; MAVProxy remains sole serial owner. No Rover steering/arming/ESC recovery assumptions imported. |
| Theme | ADAPTED | Copied palette/initializer plus same-origin iframe theme synchronization and GRAEY mission/sensor layout. |

Private older laptop classes remain for compatibility tests; canonical laptop_main delegates to the copied app. Retired launcher helpers are archived, not operator entry points. Generic Crusader tabs use the copied page; adapters are separately identifiable files. This audit does not claim byte-identical ROS architecture or that arbitrary legacy mission tuning has runtime support.

## Runtime settings verified

DVL scale changes actual published velocity while safely disarmed with stopped missions. Water density changes require paused odometry and fresh disarmed state; old pressure is invalidated and Prepare is still required. Tether interval/timeout update the actual watchdog while disarmed, stopped and ESC-inhibited. None release a fault, enable odometry, arm or restore ESC power. ROS end-to-end tests exercise copied laptop RPC through TCP, private backend, copied ParamBridge and real ROS services. Listener, serial endpoint, frame/identity and wiring lifecycle changes remain restart operations.

## Laptop use

Double-click tools/scripts/GRAEY_GUI.cmd in this repository, or GRAEY_GUI.cmd in its parent workspace. To override the same-router address: `tools\scripts\GRAEY_GUI.cmd --graey GRAEY_WIFI_IP`. Open localhost8152. Run the parent OCS.cmd for localhost8099; RFD stays deferred. Use GRAEY_OCS_PORT if OCS web uses another port. Both programs use their repository .venv, as Crusader does. Close their terminals to stop.

## Hardware commissioning still required

Deploy only when authorized and GRAEY is available. Confirm WiFi address/firewall TCP14559, persistent shared recordings volume, same GRAEY_RECORD_DIR mapping for camera/sonar/ROS, versioned recorder capability and actual sources. Local tests use synthetic telemetry/devices and do not verify hardware wiring or present vehicle parameters.

## Course map follow-up

Map correction (user, November 9): inspected Crusader main and communications map.js; both have the same SHA256 a1d77a26507a599a3cbabdb12322dd81494ad16fc46169c6a27bf492231df1af. The map is an offline canvas metre-grid anchored to received coordinates, not a satellite/tile map or fixed city. Unified Missions now loads that exact renderer from tools/crusader_map.js. GRAEY adaptation supplies actual OCS polygon, fresh Cube pose/heading, stale state and mission draft; common pan/zoom/trail/arrow/waypoint-click behavior remains. Legacy /waypoint, /pool-map and /planner resolve to unified Missions, preventing old San Diego pool imagery from appearing in operator navigation. Historical imagery/source files remain preserved. No guessed Singapore vehicle position/course is substituted: absent course and pose remain unavailable. Changes are local, not pushed/deployed.

Use Plot points to enable/disable waypoint clicks (Esc also ends plotting), Fit course / route, Follow GRAEY, wheel zoom, drag pan and double-click follow, using the copied renderer. Pipeline endpoint labels remain in the route table; numbered draft markers follow Crusader.
