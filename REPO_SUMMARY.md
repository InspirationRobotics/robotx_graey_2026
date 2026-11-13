# GRAEY repo summary (read this first)

Condensed from the code and docs as of `ruth_new_branch` (Nov 13 2026). `docs/` and
`GRAEY_CONTEXT.md` are dated history: use them for background, and trust the code when they disagree.

## Vehicle
GRAEY is Team Inspiration's UUV for RobotX 2026 (Singapore, Nov 8-14). Its task is Infrastructure
Survey & Repair (Task 2 pipeline). It works with Crusader (the boat) through the team OCS, which talks to RoboCommand.
- **Computer:** Jetson Orin Nano, JetPack 6.2. ROS 2 Humble runs in the docker container `graey`. The workspace is
  `~/robotx_ws`, and the repo is bind-mounted at `src/robotx_graey_2026`.
- **Autopilot:** Cube Orange, ArduSub **4.7.2**, VECTORED_6DOF, 8 thrusters, MAVLink system **3**.
- **Sensors:**
  - Water Linked DVL A50: TCP `192.168.2.10:16171`.
  - VN-100: serial.
  - Bar30 pressure: the Cube's `SCALED_PRESSURE3`.
  - Two OAK-D cameras, FRONT and DOWN, picked by MxId in `api/vision/camera.py`.
  - Ping360 sonar.
  - M9N GPS on the Cube. It's telemetry only; the EKF doesn't use it.
- **Other hardware:**
  - LED panel: an Arduino/Pico running `firmware/LED/LED.ino`.
  - RadioMaster RC.
  - RFD900 radios.
- **Kill:** Cube SERVO9 → RC switch → relay → contactor → ESC power. A physical magnetic kill sits in series.
- **Networks:** tether Jetson `192.168.2.2` / laptop `192.168.2.1`; WiFi on the team router.

## What runs where
Three host systemd services (`deploy/systemd/`):
1. **`graey-mavproxy`** → `scripts/start_mavproxy.sh` → `scripts/mavproxy_mission_gate.py`.
   - MAVProxy is the **only** program that opens the Cube's USB serial port.
   - It outputs a UDP broadcast on 14570 (for QGC) and `udpin 127.0.0.1:14551`, which belongs to `telemetry_gateway`.
   - `scripts/mission_input_gate.py`: a pilot's mode change, disarm or joystick input takes control back from the mission component.
2. **`graey-ros`** → `ros2 launch robotx_graey_2026 core.launch.py` inside the container. Every node respawns except `fused_nav_bridge`.
3. **`graey-sonar-launcher`** → `tools/gui_sonar_launcher.py` on the host, port 8096.
   - `tool=map` starts `tools/sonar_map.py`, which serves its page on 8095 and gets pose from `tools/pose_relay.py`.
   - `/camera/start` runs `tools/cam_pair.py` in the container, streaming on 8080 (DOWN on the left, FRONT on the right).

## The FCU gateway (`api/pixhawk/`)
`telemetry_gateway` is the only ROS node connected to MAVProxy.
- **Receiving:** it publishes each Cube message type on `/graey/fcu/mav/<kind>`; the list is `FCU_RX_KINDS` in `ros_link.py`.
  Consumers use `Link(url, component, node=node, kinds=(...))`. A ROS `Link` without `kinds` raises an error.
- **Sending:** nodes put messages on `/graey/fcu/tx`. `gateway_core.GatewayPolicy` decides what reaches the Cube, by component:

| Component | Owner | Allowed to send |
|---|---|---|
| 194 | `kill_switch` | ESC relay (`DO_SET_SERVO`), arm/disarm, MANUAL |
| 197 | `fused_nav_bridge` | `ODOMETRY` (the position for the EKF) |
| 198 | missions | motion and mode; motion only while control is permitted and the sub is armed |
| 195 / 193 / 191 | `gui_node` / `pixhawk_led_node` / `competition_link_node` | read-type messages only |
| 43/201 | `radio_mission_node` | RFD `TUNNEL` to OCS 250/190 and laptop 254/191 |

"Control permitted" comes from `/graey/control_watchdog/status`, published by `kill_switch`.

## Nodes in `launch/core.launch.py`
- **`kill_switch`:** reads the RadioMaster.
  - CH10/SA = kill
  - CH11/SB = arm/disarm
  - CH7/SC = MANUAL/STABILIZE
  - CH12 = transmitter heartbeat

  It drives the ESC relay and runs an RF or tether watchdog. At startup, and after a link loss, ESC power is inhibited.
  To recover on RF: restore the heartbeat, set SA KILL and SB DISARM, then release SA.
  On the tether: use the GUI's recover button.
- **LEDs:** `pixhawk_led_node` (status: red disarmed / yellow manual / green auto) → `led_node` (serial to the LED board).
- **Navigation:** `dvl_node`, `vn100_node` → `fused_nav_bridge` → `ODOMETRY` at 20 Hz → the Cube's EKF3.
  `EK3_SRC1_POSXY/VELXY/POSZ/YAW = 6` (external nav) and `COMPASS_USE 0`, so position, depth and heading all come from the bridge.
  `navigation_supervisor` watches the bridge's health.
- **GUI:** `gui_node` serves 8090 and starts missions. `gcs_bridge` is a TCP server on 14559 for the laptop app.
- **Comms:**
  - `competition_link_node`: GRAEY's reports to OCS over WiFi (TCP 37564, discovery on 37565) → RoboCommand.
  - `radio_mission_node`: missions and status over the RFD (see below).

## Navigation gotchas
- **The bridge's output is OFF after every graey-ros restart.** Turn it on with GUI **Start Navigation** (`/graey/navigation/prepare`:
  sub floating at the surface, disarmed; this also sets the depth zero) or **Apply heading** (`align_heading`). Until then the
  EKF has no position, and POSHOLD, GUIDED and AUTO are refused with "requires position".
- The bridge also needs fresh read-backs (under 3 s old) of the Cube parameters in `api/navigation/odometry_profile.py`.
  `GRAEY_FIRMWARE_PROFILE=4.7` must be set.
- The VN heading correction (Apply heading) lasts for one bridge session only.

## Missions (Missions page = `tools/missions.html` + `missions.js`)
Documents are validated in `api/gui/mission_store.py`. The flow is: Load (staged, with a SHA-256 revision) → Prepare → Start / Stop.
The transport can be Auto (WiFi/tether first, then RFD), IP only or RFD only.

| Kind | Runs as | What it does |
|---|---|---|
| Waypoints | `waypoint_mission` (`competition_route`, `waypoint_logic`) | surfaced geographic legs |
| Pipeline Survey | `waypoint_mission` (`competition_route`) | surface to the start, 4 m pressure-depth traverse to the end, surface |
| 5–2–5 | `waypoint_mission` (`simple_sequence`) | forward 5 m, down 2 m, forward 5 m, surface, disarm |
| Dive Here, Pool Dive | `waypoint_mission` (`dive_spin`) | (Dive Here: forward 1 m first) dive by DVL altitude or typed depth, 8 headings, surface, disarm and kill |
| Surface Route, Dock → Pipeline → Dock | `waypoint_mission` (`radio_route`) | up to 15 surfaced legs; Dock adds dive/spin and the return |
| Survey run (Ruth) | `api/gui/survey_launch.py` → `tools/survey_run.py` → route + `pipeline_mission.py` | sonar + camera survey between two buoys |

Also started by the GUI: `north_maneuver` (Calibration page). Only one mission runs at a time.

**Competition runs:** arm → actual POSHOLD → report `STATE_AUTO` → wait for a matching verified `RUN_START` from OCS → GUIDED.

**Colour report:** when Dive Here completes, it publishes a two-colour `resource_request` (currently random RED/GREEN/BLUE) on
`/graey/comms/resource_request`. `competition_link_node` sends it to RoboCommand. Sending it to the boat is not wired yet:
`api/comms/sub_link_node.py` + `mesh_link.py` exist but aren't launched and use an old UDP port.

## RFD path
1. Laptop `GRAEY_GUI.cmd`
2. `fleet_link` (UDP 14572)
3. Shared team OCS
4. RFD radios
5. Cube TELEM
6. MAVProxy
7. Gateway
8. `radio_mission_node`

The mission document travels as zlib + CRC `TUNNEL` chunks and is stored under `/root/robotx_ws/missions/radio/<sha256>.json`.
It is then run through the same GUI mission API as WiFi missions.
- Details: `docs/rfd-mission-relay-2026-11-11.md`, `docs/unified-transport-missions-2026-11-12.md`.
- The physical radio link is not verified: the Cube's TELEM ports run at 115200 baud; RFD radios ship at 57600.

## GUI
- `gui_node` serves the pages in `tools/`, listed in its `FILES` table:
  - `/` = `gui.html` (every tab)
  - `/missions`
  - `/sensors`
  - `/record`
  - `/calibration`
  - `/navigation`
  - `/navigation-supervisor`
  - `/simple-sequence`
- The laptop app is `api/gui/laptop/graey_app.py` on localhost 8152, started by `tools/scripts/GRAEY_GUI.cmd`. It is Crusader-derived.
- Recording (rosbag + camera + sonar) goes to `~/robotx_ws/recordings`.
- Operator guide: `docs/unified-operator-guide.md`.

## Sonar and cameras (Task 2)
- `pipeline_mission.py` (4.4k lines) drives the survey:
  - the sonar map (`api/sonar/`: sweep, crumbs, outlines, detect);
  - DOWN-camera detectors (`api/vision/led_boxes.py`, `pipe_line.py`).
- Set `SPEED_OF_SOUND` in `api/sonar/settings.py` for each site: currently 1512 m/s for Marina Bay; ~1480 in cool fresh water.

## Repo map
- `robotx_graey_2026/api/`:
  - `pixhawk/`: gateway, `kill_switch`, `Link`, identities
  - `navigation/`: sensors, bridge and estimators, missions
  - `gui/`
  - `comms/`, `radio/`: OCS and RFD
  - `sonar/`, `vision/`
  - `led/`
- `tools/`: GUI pages, launcher, `cam_pair`, `sonar_map`, `survey_run`, `guided_goto`
- `launch/`: `core`, `comms`, `navigation_supervisor`, `sitl`
- `config/graey_params.yaml`: node settings and gateway stream rates
- `params/`: Cube parameter files; see `params/README.md`
- `scripts/`, `deploy/`, `docker/`: MAVProxy, systemd, container
- `sitl/`: ArduSub SITL checks
- `test/`: unit tests plus `ros_*_smoke.py`, which use real ROS
- `archive/`: retired code, not installed
- `docs/`: history

## Working on the code
- **Tests:** in a ROS Humble environment, run `PYTHONPATH=<repo>:$PYTHONPATH python3 -m pytest test`. At `d52e9ac` that gives
  502 passed, 12 skipped. Run `test/ros_*_smoke.py` separately. `ros_competition_smoke` also fails on the pre-cleanup code.
  The `*.cjs` page tests need Node.
- **Deploying** (Beichen's process): copy the files into `src` on the Jetson, then rebuild/copy `install` inside the container
  and restart graey-ros. Deploy only when the operator says so, with GRAEY killed and disarmed.
- **Never** add a second program on the Cube serial port or a direct UDP `Link` to MAVProxy; go through the gateway.
  Don't change Cube parameters without the operator.
- Some files (`gui_node.py`, `kill_switch.py`, several navigation files) use CRLF line endings. Keep them.

## Recent history
- **Jul–Aug:** Chris's base stack (MAVProxy, nodes, prequalification).
- **Oct 13:** continuous navigation; source switching and `nav_ekf_bridge` retired.
- **Oct 23:** ArduSub 4.7.2.
- **Nov 10:** Beichen's Crusader-style rewrite: gateway, laptop GUI, shared OCS.
- **Nov 12:**
  - the gateway switched to one topic per message type;
  - Chris added Dive Here, Pool Dive and the RFD mission relay.
- **Nov 13:** this cleanup removed prequal/demo missions, the pole tracker, `sim/`, bench tools and test-only modules.
  It also fixed the terminal survey run, which had crashed on the new gateway. Graey's own snapshot of that day is branch `graey-live-nov13`.
