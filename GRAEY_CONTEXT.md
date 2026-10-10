# GRAEY project context

Collected 2026-09-25. This is an onboarding reference, not a live vehicle report or deployment approval.

## Provenance and review coverage

- Primary task: **Graey Background + kill GUI**, task ID `01a093cf-f501-7c61-9743-0bf5871c3349`, local host. Its paginated conversation was retrieved through its earliest turns. Some long messages/tool results were truncated; do not claim every byte was reviewed. Read that task again for detailed decisions when relevant.
- Follow-up task: GPS, task ID `01a0bca6-0cee-73e0-a9ac-13f1d5272af0`. User states M9N is connected to the sub's GPS2 connector.
- Direct local inspection included README, core launch, nav_ekf_bridge.py, pos_server.py, relevant Link helper code and selected saved parameter values in `robotx_graey_2026`.
- Attachment contents have NOT all been independently reviewed in the GPS task. The earlier task reported reading documentation/checklists/parameters and extracting electrical diagram labels, but explicitly did NOT visually trace all wiring. Inherit that limitation, not a claim of full attachment review.
- Local code, saved parameter snapshots, historical deployment and present vehicle state can differ. Resolve conflicts with the relevant source and fresh read-only inspection before implementation.

## Vehicle and mission

GRAEY is Team Inspiration's underwater vehicle for RobotX 2026, working alongside Crusader (surface boat) and BabyDragon (UAV). README identifies Infrastructure Survey & Repair as its competition task and an August 2026 prequalification submission.

- NVIDIA Jetson Orin Nano, documented JetPack 6.2.
- CubePilot Cube Orange running ArduSub, VECTORED_6DOF, eight thrusters. Saved files are named for 4.5.7; verify installed firmware rather than assuming that version.
- Water Linked DVL A50: bottom-relative velocity and altitude/bottom-lock status.
- VectorNav VN-100: attitude and heading.
- Bar30: pressure/depth; Cube internal IMU supports state estimation.
- Two Luxonis OAK-D cameras, forward and downward.
- Arduino-driven NeoPixel status strip.
- M9N on GPS2: user-confirmed connection, not yet verified working by this task.

## Repository and runtime structure

Repository: https://github.com/InspirationRobotics/robotx_graey_2026

Local checkout: `robotx_graey_2026/` under this project. Another folder, `mission_elements/`, is present; it was not fully reviewed during onboarding.

On Jetson, workspace is `~/robotx_ws`, repository under `src/robotx_graey_2026`. Source is bind-mounted into Docker container `graey`, containing ROS 2 Humble and CUDA/TensorRT. Builds run inside the container. Host systemd services are `graey-mavproxy.service` and `graey-ros.service`.

Important paths inside repository:
- `launch/core.launch.py`: kill switch, DVL, VN-100, navigation bridge, GUI and LED nodes, with node respawn.
- `launch/prequal.launch.py`: additional mission/perception support.
- `robotx_graey_2026/api/navigation/`: dvl_node, vn100_node, nav_ekf_bridge, frames, pos_server, mission_base, demo/prequalification missions.
- `robotx_graey_2026/api/pixhawk/mavlink.py`: shared Link helper for MAVLink.
- `robotx_graey_2026/api/pixhawk/kill_switch.py`: control/failsafe state and ESC permission commands.
- `robotx_graey_2026/api/gui/gui_node.py`, `tools/gui.html`: GUI backend and page. Tether draft injected its extra control panel from Python, so only two Python files changed in that work.
- `api/vision/`: cameras and pole tracking; `api/led/`: status and Arduino output.
- `scripts/start_mavproxy.sh`, `deploy/`, `params/`: routing, host deployment, parameter snapshots.

## Actual communication and navigation paths

MAVProxy alone opens Cube USB serial. Multiple GRAEY nodes use separate MAVProxy UDP endpoints through Link. This differs from Crusader's single ROS `telemetry_bridge`; do not invent an existing `graey_fcu` package.

Existing navigation:
1. DVL -> dvl_node -> `/graey/dvl/velocity` and `/graey/dvl/valid`.
2. VN-100 -> vn100_node -> `/graey/vn100/imu`.
3. nav_ekf_bridge rotates/integrates valid DVL velocity into local position and sends position, body velocity and orientation in MAVLink ODOMETRY at 20 Hz through MAVProxy to Cube EKF.
4. Cube EKF estimates vehicle state; missions use local NED targets (north/east/down, meters), not simply latitude/longitude commands copied from Crusader.
5. pos_server consumes LOCAL_POSITION_NED and ATTITUDE, exposes `/pos` on port 8081 for the pool planner, and combines pole detections with pose.

Observed local-code limitations to resolve before GPS source transitions:
- nav_ekf_bridge holds last VN-100 sample with no measurement-age gate; DVL validity is a retained flag.
- Invalid DVL produces transmitted zero velocity and held position, not an explicit proof that vehicle is stationary.
- Integrated position is process-local and resets on node restart, while core launch respawns nodes.
- Link odometry sends reset counter zero and unknown covariance in inspected code. Proper validity, uncertainty and reset semantics need review.
- pos_server refreshes a shared timestamp on position OR attitude messages; its overall freshness must not be mistaken for independently fresh position.
- Saved `graey_4.5.7_vn100_ekf.parm` has GPS_TYPE=0, GPS_TYPE2=0, horizontal ExternalNav sources, but EK3_SRC1_YAW=1 despite documentation describing external VN-100 heading. Reconcile with live settings; file naming is not proof of yaw configuration.

## Kill and controller architecture

Documented power chain: Cube SERVO9 -> Pololu RC switch -> solid-state relay -> contactor -> thruster ESC supply. Physical magnetic/reed kill is independent and in series. Exact wiring has not been visually validated by this task.

Original kill logic in the background review:
- SA/CH10 high requests ESC-off, repeated disarm and MANUAL.
- SB/CH11 transitions request arm/disarm; SC/CH7 selects pilot mode.
- CH12 must alternate; cached RC values/Cube heartbeat do not prove RF transmitter health.
- Original RF pilot-loss fault latches; original reset requires RF restored, SA killed and SB disarmed. Autonomous-operation exceptions exist and need regression checking.

Later tether work:
- GUI chooses RF or Logitech-via-tether CONNECTION FAILSAFE source. This DOES NOT implement exclusive command ownership.
- Logitech -> laptop QGC -> Fathom-X tether/network -> MAVProxy -> Cube. RF -> RP3 receiver -> Cube RC input.
- QGC joystick streaming can override RF inputs even when RF failsafe is selected. Do not promise the toggle blocks the unselected controller.
- Tether implementation used interface-bound laptop ping. Ping proves network reachability, not QGC/joystick functionality. Fresh control-stream monitoring was discussed but do not claim it was deployed.
- Source changes/loss latch inhibits. Reconnection alone must not restore ESC power. Deliberate recovery required.
- User explicitly accepted that clicking Recover tether control can allow immediate arming if another sender continues arm requests. Do not claim recovery guarantees a fresh arm-button action.
- Latched SA_KILL/RF_LOST/SOURCE_CHANGED behavior and awkward source-transition sequences were documented. Treat exact current behavior as version-dependent.
- Physical power kill, disarm commands, navigation validity and manual input arbitration are distinct mechanisms.

Historical code/deployment facts, not current authorization:
- Background task verified revert main `0b0aeb6` restored kill_switch.py to original `ab4734b` after `6884645`.
- GUI/kill changes were committed to `tether_toggle_beichen`, `63e7df3`, then transferred to Jetson and rebuilt. Main was left unchanged at that time.
- Later watchdog IP correction was Jetson-only and not committed/pushed in the reported turn.
- Software checks/simulated scenarios were not full hardware validation. Operational reports do not establish complete safety testing.

## Network and operator tools (historical, recheck live)

- Jetson tether `192.168.2.2`, laptop `192.168.2.250/24`, DVL documented `.10`.
- README's laptop `.1` is stale relative to the later watchdog correction to `.250`.
- Jetson interface `enP8p1s0`; internal Ethernet switch connects Jetson/DVL/Fathom-X. Local carrier or Fathom-X lights alone do not prove end-to-end reachability.
- GUI `http://graey.local:8090/` or `http://192.168.2.2:8090/`.
- QGC `sub_graey` listener was corrected to UDP 14570. Do not blindly use standard 14550. ROS MAVProxy ports 14551 through 14556 appeared in historical process output; verify exact allocation before adding clients.
- QGC sender ID mismatch (1 versus saved SYSID_MYGCS=255) was a diagnosed possible joystick issue, not proof of present settings.
- Network/GUI health does not mean ESC power, arming or motor response is healthy.
- Do not copy credentials from conversation history into project documentation.

## GPS proposal as of September 25: NOT implemented

Extend existing stack; no mandatory replacement FCU package or bridge rewrite.
1. Read live firmware/parameters and identify physical GPS2-to-SERIAL mapping. Enable M9N reception and display/log fix quality and age first.
2. Preserve VN-100 heading and Bar30 depth after verifying actual fusion configuration. Single M9N is not Crusader's dual-antenna heading system.
3. Resolve bridge validity/reset concerns; establish consistent geographic reference and local-frame alignment.
4. Test deliberate GPS surface <-> DVL underwater EKF source transitions without jumps to estimate or goals. EKF stays active throughout. Automatic switching is a later supervised feature, not assumed fallback behavior.
5. Separate navigation supervisor/health from kill logic. GPS returning never resets a kill, arms or permits ESC power. Expected GPS loss on descent does not itself mean RF/tether failure.
6. If no usable horizontal source, inhibit position-dependent mission progression; depth/attitude/manual capability depends on healthy required sensors and existing safety state.

## Task 2 mission clarification: Chris / Beichen, September 25

User supplied Chris's plan: UAV locates both pipeline buoys and communicates their geographic positions; GRAEY transits near the surface using GPS when available, with DVL and inertial navigation bridging intermittent fixes. GPS is primarily for getting GRAEY to the pipeline. This is a team proposal, not implemented functionality.

Revised design direction: concurrent measurement fusion where supported (GPS absolute position, DVL external velocity, verified VN-100 yaw, Cube inertial propagation, Bar30 depth), with supervised degradation rather than switching the entire stack each time the antenna gets wet. Verify support and loss behavior in the actual ArduSub version before selecting parameters. A source-set transition remains an implementation option, not an assumed automatic capability.

"Zero out DVL" means correct accumulated vehicle-position drift with a qualified GPS observation. Never reset mission origin/position to zero on each GPS fix. Do not feed GPS-corrected Cube pose back as an independent external position observation. Prefer velocity aiding for the first fusion experiment; retained integrated external position needs explicit frame alignment, correlation and reset handling.

Proposed additions inside existing package: team target receiver (run/observation IDs, buoy color, WGS84 coordinates, observation time, uncertainty, acknowledgements); geographic/local frame conversion; navigation health supervisor; Task 2 state machine; pipeline/LED perception and repair interface; OCS reporting adapter. These are not existing verified nodes. UAV buoy coordinates are target observations, not GRAEY position measurements; aircraft GPS alone is not buoy geolocation. Geographic transit ends at a standoff acquisition area, then local perception guides pipeline search and precise work.

GPS gaps are tolerable only within validated uncertainty/time limits and with usable DVL/heading. DVL bottom lock at surface is not guaranteed; if both sources fail, inertial-only operation is brief degraded estimation, not indefinite navigation. Separate sensor ages and validity must be tracked. Keep existing kill and controller failsafe authority.

Rules consulted: official RobotX 2026 page links August 7 handbook, https://robonation.org/app/uploads/sites/2/2026/08/RobotX-2026_Team-Handbook-20260807.pdf . Task 2 starts at green/active buoy, surveys ordered pipeline light states, supports magnetic repair and tier-dependent UAV resource requests. Only the OCS interfaces directly with RoboCommand. Verify new revisions before final implementation. Communication transport among team vehicles remains a design decision; do not assume RF/Wi-Fi underwater. Cache accepted mission targets and reports with an explicit communication-loss policy.

## Attachment inventory and review debt

Archive located and inventoried on September 25:
`C:/Users/libei/Downloads/Graey-20260905T225335Z-1-001.zip`

Entries:
- `Graey/GRAEY UUV — DOCUMENTATION (RobotX 2026).docx`
- `Graey/Graey water testing checklist.docx`
- `Graey/LED Matrix.docx`
- `Graey/QGC params (by date)/Graey_params_08.11.2026.params`
- `Graey/Electrical/Graey_EMO.drawio`
- `Graey/Electrical/Relay EMO` (extensionless; inspect type before opening)
- `Graey/Electrical/Graey_Electrical_Schematic_2026` (extensionless)

Background task also contains screenshot attachments. None should be called fully reviewed merely because their filenames appeared in retrieved messages. A separate electrical PDF exists in Downloads but was not established as an attachment to that task.

Full independent document and visual wiring review remains outstanding. Preserve this distinction in future answers. When reviewing attachments, record exact source, date, coverage (text versus visual), discrepancies and resulting corrections here.

## Live GPS configuration update — September 25, 2026

User authorized parameter writes. SSH through MAVProxy UDP 14554 verified Cube disarmed before writing and rebooting. Readback after reboot confirmed SERIAL3_PROTOCOL=-1, SERIAL4_PROTOCOL=5, SERIAL4_BAUD=38, GPS_TYPE=1, GPS_TYPE2=0, GPS_AUTO_CONFIG=1. Changed values were SERIAL3_PROTOCOL (5 to -1) and GPS_TYPE (0 to 1). Pre-change snapshot of relevant parameters saved inside graey container at /root/robotx_ws/logs/gps_before_20260926_040635.json.

Live EK3_SRC1_POSXY=6, VELXY=6, POSZ=1, YAW=6 remained unchanged. This supersedes assumptions based on saved yaw settings. After the reboot observation window, GPS_RAW_INT still reported fix_type=0, satellites_visible=0, zero coordinates: GPS reception/detection has NOT been verified. A working receiver without satellite lock should normally report fix_type=1; do not explain fix_type=0 solely as poor sky view. Further power/cable/UART/protocol diagnosis remains necessary. QGC vehicle map position uses estimated global position and is not guaranteed to reflect raw GPS while horizontal sources remain ExternalNav.

## September 26: GUI and remaining work

- Later September 26: user explicitly authorized deployment of the reset GUI. Applied the four-file GUI/test patch to the sub's existing tether_toggle_beichen checkout at bb3ab30 as uncommitted changes, installed only gui_node.py/navigation_view.py, and restarted only gui_node. All 11 tests passed on Jetson; /navigation serves the reset button, /api/navigation exposes display_reset with fresh GPS/global/attitude/EKF/heartbeat streams, and /api/controller returns HTTP 200. Existing local tracked edits were hash-verified unchanged; main remained fc311038ab27a8b40c8bb55d9dfb58ff80aa9003. Installed-module backup: /root/robotx_ws/logs/gui_reset_backup_20260926_231453 inside the container. This supersedes the earlier not-yet-deployed note below. No EKF parameters changed; reset itself was not triggered on the live vehicle during deployment verification.

- User reports the tether issue fixed and asks to check it off. Record as user-reported completion, not independent connection/failsafe validation.
- Navigation GUI and optional Grid/Streets view were implemented in commits e382ad0 and bb3ab30 on tether_toggle_beichen and deployed during the preceding session. Main and existing sub-local edits were preserved. GUI reads Cube MAVLink via its existing UDP 14555 connection, plus DVL and VN-100 heading ROS topics. This does not configure GPS EKF fusion.
- User authorized a display-only reset-origin/track feature. Local implementation clears both trails, waits up to 15 seconds for a newer timestamped valid GPS fix, establishes a new display origin, labels it on both maps, and recenters connected browsers. Repeated cached fixes cannot complete reset. On timeout, retry is explicit. All open GUI clients share the display reference; no Cube, mission, kill, or parameter reset is sent. This feature is not yet committed or deployed as of this entry.
- GPS EKF integration and investigation of previously absent DVL/VN-100/local-position telemetry remain separate pending work. Historical missing streams are not proof of current sensor status.

### GPS follow-up: receiver verified working

Subsequent live read-only check on September 25 received 101 GPS_RAW_INT packets over 25 seconds. Latest sample: fix_type=3 (3D), 16 satellites, HDOP=1.04, reported horizontal accuracy=1.738 m, vertical accuracy=1.892 m. GLOBAL_POSITION_INT contained matching nonzero coordinates. Cube heartbeat indicated disarmed. This supersedes the initial post-reboot no-GPS observation; M9N reception through configured SERIAL4 is now verified. No additional parameters changed. Matching global coordinates do not by themselves prove GPS horizontal EKF fusion or completed navigation integration.

## Bottom camera live check — October 2, 2026

User authorized SSH and bottom-camera testing. Connected as graey@graey.local. Deployed tools/cam_pair.py maps DOWN to MxId 14442C10C1B6BED200 and FRONT to 14442C1031B3BFD200. Both enumerated; no existing camera process was observed. A temporary stdin Python probe inside the graey container opened only DOWN using the existing camera.build_pipeline at 15 fps with RGB and depth, pinned to UsbSpeed.HIGH (USB 2), matching the deployed camera-pair configuration. Over ten seconds it received 148 RGB and 148 depth frames at approximately 14.999 fps, sequence numbers 0–147 and advancing device timestamps. RGB was 640x360; visually inspected snapshot shows floor and cables. Last depth frame had 37.84% nonzero pixels; metric depth accuracy and underwater performance were not tested. Existing pipeline requested an unsupported sensor resolution and DepthAI reported fallback to 1080p; preview output remained 640x360. Device closed cleanly. No vehicle code, parameters or services changed. USB 3 capability was not tested. This is a dated observation, not a guarantee of ongoing camera status. Local evidence: output/camera-check-2026-10-02/result.txt and bottom.jpg. Credentials were not recorded.

## Navigation supervisor repository work — October 2, 2026

User authorized conditional toggle-to-main merge, navigation supervisor, dive GPS
reference/GUI marker, source profiles and simulations. They explicitly approved
a Jetson mission reference paired to healthy Cube local pose, preserving Cube
origin; retaining integrated external position underwater with one-time alignment
on return from GPS; checking estimate health before switching; and human source
labels Underwater/Surface GPS. These supersede the earlier velocity-only proposal.

Fetched repository history: origin/main was 0b0aeb6. The toggle differed by only
63e7df3 (GUI/kill toggle), e382ad0 (GPS GUI), bb3ab30 (Streets map). Local main was
fast-forwarded to bb3ab30 under the user's conditional authorization. Existing
uncommitted four-file GUI reset work was stashed and restored. No remote push or
vehicle deployment was performed in this implementation pass.

Local implementation adds navigation_supervisor.py and pure supervisor_logic.py,
mission_reference.py, separate /navigation-supervisor page and dive marker on
/navigation, atomic DVL validity/velocity samples, stale odometry suppression,
one-time bridge alignment with reset counter, optional MissionBase permission
gate and translated mission zero. MAVProxy still owns serial; new supervisor uses
loopback UDP14557/component196. Source profile and optional Cube NAV_SRC Lua
reporter are files only, not installed/applied. Existing sensor GUI and kill
authority remain distinct.

Defaults: supervisor observes; bridge alignment disabled; mission supervision
disabled; pressure surface calibration unavailable until configured. Activation
requires explicit commissioning flag, matching source parameter readback, real
selected-source feedback, valid calibrated depth, healthy fresh sensors and
intent. Short GPS dropouts retain Surface GPS with a bounded grace interval;
longer loss blocks permission. Source confirmation needs both ACK and selected
source report. Source labels do not prove actual GPS fusion.

Important limits: optional mission interruption policy requests MANUAL/pilot
takeover and latches stopped, including on permission loss during transitions;
seamless autonomous transitions are not commissioned. Whole ODOMETRY suppression
also removes external yaw when DVL is invalid. Bridge restart still resets its
position before supervisor fault detection; no guarantee that zero odometry
cannot reach Cube first. Covariance remains unknown. pos_server keeps Cube
coordinates. No full Task2 target receiver/mission was added. See repository
docs/navigation-supervisor.md for exact gates and remaining vehicle tests.

Validation: 46 Python tests passed, including 20,000 randomized supervisor ticks,
reference conversions/persistence, GUI reference/reset separation and production
callbacks with mocked ROS/transport. Python compilation and all three GUI pages'
inline JavaScript syntax checks passed. These do not validate ROS DDS or water
behavior. Separate stationary official ArduSub4.5.7 SITL experiment (no arming)
confirmed local-position startup with external position and source command
acceptance; GPS disabled produced uninitialized EKF on GPS selection. Timing
under tracing was limited. Evidence: output/gps-sitl-2026-10-02. No claim of live
EKF source integration or complete transition validation is warranted.

Repository implementation committed as 1ee606d on codex/navigation-supervisor;
working tree verified clean afterward. Local main remains the authorized toggle
merge at bb3ab30. Neither branch was pushed in this pass.

Follow-up: user requested placing the work on tether_toggle_beichen. Fetched
origin, fast-forwarded that local branch to 1ee606d and successfully pushed only
tether_toggle_beichen to GitHub. The checkout now uses tether_toggle_beichen.
Remote main and the vehicle were not changed by this follow-up.

Next follow-up: user explicitly requested merging the older toggle work to
GitHub main while excluding the new GPS supervisor. Fetched history and pushed
the fast-forward origin/main 0b0aeb6 -> bb3ab30. Main now includes the original
RF/tether toggle and existing GPS GUI/Streets commits. Supervisor commit 1ee606d
remains only on tether_toggle_beichen relative to main. Checkout remains on the
toggle branch, clean; no vehicle deployment performed.

Jetson repository update follow-up: user authorized SSH over Wi-Fi and updating
both branches. Verified SSH used wlP1p1s0 (Wi-Fi IPv4 192.168.8.105). In
/home/graey/robotx_ws/src/robotx_graey_2026, fetched origin, advanced local main
to bb3ab30 and checked-out tether_toggle_beichen to 1ee606d. Preserved tracked
edits in stash named before-navigation-update-2026-10-02, then reapplied them.
GUI reset overlap conflicts were inspected: new HEAD already contained all old
reset changes plus supervisor additions, so HEAD resolved those three files.
Jetson-specific service network ordering, tether watchdog .250 and QGC port
14570 edits remain uncommitted; stash retained as backup. Untracked sonar code,
parameter snapshots and image were left in place. All 46 tests passed on Jetson
Python3.10; diff check passed. No build, service restart, Cube parameter write or
active supervisor commissioning performed. Source files are updated, but this
does not establish that installed/running Python nodes use the new code.

GUI 404/tab troubleshooting — October 2, 2026: user clarified the GPS and
supervisor displays must be tabs within the main GUI, with System remaining
available. Wi-Fi SSH confirmed the live GUI returned 404 for
/navigation-supervisor while / and /navigation returned 200. Running gui_node
was the installed executable dated Aug 29; it predated the route even though the
source branch contained it. The root page's GPS/supervisor controls navigated
away from the root tabs, explaining why System disappeared. Commit 5136bf6
replaces those links with same-page GPS & Pose and Navigation Supervisor tabs,
embedding the existing views and fixing selected-tab mapping; links inside the
embedded views return to the top-level page. Pushed to tether_toggle_beichen.
Jetson fetched the commit, built robotx_graey_2026 with colcon, and restarted
graey-ros while Cube status was disarmed/MANUAL. Verified /, /navigation and
/navigation-supervisor HTTP 200, root HTML includes GPS & Pose, Navigation
Supervisor and System tabs, and ROS node /navigation_supervisor is running.
Supervisor currently reports unavailable EKF estimate, Cube position, DVL and
bridge data; the tab is reachable but navigation is not presently healthy.
Local Jetson service-order, tether-IP, QGC-port edits and untracked sonar work
were preserved. Prior stashes remain. No Cube parameter changes or arming action.

Local software-in-the-loop setup — October 3, 2026: added an isolated WSL2
ArduSub SITL workflow in `robotx_graey_2026/sitl` and
`robotx_graey_2026/launch/sitl.launch.py`. It loads a SITL-only source profile
mirroring the repo's candidate EKF sets, then launches the production
`nav_ekf_bridge`, active `navigation_supervisor`, and GUI with synthetic sensor
inputs. A SITL-only mission driver runs surfaced GPS fix/reference capture,
Underwater source selection, a 0.5 m dive with simulated GPS loss, one GPS target
under DVL/IMU navigation, GPS restoration and surfacing. This is local simulator
evidence only; it does not establish live Jetson/Cube code, parameters, wiring,
or vehicle performance. It never uses the real vehicle connection.

The SITL supervisor accepts an optional `depth_topic`; the default remains empty,
so production continues deriving water depth from the configured Cube pressure
message. The simulator supplies local NED depth on that topic. GUI mission
waypoint display is passed through supervisor status onto both the Navigation
page's offline Grid and online Streets layers. GUI static assets resolve relative
to the package path instead of a Jetson-only absolute path, so local WSL serves
`/navigation` too.

Verified final SITL outcome (October 3, 2026; `docs/sitl-validation.md` and
ignored `robotx_graey_2026/sitl/logs/scenario-result.json`): source sequence
Underwater → Surface GPS → Underwater → Surface GPS; dive target 0.500 m reached
at 0.502 m; GPS loss confirmed with fix type 1; underwater waypoint reached at
0.248 m horizontal error and 0.490 m depth; GPS recovered with fix type 6; final
supervisor state Surface GPS. WSL build and 47 unit tests passed. Separate local
failure checks confirmed stale DVL/IMU blocks `navigation_ready` and recovers when
the simulated sensor publisher resumes. The configured coordinate
32.9240586, -117.0385389 is an approximate road geocode from the user's
street-level input, not a verified pool location. The demo waypoint is an offset
from that road reference and must be replaced with measured pool coordinates for
backyard-accurate map results.

QGC marker oscillation follow-up — October 3, 2026: the initial statement that
local SITL was stopped was incorrect; it inspected default WSL `Ubuntu` rather
than `Ubuntu-22.04`. The latter was still running ArduSub system ID 1 at the
street geocode and MAVProxy forwarding to Windows QGC UDP14550, competing with
the physical vehicle's identity. Stopped the verified local simulator MAVProxy
process; the wrapper cleaned up its SITL/ROS children. Confirmed no remaining
simulator processes or UDP14551–14559 listeners in Ubuntu-22.04, and fresh real
Graey GPS/global telemetry continued to match. QGC screen behavior after cleanup
was not independently inspected. Local sitl/run.sh now disables QGC forwarding
unless QGC_HOST is explicitly supplied for a SITL-only QGC instance. No vehicle
parameters, Jetson services, or QGC connection settings were changed. Use
`wsl.exe -d Ubuntu-22.04` explicitly when checking this simulator.

Tether watchdog target request — October 3, 2026: user requests laptop ping
target 192.168.2.1. The local tether_toggle_beichen checkout already defaults
`tether_ip` to 192.168.2.1 in kill_switch.py. Last successful live telemetry had
reported 192.168.2.250. The requested live change is pending: SSH timed out at
graey.local (resolved 192.168.8.105); TCP22 was unreachable on both that Wi-Fi
address and vehicle tether address 192.168.2.2, and the GUI HTTP request timed
out. Do not treat the requested target as deployed. Vehicle tether address
192.168.2.2 is distinct from the laptop/watchdog target 192.168.2.1.

Follow-up: Wi-Fi SSH subsequently succeeded at 192.168.8.105. Confirmed the
deployed default was .250. With fresh GUI telemetry confirming disarmed,
changed only tether_ip to 192.168.2.1 in Jetson source and installed Python
module, AST-checked both, and backed up each with suffix
`.before-watchdog-192-168-2-1`. Sent SIGTERM to the verified kill_switch process
for launch-managed respawn. Local repo already used .1 and required no code
change. Post-restart HTTP/SSH attempts timed out, so the live runtime target,
respawn success and ping reachability remain unverified. Do not automatically
clear any startup inhibit when connectivity returns.

## Supplied GPS telemetry review and local hardening — October 3, 2026

User supplied graey_2026-10-03_session_telemetry.tlog and authorized offline
patching/tuning; Graey is unavailable. Analysis found 285 consecutive global
coordinate steps over 1 m (interval <2 s), 284 with EKF flags167 (constant
position, no valid horizontal position). Local-position reports show no >1 m
steps and disappear before most global jumps. Logged ODOMETRY exists only in
the early segment; this does not establish later DVL hardware failure. Source1
horizontal aiding is ExternalNav; recorded source2 horizontal sources/yaw are
disabled and SRC_OPTIONS=1. No source commands or NAV_SRC feedback were logged.
Thus the log does not demonstrate GPS overpowering a healthy DVL-aided EKF.

Local tether_toggle_beichen changes add a GPS/DVL motion-consistency screen,
differentiate rejected GPS from missing measurements, request external aiding
after rejected/prolonged-lost GPS subject to existing commissioning/health and
alignment gates, and withhold unhealthy global/GPS markers while keeping raw
coordinates visible. Supervisor publishes GPS rejection reasons. The direct
M9N-to-Cube path remains: Jetson cannot veto each GPS packet, and fallback does
not undo an already erroneous Cube position. No vehicle settings, deployment,
main branch, commit or push changed. Existing uncommitted SITL work preserved.
Numeric EKF noise/innovation tuning awaits Cube .BIN and current full parameters;
the tlog lacks that evidence. See docs/gps-telemetry-review-2026-10-03.md and
tools/audit_navigation_tlog.py. GUI replay withheld all284 global steps associated
with invalid horizontal EKF flags; this is not validation of physical navigation.
Validation: 57 regression tests, Python compilation and both navigation pages'
JavaScript syntax checks passed. No new full ArduSub SITL or vehicle trial.

## QGC forwarding correction — October 4, 2026

User authorized correcting QGC forwarding and restarting MAVProxy. Laptop Ethernet was 192.168.2.1; QGC PID3656 listened on UDP14570 and14550. Live Jetson GUI heartbeat was fresh/disarmed. Old MAVProxy broadcast socket was connected to stale laptop192.168.2.250:14570; neighbor entry .250 was INCOMPLETE while .1 was REACHABLE. Added /etc/systemd/system/graey-mavproxy.service.d/qgc-tether.conf overriding ExecStart to pass192.168.2.1 to existing start_mavproxy.sh, selecting udpout:192.168.2.1:14570. Reloaded systemd and restarted graey-mavproxy. Captured864 outgoing UDP packets to .1:14570 over5seconds. Old container MAVProxy PID48 survived service stop; explicitly terminated that verified stale process. Final process inspection showed only new host PID12911 MAVProxy and fresh GUI heartbeat, disarmed/MANUAL. QGC UI reconnection was not independently inspected. Override intentionally selects tether laptop only; future laptop address changes require updating it. No repository source or Cube parameters changed. Vehicle clock reported October3 while client date was October4; date here follows client context.

Pool mapping follow-up — October 4, 2026: user authorized mapping and pushing pending repo work. Commit 02c5027 pushed to tether_toggle_beichen; main unchanged. Includes prior local GPS screening/SITL work and approximate lab pool image scale (5 m over 620 pixels), browser-local per-image calibration, and separate position/attitude freshness for pos_server. Only pool_planner.html and pos_server.py deployed to Jetson with backups; planner feed started via GUI API. No mission or EKF parameter changes. 57 unit tests and planner JS syntax passed. User identifies backyard pool and will supply a picture; precise start/facing and geographic registration remain pending. GPS/SITL changes are pushed but not deployed in this pass.

GPS/SITL deployment — October 4, 2026: explicit user authorization. Jetson tether_toggle_beichen advanced via Git bundle (Jetson GitHub DNS failed) to 2496a82, matching pushed GitHub branch. Tracked predeploy changes backed up in stash before-gps-sitl-deploy-20261004; service/MAVProxy routing overrides restored, untracked work preserved. Colcon build succeeded; 57 tests passed on Jetson. GUI and supervisor restarted while fresh telemetry showed disarmed, ESC off and no mission. Fixed non-symlink GUI asset packaging in 2496a82; root/planner/navigation return HTTP200. Updated supervisor publishes GPS rejection reason, remains Observation with navigation_ready=false and missing EKF/Cube local position/DVL validity/bridge. No source parameters or activation flags changed. SITL source/launch/profile installed but simulator not launched; run.sh still requires a suitable SITL binary (default path targets laptop WSL), so this does not claim standalone Jetson SITL readiness.

October 4 pool registration: user provided green point 32.92357,-117.03855 and screenshot with north-up orientation and 10 ft bar. Commit4193ff5 pushed and Jetson branch fast-forwarded with routing overrides preserved. New /pool-map uses original screenshot reference pixel924,850 and 3.048m/150px scale, fits viewport, shows qualified raw GPS separately from valid Cube global estimate. Screenshot reference is approximate, not surveyed; old screenshot vehicle glyph masked as image overlay. GUI rebuilt and restarted disarmed; no EKF parameters changed. Live check before restart: flags167, no local pose, stale DVL validity, supervisor Observation/not ready; GPS25 sats hacc1.199m, VN heading121.392deg. This is dated status, not continuing proof.

Stationary GPS capture at repositioned setup — October 4, 2026: after user reported moving Graey to improve DVL bottom lock, confirmed live disarmed, MANUAL, fresh vehicle, ESC off and no mission. Captured GUI telemetry at 2 Hz for 299.8 s to `output/stationary-gps-20261004/telemetry-new-position.jsonl` (548 valid GPS records). Mean raw GPS was 32.9236205, -117.0385269. Horizontal scatter around that mean: 0.53 m RMS, 0.84 m 95th-percentile radius, 0.91 m maximum. Reported hAcc in first/last records 1.666/1.252 m. GPS fix 4D, 24–25 satellites. Every EKF report flags167; zero Cube local-position samples; zero valid DVL samples. Supervisor stayed Waiting for healthy navigation / Observation. Cached parameters show EK3_SRC1 POSXY=6, VELXY=6, YAW=6 (ExternalNav); GPS source set2 POSXY/VELXY/YAW=0. No parameter modifications. This measures short-term stationary scatter only, not absolute accuracy: exact surveyed coordinate for the repositioned Graey location is unknown. User reported moving to improve bottom lock, but DVL validity did not recover during capture. QGC map offset cannot be attributed to EKF from this sample because horizontal EKF position is invalid and GPS is not selected in source1.

## Stationary GPS and DVL comparison — October 4, 2026

Following user confirmation Graey was disarmed, captured 179.7 s (308 samples) of
GUI telemetry with armed-state guard in every iteration. No vehicle commands or
parameter changes. JSONL evidence at `output/stationary-gps-20261004/telemetry-dvl-lock.jsonl`.
Raw GPS coordinates varied 1.34 m north x0.90 m east; reported hAcc 1.28–1.35 m;
fix4, 22–24 satellites. Cube global varied 0.12m north x0.27m east; fresh local
pose throughout. Raw GPS/Cube global separation mean7.08m, range6.48–7.63m. DVL
valid307/308, mean clearance1.10m; EKF flags831. Supervisor stayed Fault /
Observation due “Navigation bridge restarted; external frame must be revalidated”;
source configuration unverified. No EKF changes. This does not establish GPS fusion
or absolute accuracy. Cached parameter snapshot has SRC1 horizontal pos/vel/yaw=6
ExternalNav, SRC2=0 None; live parameter readback still required. ROS logs show earlier VN100 serial read/reopen warnings; process listing was ros2-run wrapper/child pairs, which do not establish duplicate sensor nodes. Not proven active faults during the capture. Search found no BIN dataflash
log on Jetson/container. End state remained disarmed, SA_KILL inhibit active. See
`docs/gps-stationary-check-2026-10-04.md` for interpretation and next diagnostics.

## Cube DataFlash and source-state correction — October 4, 2026

The Cube LOG protocol reported log 446 at 218,787,840 bytes. A full transfer was
not attempted over the 115200-baud serial owner because it would be unnecessarily
long; the retrieved evidence is a raw 64 KiB header plus a raw 4 MiB recent
segment at `output/stationary-gps-20261004/ekf-log-446-header.bin` and
`output/stationary-gps-20261004/ekf-log-446-tail.bin`. The joined file is an
intentionally incomplete header-plus-tail sample, not a complete `.BIN`.
The segment contains 4D GPS (24–26 satellites, GPA HAcc 1.20–1.33 m), 4,937
XKF1/XKF3 records and 4,938 XKF4 records. XKF4.GPS is 0 for every decoded
record; XKF3 innovations are present, so this sample does not show GPS being
used as the active EKF horizontal source. Exact enum interpretation and current
parameter readback remain separate checks.

The live supervisor was inspected while the vehicle endpoint reported armed. It
had `depth_m:null`, `surfaced:false`, `requested_source:"Underwater"`, and
`confirmed_source:"Unknown"`, while the supervisor was in Observation/Fault.
That was a false classification caused by initializing an unqualified depth
state as submerged. The local supervisor logic now uses an explicit Unknown
state until calibrated depth stays inside the surface or underwater hysteresis
band for its dwell time, and the GUI hides depth-derived source/state labels
when depth is unavailable. The GUI fallback was copied to the installed Jetson
asset and verified by HTTP; the navigation process was not restarted while the
vehicle was armed. The supervisor logic patch passes 17 unit tests in the Jetson
container. The logic source is staged in the tether worktree but not committed
or pushed in this pass.

## Repository hardening and audit — October 6–7, 2026

Repository-only work authorized on tether_toggle_beichen. Added Linux exclusive
ownership before construction for DVL/VN/bridge/supervisor, disabled GUI navigation
start/stop, made prequal core optional, disabled bridge per-node respawn, and
latched bridge identity overlap/restart faults. Fixed component-ID duplication,
VN reconnect cleanup, DVL malformed-report handling, GUI vendor packaging/encoding,
and SITL isolation/freshness/waypoint/failure-reporting issues. No Jetson deployment,
Cube parameter changes, main changes or live vehicle access in this session.
79 Linux tests passed; all 5 inline HTML scripts passed JavaScript syntax checks.
Three bounded ArduSub SITL runs did not complete successfully; final run failed
source confirmation and recorded passed=false. Do not treat prior success logs
as validation of this revision. See repository docs/software-pipeline-2026-10-06.md,
docs/navigation-recovery-plan-2026-10-06.md and docs/overnight-validation-2026-10-06.md.

Correction to earlier context: XKF4.GPS=0 is NOT proof that GPS was not fused;
it is a filter GPS-status field. Source-set parameter readback is configuration,
not selected-source proof. GPS fix value4 is DGPS, not 4D. The partial DataFlash
TimeUS span was about180.7 seconds, not30minutes. Partial decoding is insufficient
for confident numeric EKF tuning. Historical state remains dated evidence.

## Source-switch confirmation work — October 8–9, 2026

User authorized source-switching fixes and isolated local simulation on
`tether_toggle_beichen`; on October 9 explicitly confirmed Astra light and resumed
the work after a model/usage pause. Changes are local working-tree changes, not a
Jetson deployment or a remote synchronization claim. Main and live Cube parameters
were not changed. Supervisor now rejects late/pre-request confirmation evidence,
ignores unsolicited/wrong-target ACKs, retains timely evidence across subsequent
reports, checks position continuity in both directions, and records transaction
and callback timing diagnostics. MAVLink ACKs lack wire transaction IDs, so the
internal trace ID must not be described as perfect delayed-ACK correlation.

94 Linux software tests passed. Three consecutive full ArduSub SITL scenarios
from the October 8 diagnostic snapshot completed successfully; missing ACK/source
injection also demonstrated faults with navigation readiness withheld and the
simulator disarmed. Earlier nominal runs aborted on >1-second status ages. Later
logs show occasional ~1-second supervisor callback gaps with ~15ms MAVLink drains;
the cause of scheduling delays remains unproven. Neither passing repeats nor these
supervisor changes establish physical GPS/heading accuracy or live readiness.
See `robotx_graey_2026/docs/source-switching-session-2026-10-08.md` for the final
batch results, exact evidence paths, reproduction commands and remaining limits.

## Live dry-land baseline — October 9, 2026 (Pacific)

User authorized inspection with Graey on land. Tether SSH to 192.168.2.2 succeeded
via laptop Ethernet 5 / 192.168.2.1. Vehicle repeatedly confirmed disarmed, MANUAL,
SA kill active, ESC commanded off, no mission; RF control selected, both links
healthy. No deployments/restarts/parameter writes/origin resets performed.
Live checkout HEAD 1e09b328576dfb8c5341fe10daab1c23e3e92ade on tether_toggle_beichen;
cached upstream reports ahead9 but no remote fetch was performed. One executable
per navigation node listed. VN heading and DVL streams stale for tens of minutes
despite running processes; their power/byte reception not established. GPS fresh;
60-sample capture over62.8s measured0.423m north and1.215m east scatter, hAcc
1.442–1.481m. No surveyed absolute-accuracy conclusion. Cube horizontal position
invalid, EKF flags167, no local-position messages during20s readback.
Live readback SRC1 POSXY/VELXY/YAW=6, SRC2 corresponding values=0,
EK3_SRC_OPTIONS=1, SCR_ENABLE=0, LOG_DISARMED=1. No NAV_SRC observed. Supervisor
Observation, selected source Unknown. Intended surface source/reporting is not
commissioned; parameters do not prove which source is selected. Restore fresh
sensor telemetry before heading comparison or EKF tuning. Evidence/report at
`output/dry-land-20261009/README.md`; selected parameters saved, not a full backup.

Later the same evening, VN telemetry recovered. Two user-performed approximately
90-degree clockwise turns gave VN/Cube headings337.11/154.49 ->63.09/240.29
->154.32/330.21 degrees: rotation increments agree, with near180-degree absolute
disagreement. User judged Cube240deg southwest physically plausible. Live ROS
parameters: VN flip_180=true, yaw_offset_deg=0; bridge yaw_offset_deg=0,
allow_alignment=false. Installed driver applies upside-down roll/pitch/vector
correction but no heading reversal; installed bridge adds no yaw correction.
ROS IMU quaternion yaw matches VN heading. Bridge healthy=false withholds odometry
including external yaw, and supervisor reports bridge-restarted fault. No changes
or resets made. Mounting axes must be physically verified before applying a yaw
or full-body transform; do not treat Cube agreement as surveyed heading truth.

Subsequent user-authorized temporary +180 VN offset/restart was NOT applied:
single-owner precondition aborted before any mutation. Live process tree then
confirmed duplicate actual VN, DVL and bridge executables: GUI-owned children
5854/5853/5852 respectively (via ros2-run wrappers under GUI88), and core-launch
children5879/5877/5873 under27. Earlier single-instance inventory is no longer
current. Requires resolving GUI/core competing ownership before offset test;
no source changes, signals, or parameter writes occurred in the failed attempt.

User then authorized duplicate cleanup and continuing the temporary offset test.
Stopped GUI-owned VN5854/DVL5853/bridge5852; their wrappers exited into zombie
state awaiting GUI reaping. Verified only core executables remain. Core bridge5873
and DVL5877 retained. VN5879 alone restarted as14916 with verified runtime offset
+180deg. Original installed VN source restored byte-for-byte after startup; offset
is TEMPORARY and returns to0 on next VN restart. Backup in container at
/tmp/graey-vn-offset-90j9prbh/vn100_node.py.original. Fresh VN334.426 vs Cube330.058
degrees (~4.37deg difference). Disarmed/ESCoff/SAkill maintained; supervisor remains
Observation/Fault, navigation not ready. No Cube parameters, permanent repo code,
or bridge position resets performed. Body mounting correction still unvalidated.

Later October 9 live cleanup: GUI/core duplicates recurred after sensor start/stop. User authorized removal; initial attempts made no changes because SA kill was released. Once controller reported disarmed, SA_KILL and ESC commanded off, removed GUI VN24743/DVL24741/bridge24742 and retained core VN24767/DVL24769/bridge24765 without restarting them. Verified single executable per node. Post-cleanup DVL valid and fresh (~0.105s), VN fresh; supervisor remains Observation/Fault (bridge restarted), not navigation ready. No permanent GUI prevention deployed, no fault reset or Cube parameter changes. Earlier temporary VN offset must not be assumed to persist across the intervening VN restart.

October 9 manual pool tests (documented October 10): captures manual-20261010T044222Z.jsonl and manual-20261010T045125Z.jsonl under output/dry-land-20261009. Repeat had222/222 fresh valid DVL samples over64.9s, approximate DVL net1.29m versus Cube local1.30m, heading agreement within1.24deg. Heading varied15deg; no independent distance/heading truth or scale calibration. Raw GPS shifted1.94m in a different direction. This supports relative consistency only, not absolute accuracy or proof of fusion. Supervisor remained Observation/Fault, sourceUnknown. Permanent GUI/core duplicate prevention remains undeployed. See robotx_graey_2026/docs/pool-session-2026-10-09.md for full session and remaining checklist.

## Standing workflow authorization — October 10, 2026
User instructs future file edits to include proactive repository updates (commit/push related work to tether_toggle_beichen). When user says Graey is ready and online, proactively reconcile and sync the relevant repository changes with Jetson. Preserve unrelated work and vehicle-specific configuration; verify state before service disruption and do not restart navigation while armed. This does not authorize main changes, arming, automatic fault clearance, or arbitrary live EKF parameter changes. Keep the repository copy of this context current.


October 10 repository GUI fix: existing HTTP navigation start/stop rejection supplemented by subprocess-helper rejection of VN/DVL/bridge/supervisor launches. Nine Linux ownership tests passed. Vehicle deployment and live restart/click verification remain outstanding; do not claim last night's duplicates are permanently prevented on Jetson yet.

October 10 authorized Wi-Fi deployment: Jetson fast-forwarded to navigation executable revision87e8788, colcon built and core restarted while freshly disarmed/ESCoff/no mission. GUI/core duplicates removed; deployed HTTP navigation start/stop now409, exactly one VN/DVL/bridge/supervisor verified. Local sonar GUI changes reapplied and unrelated untracked sonar files preserved, so whole checkout is not clean/identical. Runtime VN offset0/fliptrue and bridge offset0 verified after cleanup. Fixed cached-frame parameter writes by making settings startup-only; separate descriptors corrected a metadata alias found live. Final97 Jetson tests passed. Supervisor now Observation/Waiting for healthy navigation due Depth reference unavailable, sourceUnknown/config unverified; restart is not validated frame recovery. No Cube parameters or arming changes. Details: docs/deployment-2026-10-10.md in repo.
