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

October 10 final sync reconciliation: concurrent sonar merge preserved via7160cab; Jetson fast-forwarded with empty tracked diff, matching installed navigation code and GUI; unrelated untracked backups/data remain. Sonar commit2c007c8 reports physically measured inverted VN pitch and downstream VN_PITCH_BACKWARDS compensation. That test was not repeated here; coordinate any upstream pitch fix with downstream compensation removal. This is distinct from yaw offset.

## Pre-dive heading consistency gate — October 10, 2026

The user approved a GPS-course/VN-100/Cube heading consistency gate before
switching from a surfaced state to the underwater source profile. This is a
supervisor check, not a GPS yaw correction: GPS course-over-ground is compared
with the two heading estimates only during valid, sustained, straight forward
DVL motion. Stationary, reverse, strafe, vertical, turning, stale or poorly
qualified motion skips the course comparison and may rely on attitude only if
the heading north references and forward axes have been commissioned. Sustained
disagreement blocks dive preparation until sustained agreement returns. The
default heading-reference validation remains false; offsets and thresholds are
read-only startup parameters. No Cube yaw, EKF, arm, kill, or ESC command is
changed by this feature.

Correction during October 10 waypoint work: the prior assertion that VNYBA gyro
fields are degrees/second was wrong. Manufacturer UM005 section 9.2.9 specifies
radians/second, consistent with the existing ROS driver. The supervisor now
converts radians to degrees for its turn threshold, with a regression test. No
driver/bridge units or vehicle parameters were changed. The DVL check assumes
its configured body-forward axis is +x, as used by the existing bridge and
simulator; verify that during commissioning.

This feature is local source work only. No vehicle access, parameter write,
deployment, or in-water check was performed. The pre-dive gate and source
transition require software tests and isolated simulation; physical heading
north calibration and surface-to-underwater behavior remain unverified.

## Click-to-waypoint implementation — October 10, 2026

User authorized implementing/verifying a map-click mission: mark surface start,
dive to 0.5 m, travel underwater, surface at the clicked waypoint, and display
Cube location/heading throughout. Implemented locally on tether_toggle_beichen:
main GUI Waypoint mission tab /waypoint, shared /pool-map page, reviewed single-use
plans, owned waypoint_mission controller, explicit live commissioning gate.
GUI starts one mission process; supervisor owns EKF source selection/reference;
bridge remains core-owned; Cube estimates and controls. No EKF origin reset.
Cube global/ATTITUDE telemetry supplies the marker on surface and underwater;
raw GPS stays separate. Pressure-to-local-down offset is frozen before descent.
Underwater aiding is retained through ascent; surface GPS recovery follows disarm.

SITL exposed and corrected the pre-dive heading gate expiring during depth
hysteresis. It now remains approved for a prepared current dive with confirmed
Underwater source and saved reference, and resets on new run/SURFACE intent.
SITL timing investigation measured WSL wall-clock rollback (-2.260 s) without a
monotonic scheduling gap. Bridge/supervisor/GUI/waypoint/SITL timers now use a
steady clock; synthetic sensor timestamps use steady ordering. ROS wall clock
and physical VN/DVL timestamp conventions are unchanged. Live clock-sync/restart
behavior remains to be checked; no sensor freshness limits were relaxed.

Validation: 130 tests passed with ROS loaded/no skips. Three complete isolated
waypoint missions passed after the clock fix, including GPS absence underwater,
Cube heading/location display, surface arrival/disarm/GPS recovery. Two operator
abort runs and missing-ACK/missing-source-report injections passed. Earlier
failed runs are retained and explained in docs/waypoint-mission-2026-10-10.md.
No QGC forwarding, vehicle access, deployment, physical arming, live parameter
writes, or main changes occurred. This is simulation validation, not wet readiness.

Mapping remains approximate (existing screenshot reference/scale/north-up).
Live pool-image mission starts are blocked pending a measured survey and boundary
validation; the software does not yet ingest a survey. Proposed solution:
multiple measured pool control points plus independent hold-out check, then
independent vehicle position/heading and underwater endpoint measurements.
IMU/DVL cannot identify a constant absolute GPS/map bias. Live waypoint execution
defaults disabled; GRAEY_WAYPOINT_ENABLED is only a commissioning opt-in, not a
replacement for source/depth/heading/control checks. No collision avoidance or
validated pool geofence has been added.

### Verified waypoint deployment (2026-10-10)

On explicit deployment request, fresh disarmed/ESC-off/no-mission state was
verified before restarting core. Jetson-only sonar commits through 3364bd6 were
preserved by merge b86add1 with navigation update 6ef0647; combined branch pushed
and Windows checkout synchronized. Colcon build and all 130 isolated mocked
regressions passed on Jetson. /waypoint HTTP 200 and live Cube position/heading
API verified, with one instance each of bridge/supervisor/VN/DVL/GUI. MAVProxy and
sonar launcher stayed active. No vehicle mission, arming or Cube parameter writes.
Live waypoint execution remains disabled; screenshot registration approximate.
Supervisor observed Depth reference unavailable, Observation, source Unknown,
configuration unverified. Restart reset control selection to RF/STARTUP inhibit;
vehicle remained disarmed with ESC off and SA kill active. Bridge frame continuity
must be revalidated after restart. See docs/deployment-2026-10-10.md for evidence;
these are dated observations, not permanent live status.

2026-10-10 follow-up: user-confirmed water-surface calibration set the current
observation supervisor session baseline to997.4154663085938hPa (57 samples).
Verified Surfaced, depthabout0.0015m. Only observation supervisor reloaded;
bridge/control processes unchanged. No source switching performed. Baseline is
session-only and must be re-established after core restart at confirmed surface.
See deployment-2026-10-10.md for remaining commissioning blockers and evidence.

2026-10-10 commissioning follow-up: Cube surface source profile now matches code
(SRC2 POSXY3 VELXY6 YAW6, SRC_OPTIONS0); SCR_ENABLE1 and telemetry-only Lua source
reporter installed/readback verified. All persisted after disarmed Cube reboot;
75 NAV_SRC reports/15s confirmed Underwater selected. No transition test completed:
guarded test aborted before command when vehicle was armed again. Supervisor stays
Observation, commissioning validation incomplete. Detailed evidence/failures in
docs/deployment-2026-10-10.md. Calibration restored after core restarted on reboot.

2026-10-10 later: disarmed Surface-source command test succeeded (ACK0,50 actual
NAV_SRC=2 reports/10s); GUI confirmed Surface GPS. Estimated local position moved
up to2.4341m relative to pre-command value; continuity/accuracy not commissioned.
Supervisor remains Observation; no automatic round-trip test or mission enabled.

2026-10-10: operator-requested VN yaw_offset_deg180 deployed and readback verified;
flip_180true unchanged. Core restarted disarmed; prior depth baseline restored.
Bridge frame reset, mission continuity unverified. VN/Cube still differedabout20deg
on final sample; do not equate applied offset with commissioned heading accuracy.

2026-10-10 subsequent check: restarted VN/DVL drivers while disarmed (bridge and
Cube kept running).45 fresh paired samples: VN/Cubeabout196.3deg; median wrapped
absolute error0.0251deg. Retained yaw180. Previous20deg disagreement cleared;
no independent physical heading validation and no causal claim restart vs settling.

2026-10-10 latest: user requested net VN yaw offset90deg CCW; deployed/readback
verified. Surface pressure baseline restored. After reload VN~15.6deg, Cube~358deg;
20 paired samples median error15.87deg. Heading still uncommissioned. Details in
waypoint-mission-2026-10-10.md.

## 2026-10-11 continuous-EKF redesign checkpoint
User requested replacement of source switching with Jetson-qualified GPS feeding
one continuous Cube fusion configuration, with accurate receiver timing. Authorized
online tether_toggle_beichen development, not deployment. Investigation only:
upstream ArduPilot GPS_RAW_INT carries Cube last-fix time; GPS_INPUT backend uses
GPS week/time-of-week for jitter correction. Exact vehicle firmware/time transport
not verified. Do not claim receiver synchronization or implement it by retimestamping
old fixes. Runtime replacement paused at this uncertainty per user's instruction;
existing source-switching code is unchanged. 134 baseline regressions passed.
See docs/single-ekf-redesign-2026-10-11.md for pipeline, migration, and saved questions.
No vehicle access or parameter changes. This is a proposal, not deployed behavior.

2026-10-11 follow-up: user accepted timing approximation. Experimental GPS gate
and continuous supervisor implemented on tether_toggle_beichen, opt-in only;
GPS forwarding disabled by default, legacy observation default retained. Matched
TIMESYNC estimates Cube-fix transport age; receiver delay remains unknown. Fixed
GPS history expiry allowing prolonged stationary jumps to requalify. 145 Python
regressions and browser test pass. Native isolated SITL admission/recovery passes,
but GPS loss produces EKF flags39 (no horizontal position validity), including a
confirmed-armed simulator run. Full underwater replacement is NOT complete; no
health-check bypass, vehicle deployment, parameter changes or main changes.
See docs/single-ekf-redesign-2026-10-11.md and its committed JSON evidence.

2026-10-11 subsequent user revision: external navigation sensors now enter an
experimental fused_nav_bridge. Pressure gates GPS; admitted GPS corrects bridge
position and conditional forward-motion heading/speed, then one ODOMETRY stream
feeds external-only Cube source settings. Virtual GPS prototype removed. Supervisor
retained as mission/reference guard, using authoritative bridge sensor assessment;
no continuous-mode source selection. Opt-in and output disabled by default.
156 regressions, ROS build and GUI rendering pass. Isolated native stationary
fixture passes GPS jump/outage/recovery with horizontal validity retained. Failed
origin-startup runs retained; altitude-only origin metadata is independent of
pressure depth, horizontal origin changes still fault. No vehicle deployment.
See docs/bridge-odometry-2026-10-11.md for conditions, evidence and outstanding
full ROS/native integration, moving mission and commissioning.

2026-10-11 deployment: authorized tether SSH at192.168.2.2. Navigation code through
121a25a installed/built/tested on Jetson, preserving thirteen local sonar commits
(final observed merge e04b831). New bridge running observation-only, output disabled;
no Cube parameter writes. Confirmed-surface baseline1013.745361328125hPa gives
Surfaced/depthabout-0.006m. DVL validity false, old Cube profile and unvalidated
heading block waypoint readiness. Scoped temporary runtime override expires at
reboot. Controller ended disarmed MANUAL/RF, SA active, ESC off. Online branch and
Jetson include the same navigation changes but Jetson-only sonar history remains.
See docs/deployment-2026-10-11.md; this is dated evidence, not future live status.

2026-10-11 Wi-Fi follow-up: user requested removal of yaw offset and Pixhawk
navigation preparation. Connected at192.168.8.105; verified kill active, disarmed,
ESC off before restart/writes. Deployed cc7772c yaw_offset_deg0; separate roll
mounting transform unchanged. Backed up and applied odometry_profile source values;
fresh readback all match. Initial script readback failed on queued old replies;
draining then rereading verified with no further writes. Backup/result JSONs are
Jetson /home/graey/robotx_ws/logs/external-profile-*. New confirmed-surface baseline
1011.2716064453125hPa. Final bridge healthy, valid DVL/VN, GPS qualified, Surfaced,
depthabout-0.005m; VN heading86.8deg not physically validated. Output disabled,
mission observation-only, controller RF/MANUAL/disarmed/SA active/ESC off. Runtime
override temporary until reboot. Cube fusion/moving waypoint validation remains;
this is preparation, not waypoint readiness. No full Pixhawk reset or arming.

## October 12 teammate context and waypoint audit
User supplied Ruth's sonar reports dated Oct4, Oct10 and Oct11. Originals copied
under docs/teammate-context; Oct11 explicitly supersedes misleading Oct10 pipe-band
and ESC-noise conclusions. These are attributed teammate observations; recordings
and camera attachments were not provided or independently inspected. Sonar map
uses its own VN/DVL dead-reckoned Start frame, not Cube global navigation. Latest
reported2b30056 sonar history was preserved in Oct11 deployment, not published as
part of nav work. See docs/session-after-source-switching-2026-10-08.md for the
consolidated history and correction provenance. Current live status not inferred.

2026-10-12 local GUI audit:159 regressions pass; continuous production ROS bridge,
supervisor, native ArduSub and GUI API now complete nominal dive/transit/surface
missions with underwater GPS outage/recovery; operator Abort confirms MANUAL.
Final deterministic fixture endpoint errorabout0.111m, not physical accuracy.
Fixed TIMESYNC reply polling50ms vs50ms RTT gate (now10ms receive,20Hz output),
bounded MAVLink drain, preserved callback error exits, monotonic independent-truth
SITL fixtures. GUI map/status/zoom/track improved and browser inspected. No live
deployment or mission enablement. Approximate pool-image live targets remain
blocked; commissioned geographic route tested. See docs/waypoint-gui-audit-2026-10-12.md
and committed failed/successful validation JSONs. Prior stationary-only limits
are superseded for local runtime reachability, not live commissioning.

2026-10-12 second architecture review: fixed receipt-only GPS qualification expiry
(now checks current Cube-fix age and requalifies), waypoint pilot-mode takeover
handling in ARM/DISARM, altitude-only origin metadata false aborts, and nonfinite
pressure crashing bridge diagnostic JSON. 165 regressions and both GUI script
checks pass; continuous GUI/API native SITL nominal and Abort runs pass. No live
deployment. Identified remaining mission-process-death gap: kill watchdog treats
GUIDED as autonomous; expiring mission Bool does not itself guarantee MANUAL.
Independent run-scoped mission-liveness policy needs design and fault injection
before unattended operation. See docs/architecture-review-2026-10-12.md. This is
not a declaration of live readiness; existing untracked nav_lgoic.md preserved.

2026-10-13 authorized Wi-Fi land deployment: Jetson merged navigation through
6843f9d preserving local sonar history (merge d0ca60c); build and165 tests passed.
Persistent /etc systemd override now launches continuous observation with depth
baseline0, output disabled and supervisor observation. Fresh passive preflight
confirmed Cube profile matches, VN fresh, disarmed MANUAL/SA active/ESC off.
DVL invalid and depth/GPS/heading readiness unconfirmed on land. User requested
waiting until floating with bottom lock. No parameter writes, output enablement,
arming or mission. See docs/deployment-2026-10-13.md; current physical status must
be checked again, and mission-process-death safeguard remains outstanding.

2026-10-13 continuous-only deployment: navigation commit81fd138 merged on Jetson
as35eaec8, preserving sonar history.117 current regressions passed locally and
on Jetson; native GUI/API nominal and Abort simulations passed with underwater
GPS outage. Legacy switching runtime and executable registration removed; stale
installed launcher archived outside executable directory. User confirmed floating,
killed/disarmed. Session pressure baseline997.84228515625hPa applied. Initial DVL
continuity fault required controlled core restart; subsequent passive check passed
bridge health, DVL/VN, surface depthabout0.032m, GPS admission and Cube profile.
Output and supervisor remain disabled. Physical heading reported68deg; observed
VN263deg and Cube12deg disagree. VN yaw offset readback0.0, roll flipTrue. Heading
NOT commissioned; no arbitrary correction, arming or mission performed. Persistent
startup stays continuous observation; surface baseline is session-only. See
 docs/continuous-only-2026-10-13.md. These are dated observations, not live status.

October 13 rotation check: user ready at requested approximately124deg after
stationary34deg reference. Fresh VN334.090deg and Cube98.422deg; previous
VN261.746deg/Cube22.106deg. Observed clockwise changes72.344deg/76.316deg,
versus requested approximately90deg. Both respond in the same direction, but
absolute heading remains unvalidated. Approximate physical endpoints do not
establish sensor scale or justify a fixed yaw offset. No parameter writes.

2026-10-14 offline VN workflow: user reports teammate restored relative mode for
magnetic interference and selected GPS/DVL surface movement for north alignment.
Production VN driver now verifies relative mode at connection without saving flash.
New guided tools/vn_calibrate.py acquires sole-reader lock, backs up registers,
collects slow3D HSI/six-face coverage and stops learning on interruption; volatile
only, hardware bench validation pending. Every new core bridge session now needs
GPS/DVL velocity-vector north alignment before position initialization/odometry.
No stationary GPS yaw assumption; timing/pressure/GPS/DVL/tilt/rate/speed/dwell gates
apply. GUI shows alignment status. New checked session commissioning service enables
supervisor after healthy Cube fusion/aligned odometry and fresh kill/disarm/ESC-off,
without restarting bridge. Launch flags default output and mission off.130 regression
tests, browser checks, nominal and Abort native GUI/SITL runs pass; injected40deg
relative-reference error recovered as-40deg. These are simulated results, not live
accuracy. Graey offline; SSH timed out. No deployment, live sensor writes, odometry
activation, arming or vehicle mission. See docs/vn-pretest-workflow-2026-10-14.md.
Earlier relative-mode diagnosis was overstated: relative mode alone does not prove
arbitrary yaw/no north reference. Compass disabled/external yaw meant Cube yaw was
not an independent compass reference in October13 comparisons. North-reference,
magnetic calibration, mounting/body axes and timing require separate validation.

2026-10-14 GUI follow-up: user explicitly prefers VN-100 as primary external
attitude source over Cube-derived attitude. This is a design priority, not proof
that VN is accurate under interference; no covariance weighting was tightened.
Added /calibration page and main/Waypoint links. GUI jobs suspend mission permission
and odometry, run HSI calibration in the existing VN reader (no competing serial
owner), show120s/six-face progress and stop/fail on safety/link/readback problems.
Prepare requires explicit floating-surface confirmation and stable fresh pressure,
starts new relative-VN/GPS/DVL north alignment, and waits for killed/disarmed Cube
fusion verification before checked session mission activation. No arm/mode/ESC
commands. Bridge reset counter increments on controlled preparation; Cube origin
stays continuous. Hardware kill/watchdog faults are not cleared by preparation.
Reproduced/fixed supervisor crash on None VN/DVL received timestamp; regression
added. New GUI physical calibration/positive commissioning paths await hardware;
Graey remains offline and changes are not deployed. See docs/calibration-gui-2026-10-14.md.

2026-10-14 tether deployment: navigation through9faf100 merged into Jetson sonar
history as45b9525; ROS build and142 tests passed on Jetson. Calibration page/API
HTTP200 and six new maintenance services verified. One fused bridge/supervisor,
VN relative mode, fresh valid DVL, Cube bridge profile verified. Core remains
continuous Observation, output disabled, baseline0; no physical calibration,
preparation, arming or mission. Restart controller selected RF; killed/disarmed
MANUAL/ESC-off confirmed. Sonar relay and untracked work preserved. Jetson sonar
history remains additional to online navigation branch, not fully Git-identical.
See docs/calibration-deployment-2026-10-14.md. These are dated observations.

2026-10-14 repeated Prepare failure resolved: supervisor latched the intentional
bridge maintenance sentinel. d664f93 skips only that sentinel when supervisor
inactive AND bridge output disabled; real/active faults remain latched. Source
merge/restart alone did not update the installed Python copy on this Jetson.
Rebuilt package, verified installed supervisor maintenance_pause and GUI
prepared_at code using inspect from /tmp, then restarted supervisor/GUI only.
147 tests passed on Jetson. Live API Prepare remained busy/preparing after5s,
GPS qualifying, supervisor no longer Fault. Bridge instance and collected VN
calibration preserved. No arming/movement/mission; manual north alignment pending.
Future deployments must rebuild and verify installed imports, not assume symlinks.

2026-10-14 GPS qualification follow-up: d161c6f deployed on Jetson as051877b,
150 tests passed there. Fix withholds expired measurements but retains good dwell
within the unchanged receipt-continuity limit; actual quality failures still reset.
Added progress and last rejection diagnostics. Full observation restart reset
baseline/north alignment; VN reader reconnects relative without saving flash or
resetting learned HSI registers, but in-memory six-face coverage returns idle.
Fresh passive read on continuation: killed/disarmed MANUAL/ESC-off, output off,
baseline0, GPS position accuracy1.736m, velocity uncertainty0.433m/s. Automatic
north alignment requires<=0.20m/s, so this receiver reading cannot pass; do not
keep driving to chase the speed prompt. Reordered alignment checks to show poor
velocity precision before asking for motion; thresholds unchanged.151 tests pass
locally, including delayed5Hz packet/20Hz status alignment and rejection priority.
Live moving alignment and physical mission remain unverified.

2026-10-14 time association follow-up: live last rejection was GPS cannot be
aligned to DVL/VN history, not DVL-reported uncertainty. Passive ROS topic rate
about9Hz (intervals85–148ms). A newly received GPS fix can be newer than the last
DVL history sample beyond the100ms extrapolation limit. Such a fix formerly
invalidated all GPS qualification. Added bounded deferred association: wait at
most150ms for a DVL sample, retain original GPS timestamp, recheck measurement
age<=350ms plus pressure/VN/fault gates, and never apply expired fixes. Missing
history no longer erases receiver-quality dwell. New rejected fixes clear pending
work. Simulated low-rate DVL/new GPS ordering completes alignment; queued expired
fix cannot initialize position.153 tests passed before final queue invalidation
line; full suite rerun during deployment. GPS velocity uncertainty~0.488m/s remains
an independent blocker; no accuracy/angular/freshness threshold loosened.

2026-10-14 added stationary known true-heading method to Calibration Prepare,
with explicit measured-heading confirmation, independent bridge kill/disarm,
surface/stationary checks and session-only provenance. Does not relax GPS position
or mission/Cube health gates. GPS velocity precision is not needed for operator
north reference.161 regressions and Node page check pass; physical capture requires
user's actual true bearing. See docs/calibration-deployment-2026-10-14.md.

2026-10-14 known-heading feature deployed over tether: navigation through236815a
merged on Jetson asa2a7ed7, package rebuilt,161 tests passed there. Installed
service/client imports and served Calibration HTML verified. Live unconfirmed
known-heading request rejected409 with no job/reference change. Fresh controller
killed/disarmed MANUAL/ESC-off. Core observation output off, no heading captured;
operator must supply actual independent true bearing. Backup at
/home/graey/deployment-backups/20261014-known-heading-234523. Main untouched.

2026-10-14 explicit recovery verified: fixed Prepare's supervisor old-fault race
in afb9b1f, rebuilt/deployed GUI only (bridge/sensors preserved),163 tests passed
on Jetson. Recovery using retained true frame completed Ready; bridge fault empty,
healthy/transmitting, reset_counter3. Cube LOCAL_POSITION_NED fresh age0.171s,
EKF flags831 accepted, Cube estimate valid and heading288.4deg at that sample.
Killed/disarmed/ESC-off throughout; no Cube reboot or source/firmware parameter
write. First stationary check aborted without mutation; subsequent recovery attempt
failed old supervisor latch before the fix. Original known20deg reference had
rotated with VN; recovered286.318deg is DERIVED from retained reference, not newly
measured true north. Preserve this provenance despite generic GUI method label.
Automatic DVL dropout fault recovery is NOT added: explicit Prepare now recovers
Cube without core restart. Underwater missing velocity cannot be silently repaired.

2026-10-14 DVL recurrence diagnosed:224 published samples/25s included2 invalid,
no>=0.5s gaps or regressing timestamps. Added bounded reacquisition: withhold
invalid odometry; resume after2 consistent valid velocities before0.5s deadline,
increase uncertainty for unmeasured gap; long loss and other hard faults latch.
167 tests, native nominal waypoint and native short/long DVL fault injection pass.
Mission may still abort on real unhealthy data; no silent run restart. See
 docs/calibration-deployment-2026-10-14.md and committed evidence.

2026-10-14 bounded DVL recovery deployed through ebdcf9c on Jetson as9f100a3.
ROS build and167 regressions passed on Jetson. Fresh API exposes dvl_continuity,
VN/DVL inputs fresh, no bridge fault; killed/disarmed ESC-off, RF selected.
Restart intentionally baseline0/output disabled; known-heading Prepare required.
Backup /home/graey/deployment-backups/20261014-bounded-dvl-001311. Native simulation
validated recovery; live postinitialization dropout recovery still awaits wet test.
No real mission started, no main changes; sonar history/untracked files preserved.

2026-10-14 waypoint interaction simplified: removed user-facing Preview and30s
selection expiry. Map click retains selected coordinates/depth without POST.
Start performs fresh server target validation then consumes the returned single-use
plan; network-ambiguous retries retain the same plan/run identity. Definitively
rejected validation clears cached token for next explicit Start. Existing backend
frame/age/readiness/map registration checks remain; no new arm/mode permissions.
Page tests cover selection surviving60s, no command on map click, validation failure
not starting, and timeout retry using same plan.167 Python regressions pass; updated
Node page check passes. Static page deploy needs no GUI/core restart or re-Prepare.

2026-10-14 Start/displacement investigation: user confirmed Pool image mode.
Live Prepare Ready, bridge healthy/transmitting, Cube local/global fresh, mission
capability enabled, no active waypoint. Therefore live pool-image restriction and
selected target are the remaining page blockers, not current supervisor failure.
Pool screenshot registration is approximate; no registration block bypassed.
Added explicit view-only dropdown/watermark, disabled-button reason and Use live
navigation grid button that switches mode and clears old image-derived target;
operator must select a new target on the grid. Node tests cover this transition.
GUI Cube marker reads GLOBAL_POSITION_INT, matching QGC Vehicle::_handleGlobalPositionInt
in official https://github.com/mavlink/qgroundcontrol/blob/master/src/Vehicle/Vehicle.cc.
Actual user's QGC feed/version was not inspected. Observed Cube global vs local NED
about[-0.323,-1.055]m vs[-0.324,-1.057]m using Cube origin at one snapshot, consistent
within centimetres. Pool image displacement mismatch is suspected map registration/
scale, not a demonstrated EKF scale defect; physical/QGC movement comparison remains.

2026-10-14 added georeferenced online satellite background for live pool waypoint
selection. Esri World_Imagery tiles use EPSG3857; clicked points become geographic
plans, separate from old approximate screenshot view. Shows Cube/GPS/track/route,
loading/errors and provider attribution; offline grid retained. No fusion or live
mission permission gates bypassed. Browser internet required. Node tests cover
projection inversion, tile readiness and geographic target submission. See sessiondoc.

2026-10-15 local waypoint/map follow-up (not deployed): user requested finish
locally and deploy later. Satellite page now defaults to a visually inspected,
clearer Esri Wayback release 2023-08-31, with current-layer choice, Find pool and
Reload imagery. Geographic clicks retain live target validation; old screenshot
remains view-only. Find pool changes viewport only. Image date/position and GPS
accuracy are not surveyed guarantees. Full-page browser localhost preview timed
out; direct tile visual inspection and page JavaScript checks passed.
Fixed demonstrated stale pilot arm/disarm retries fighting mission GUIDED under
RF or TETHER: cancel retries on new mission claim; static SB level cannot force
mode/arm/disarm during ownership. New SB disarm edge and SA kill remain effective.
Added pilot-control evidence to mission stage logs. No firmware failsafe bypass.
173 Python regressions and Node page checks pass; native GUI-API continuous SITL
completed 0.5m dive/transit/surface/disarm/GPS recovery, simulated target error
0.153m, QGC forwarding disabled. Native fixture excludes hardware kill node;
production pilot callbacks tested with mocked transport. Live cause confirmation
and deployment pending. See docs/waypoint-map-control-session-2026-10-15.md.

2026-10-15 added collapsible Cube estimate stats on /waypoint, open by default.
Displays Cube geographic/local position, attitude, velocity and speed, calibrated
pressure depth, EKF reported metrics/flags and stream ages. Also shows validated
corrected VN bridge heading, north offset/GPS heading adjustment, wrapped Cube/VN
heading difference and raw GPS separation. Withholds stale/missing derived values;
marks whether bridge odometry is transmitting. Uses existing telemetry only.
Node page regression passes, including heading wrap across north and stale-data
withholding. Static UI change; local/online toggle update, Jetson deployment pending.

2026-10-15 deployment verified over tether: merged current navigation/UI through
44731b2 with Jetson sonar history17397c8 as2029343. Backup20261015-waypoint-223525.
Build and173 regressions passed on Jetson. Stopped hand-started August bridge;
one normal continuous bridge/supervisor/pilot node observed. Restored sole profile
mismatch EK3_SRC1_POSZ from1 to6; complete profile readback verified and running
bridge reports configuration verified. Bar30 pressure and VN/DVL inputs fresh.
Refreshed stale installed waypoint HTML; served map controls/Cube stats verified.
Killed/disarmed ESC-off confirmed; restart selected RF (previously TETHER_LOST).
Operator will Calibrate/Prepare personally; baseline0 and output withheld at final
snapshot. No physical mission or post-calibration accuracy measurement yet. See
 docs/waypoint-deployment-2026-10-15.md. Local clone includes preserved sonar history.

2026-10-15 satellite blank desktop reproduced and fixed: archive level21 requests
failed at1440x900 while level20 rendered. Cap archived tile requests at20 and
upscale; current provider retains21. Node desktop regression and live screenshot
pass. Static HTML deployed with controller/bridge/Prepare identities preserved.
Operator had calibrated/prepared (known true heading102 captured). Later live
DVL continuity fault latched with4 dropouts; DVL subsequently fresh/valid at0.729m
clearance, but supervisor Fault/navigation_ready=false. Old Prepare job ready
label is historical; do not treat it as current mission permission. No mission.
Single earlier snapshot Cube172.2/corrected VN171.6, GPS separation0.11m is only
consistency evidence, not absolute accuracy. See deployment session document.

2026-10-15 implemented explicit surfaced DVL position recovery. New calibration
page action retains bridge-session north alignment/heading bias and pressure
baseline, requires killed/disarmed ESC-off plus stationary bottom lock and VN
continuity, discards uncertain position and waits for new qualified GPS. Generic
Prepare still resets alignment; recovery avoids magnetic recalibration and north
recapture for DVL-only faults. No underwater recovery or automatic old mission
resume. 179 Python regressions and both Node page suites pass; isolated native
SITL explicit ROS recovery resumes GPS-anchored odometry with reset counter+1.
See docs/dvl-position-recovery-2026-10-15.md. Installation restart requires one new
Prepare; routine recovery thereafter does not restart sensors or bridge.

2026-10-15 DVL recovery deployed through140a9c8. Backup20261015-dvl-recovery-230552.
Caught stale installed Python despite successful build; backed up/refreshed3
modules, verified actual Trigger recovery service/status field, and179 tests pass
against installed runtime from/tmp. Page recovery controls visually verified.
Killed/disarmed ESC-off, RF selected. Restart leaves baseline0/north unaligned;
operator needs one Prepare now. Magnetic calibration was not run/erased. No
physical recovery or mission started. Routine DVL-only recovery then retains
alignment. See docs/dvl-position-recovery-2026-10-15.md for provenance/limits.

2026-10-15 heading graphic clarified at operator request: dot/line replaced by
explicit Cube bow arrow, exact Cube angle label and north/east legend. Cardinal
geometry and305deg NW checks pass. Deployed static HTML through89f1e99 without
restarting navigation; controller/bridge/job identities preserved. Live screenshot
shows299.7deg arrow pointing NW. No sensor yaw offset or DVL tuning applied.
12 live samples saved Jetson logs/map-consistency-20261015.json; Cube/bridge NE
separation roughly0.05–0.08m, GPS separation roughly0.07–0.11m in sampled endpoints.
These measure consistency only; no independent physical heading/displacement
reference supplied. Operator says DVL error slight, prioritize graphic heading.
Also deployed recovery disabled-reason textb70df32: initial post-restart Prepare
required when no baseline/north exists; no fault bypass introduced.

2026-10-15 implemented DVL stationary position hold: after initialization, two
seconds of fresh valid bottom lock <=0.03m/s and low VN rotation holds bridge NE
and zero velocity, withholding GPS translation/bias corrections. Movement, invalid
DVL or stale VN/pressure releases hold. Raw GPS stays visible/quality checked.
No Cube origin reset, direct GPS fusion or heading tuning.182 regressions and
page check passed; native mission/deployment results in stationary-position-hold
session document. Very slow actual movement below3cm/s may be treated as rest.
