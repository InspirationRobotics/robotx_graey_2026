# Graey dry-land baseline — October 9, 2026 (Pacific)

Read-only inspection authorized by the user. SSH connected to 192.168.2.2 through
Ethernet 5, laptop source 192.168.2.1. Graey also has Wi-Fi 192.168.8.105.
No parameters, source sets, origins, code, services, power or control mode changed.

Controller confirmed disarmed, MANUAL, SA kill active, ESC commanded off, no mission.
Both links healthy. Control selection remained RF; using tether SSH does not change
control selection. End-of-inspection check confirmed the same safe/idle state.

## Observations

- Host checkout: tether_toggle_beichen, HEAD 1e09b328576dfb8c5341fe10daab1c23e3e92ade.
  Git reported ahead 9 of its cached origin tracking ref; no fetch was done, so that
  is not a current GitHub synchronization comparison. Untracked work was preserved.
- Exactly one listed executable each for DVL, VN-100, bridge and supervisor.
  MAVProxy is the Cube serial owner. Supervisor launch has active=false.
- Deployed source lacks recent local sensor cleanup/ownership fixes. Startup log
  shows LED telemetry and supervisor both identifying as MAVLink component 196.
  The exact installed Python module/source hash equivalence was not established.
- VN heading stale ~35 minutes initially, ~38 minutes at end of capture. DVL stale
  ~25 minutes initially, ~28 minutes at end. Both processes alive. VN serial device
  exists and process has ttyUSB2 open. No competing serial reader was started.
  Process I/O counters unavailable; this inspection cannot prove whether bytes
  arrive, the parser rejects them, or ROS publishing stalls. Sensor power remains
  unconfirmed. Dry land prevents a valid underwater bottom-lock assessment.
- GPS fresh, fix type 4, 25–26 satellites. Stationary capture: 60 samples over
  62.8 seconds, all disarmed. North range 0.423 m, east range 1.215 m; receiver
  horizontal accuracy 1.442–1.481 m during capture. These measure short-term scatter
  and receiver estimates, not absolute accuracy against a surveyed location.
- Cube position marked invalid; no LOCAL_POSITION_NED in the 20-second MAVLink
  observation. EKF flags 167. Cube yaw approximately 154.6 degrees; stale VN heading
  339.07 degrees must NOT be compared as a simultaneous heading measurement.
- Live source1 horizontal position/velocity/yaw configured ExternalNav (6).
  Source2 corresponding values all None (0). SCR_ENABLE=0; no NAV_SRC observed.
  Therefore the intended surface GPS source profile/reporting is not commissioned.
  This readback is configuration evidence, not proof of the active source or fusion.
- LOG_DISARMED=1. Complete DataFlash download was not performed.

## Evidence and next step

`stationary.jsonl` contains timestamped controller and stream snapshots;
`summary.json` has the capture summary. `parameters-readback.json` records selected
live parameters, not a full backup. Diagnostic scripts are alongside these files.

Restore/verify VN-100 and DVL telemetry first; do not interpret stale values as
live orientation or attempt numeric GPS tuning from this capture. Next inspect
sensor power/cabling and reader/publisher behavior while keeping thruster kill
engaged. Once fresh, compare headings on land; use water for valid DVL motion and
depth tests. Commission source reporting and the reviewed surface/underwater
profile only after sensor/frame validation and explicit deployment authorization.

Enum reference: https://github.com/ArduPilot/ardupilot_wiki/blob/master/common/source/docs/common-ekf-sources.rst

## VN recovery and manual heading checks (same evening)

User reported VN recovered; subsequently verified fresh heading updates. Vehicle
remained disarmed, ESC commanded off. Control selection now TETHER with
SOURCE_CHANGED inhibit. Supervisor now reports a latched bridge-restart fault;
this inspection did not restart the bridge or clear the fault.

| User orientation | VN heading | Cube yaw |
| --- | ---: | ---: |
| Before turns | 337.11 | 154.49 |
| Approximately 90 degrees clockwise | 63.09 | 240.29 |
| Another approximately 90 degrees clockwise | 154.32 | 330.21 |

First increments: +85.98/+85.80 degrees; second: +91.24/+89.92 degrees.
User judged the Cube's 240-degree southwest direction consistent with physical
orientation. This is a user observation, not a surveyed true-north reference.
Persistent near-180-degree disagreement is confirmed; exact mounting/magnetic
cause remains unproven. Do not fit a numeric correction from the Cube alone.

Read live ROS parameters and imported installed modules without constructing
sensor nodes or opening serial ports:

- VN `flip_180=true`, `yaw_offset_deg=0`, baud115200 on the expected FTDI port.
- Bridge `yaw_offset_deg=0`, `allow_alignment=false`.
- Driver flips roll/pitch and Y/Z vector signs, but heading is simply raw yaw plus
  yaw_offset_deg. The upside-down flag does not add a 180-degree heading correction.
- Bridge applies a world-Z quaternion yaw offset, currently zero; thus it does
  not secretly correct the heading difference. ROS IMU quaternion yaw154.375
  agrees with the VN heading after the second turn.
- Bridge status healthy=false, aligned=false; installed send_odom withholds all
  odometry when freshness/validity fails, including external yaw. Current Cube
  yaw cannot be treated as a verified fused VN yaw in this condition.
- Installed VN module SHA256:
  b0728413aa7206394cc5f867e41484f5c5b8f02fec989cf3381b1ff847f8c413
- Installed bridge module SHA256:
  4f65e88b55a1c85084eb99381ccb1ee92306270578feb078c4be3ba15922f0e3b

Next: identify the VN physical sensor axes relative to vehicle forward/right/down.
Any mounting correction must handle orientation and body vectors consistently,
not just rotate the GUI heading. No offsets or source parameters were changed.

## Temporary offset attempt blocked before mutation

User authorized temporary +180 offset and VN-only restart. The diagnostic script
`temporary_vn_offset.py` aborted at its single-owner precondition BEFORE modifying
any source, creating a backup, or signalling a process. A subsequent process tree
confirmed actual duplicate executables (not merely ros2-run wrapper/child pairs):

| Node | GUI-owned executable | Core-launch-owned executable |
| --- | ---: | ---: |
| VN-100 | 5854 (parent5847) | 5879 (parent27) |
| DVL | 5853 (parent5846) | 5877 (parent27) |
| Navigation bridge | 5852 (parent5848) | 5873 (parent27) |

GUI pid88 owns ros2-run wrappers5846/5847/5848. Core launch pid27 owns the second
set. This is a changed live state from the earlier one-instance inventory. It
explains an ownership problem but does not prove the ~180-degree heading cause.
No temporary offset was applied. Request approval for removal of GUI-owned
duplicates while retaining core-owned nodes before retrying a VN-only restart.

## Authorized cleanup and temporary offset applied

User approved stopping GUI-owned duplicates, retaining core owners, then applying
the temporary offset. Sent SIGINT only to duplicate executables5854/5853/5852.
Their ros2 wrappers became inactive zombies awaiting GUI reaping; the cleanup
helper initially asserted on this changed state. A fresh process tree verified
that no duplicate executable remained. Helper updated to recognize zombie wrappers.
Core DVL5877 and bridge5873 retained; no core bridge restart or position reset.

Applied +180 degrees via VN's startup default, restarted only core VN5879, and
verified new VN14916 reports yaw_offset_deg180 via ROS parameter service. Original
installed source restored byte-for-byte after successful readback; SHA256 matches
the prior source. Backup: /tmp/graey-vn-offset-90j9prbh/vn100_node.py.original.
Offset lives only in the current process and reverts to the original0 on its next
restart; no persistent repository or startup-default change. This is a temporary
world-heading correction, not verified physical body-frame calibration.

Fresh post-change VN334.426 degrees versus Cube330.058 degrees, difference4.368.
Vehicle disarmed, MANUAL, SA active, ESC commanded off, no mission. Supervisor
remains Observation/Fault for the previous bridge restart; readiness false.
Do not clear that fault or commission navigation solely because headings agree.
Avoid the deployed GUI's sensor start/stop controls, which can recreate duplicates.

Further user-performed approximately90deg clockwise turn after applying offset:
fresh VN62.496deg, Cube58.008deg, residual4.488deg. Changes from prior readings:
VN+88.070deg, Cube+87.950deg. Residual stayed approximately4.4–4.5deg across this
turn, consistent with retained temporary correction. Still disarmed/ESCoff,
supervisor Observation/Fault. This comparison does not establish absolute heading
accuracy or validate a permanent sensor mounting transform.
