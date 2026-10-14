# Navigation work after the October 8 source-switching session

Compiled October 12 from committed session reports, test evidence and this chat.
This is a dated history, not a claim about the current running vehicle.

## October 9: hardware and navigation observations

Dry-land and pool work compared VN attitude, DVL motion and Cube estimates.
Source-switching confirmation work distinguished command ACK from actual source
feedback and withheld readiness on deadlines/faults. GUI/core duplicate navigation
processes were investigated and ownership protections added. Temporary heading
offsets were tested; agreement between VN and Cube did not independently establish
true heading. Manual pool movement provided observations, not a commissioned
autonomous waypoint mission. See pool-session-2026-10-09.md and dry-land evidence.

Stationary GPS capture:60 samples/62.8s, scatter0.423m north/1.215m east and
reported hacc1.442–1.481m; repeatability is not surveyed accuracy. Two manual
captures integrated about1.15m/1.53m travel. In the repeat, DVL net1.29m and
Cube net1.30m agreed while raw GPS shifted approximately0.1m north/1.936m east.
These GUI-polled measurements were not atomic sensor logs and did not justify
scale tuning. Temporary duplicate cleanup retained the core navigation owners.

After the October8 checkpoint, final ACK/source confirmation hardening preserved
timely evidence, rejected late/unsolicited/foreign ACKs, checked both-direction
position continuity and logged frozen transaction outcomes. Drop-ACK/drop-source
injections withheld readiness. Intermittent scheduling delays remained documented;
successful repeated SITL runs did not erase failed evidence. The resumption
automation was removed after validation, per the source-switching session report.

## October 10: deployment and waypoint preparation

Deployed tether-branch navigation/GUI work while preserving local sonar changes.
Fixed runtime offset parameters that accepted changes without updating cached
transforms by making mounting settings startup-only. Corrected supervisor gyro
conversion: VN VNYBA angular rates are radians/second. Added GUI/API waypoint
preview/start/abort flow, bounded mission stages, persisted surface start and
Cube position/heading display. Mission begins surfaced, captures a reference,
dives, transits, surfaces and waits for GPS recovery.

Depth reporting/calibration and telemetry freshness problems were investigated.
Compact supervisor polling replaced full-map polling and exposed connection/HTTP
errors. A976-parameter Cube backup preceded surface profile configuration. Waypoint
preparation was revised to accept an already-armed MANUAL/STABILIZE start while
aborting on pilot disarm instead of automatically rearming.
Verified selected source profile and Lua feedback after a disarmed Cube reboot.
A disarmed surface-source command succeeded, but Cube position changed by up to
2.4341 m; source selection was not proof of accuracy. VN yaw180 was applied,
then driver restart produced close VN/Cube agreement; later operator-requested
90-degree CCW net correction remained uncommissioned. See deployment-2026-10-10.md,
session-2026-10-10.md and waypoint-mission-2026-10-10.md for precise evidence.

## October 11: continuous external navigation redesign

User replaced surface/underwater source switching with one continuous fusion
configuration. Initial virtual-GPS prototype qualified fixes using Cube timestamps
and TIMESYNC. Receiver measurement epoch/delay were unavailable; user accepted
the explicit timing approximation. Native SITL showed loss of horizontal position
validity on GPS outage, including an armed simulator run. The prototype was not
declared successful and its failed results were retained.

User then required external sensor aiding through one bridge. Implemented
fused_nav_bridge with DVL/VN propagation, pressure-qualified GPS admission,
bounded GPS position corrections, conditional course/forward-speed correction,
and continuous ODOMETRY. All source slots use identical external-navigation
position/velocity/yaw settings. Cube retains its internal inertial fusion.
Supervisor became a mission/reference guard using bridge-owned depth/GPS
assessment rather than independently qualifying those inputs.

Fixed Cube-fix timestamp compatibility in mission reference/heading checks,
prolonged GPS jump requalification, queue freshness handling, and altitude-only
origin metadata handling. Horizontal origin changes remain faults. GUI process
management protects the new bridge. 156 regressions, ROS build and GUI checks
passed. Native stationary fixture rejected a ~50 m GPS jump and submerged GPS,
continued through outage and recovered surface GPS while retaining Cube horizontal
validity. This did not validate moving mission dynamics or full ROS integration.

## October 11 live deployment

Transferred Git bundles because Jetson could not resolve GitHub. Preserved thirteen
Jetson-local sonar commits and untracked files; scoped restart retained sonar PIDs
and MAVProxy ownership. Observation-only continuous bridge installed and tested.
Measured surface baselines were session-specific, not persistent calibration facts.

After reboot, connected via Wi-Fi, confirmed valid DVL lock, removed the temporary
yaw correction (yaw_offset_deg0, roll mounting transform retained), and backed up,
applied and verified external-only Cube parameters. Initial parameter verification
read old queued replies; fresh drained readback matched. Final observed bridge had
fresh VN/DVL, qualified GPS, Surfaced and near-zero depth. Heading was not physically
validated. Bridge output and missions stayed disabled; no live waypoint was run.
The later activation SSH request was not executed because automatic approval
review hit the usage limit. See deployment-2026-10-11.md for backups and exact state.

## October 12 preparation

Added a passive 12-second waypoint_preflight.py snapshot and test-order document.
Reports sensors, depth/GPS, profile differences, bridge identity and output,
supervisor mode, control/kill state. It never sends vehicle commands. Temporary
runtime launch overrides vanish on reboot while Cube parameters persist; therefore
reboot alone does not restore a ready estimator. Full ROS/native moving mission
validation, physical heading/map validation and deliberate commissioning remain.

## Teammate sonar context

Original user-supplied reports copied verbatim under docs/teammate-context/:
sonar_updates_2026-10-04.md, sonar-session-2026-10-10.md and
sonar-status-2026-10-11.md. These are Ruth's reports, not independently inspected
recordings or a fresh hardware survey. October 11 explicitly supersedes October 10:
the apparent pipe band follows the vehicle track/ringing; ESC-noise attribution
is weakened by Gate0; missed-ping waits were reduced to about1s. Keep those
corrections when using older notes.

Sonar mapping separately dead-reckons VN/DVL and does not consume Cube fused
position. It has its own Start-relative frame, 0.42m sonar-forward offset,
height/turn/validity gates, recording/replay and map controls. Do not conflate
that frame with geographic waypoint navigation. Reported hardware repairs include
topside Fathom-X replacement, Ping360 power-cycle recovery and loose kill connector.
Latest reported code2b30056 was on Jetson, not GitHub at reporting time; our merge
preserved that history. Follow-by-distance autonomous sonar behavior was not built.

No supplied recording, replay HTML, camera footage or data sheet was attached here;
their conclusions remain attributed to the teammate. Dates in Graey recordings
may be wrong; relative timing is more trustworthy per the October11 report.
