# Calibration GUI deployment — 2026-10-14

Authorized tether deployment to 192.168.2.2 after fresh confirmation of RadioMaster
kill engaged, disarmed MANUAL, ESC power off and no claimed mission.

- Merged navigation through `9faf100` into the Jetson's existing sonar history,
  producing `45b9525721b891772126a9612e37c5bd582a6d56` on
  `tether_toggle_beichen`. No conflicts; tracked tree clean. Preserved untracked
  sonar data/backups and the already-running sonar pose relay. Main untouched.
- Backup: `/home/graey/deployment-backups/20261014-calibration-225900`, including
  predeployment Git bundle, controller snapshot, status and service configuration.
- Stopped core before merging, built the ROS package successfully, and ran all
  142 Python regressions successfully in the Jetson container. Only routine
  setuptools byte-compilation-disabled warnings appeared. Restarted core using
  the existing persistent continuous observation override, pressure baseline0.
- `/calibration` and `/api/calibration` returned200. New VN calibration start/cancel
  and navigation prepare/pause/suspend/commission ROS services are present.
- Exactly one fused bridge and one supervisor process observed. VN calibration
  telemetry reports relative heading mode; VN and DVL streams fresh, DVL valid.
  Bridge Cube profile verification passed. No physical calibration was started.
- Output disabled/not transmitting; supervisor Observation. Pressure sample is
  fresh but calibrated depth intentionally unavailable until floating-surface
  preparation. GPS withheld for missing pressure eligibility; north alignment and
  position initialization remain incomplete. This is not mission readiness.
- After restart, controller selected RF (before restart TETHER with TETHER_LOST
  latched). No explicit selection or fault-recovery command was sent. Fresh
  readback remained killed, disarmed MANUAL, ESC-off, no mission/autonomous.
  Operator must check desired control selection before subsequent testing.
- Jetson journal clock labels this restart October13; session date here follows
  the workstation October14. Do not use wall-clock strings alone to correlate logs.

Next: follow the Calibration page instructions for mounted six-face calibration,
then Prepare only while floating stationary at the surface. Complete manual
GPS/DVL north-alignment movement and the killed/disarmed Cube fusion checks before
any supervised waypoint. Do not infer a calibrated heading from relative VN yaw.
Real calibration/positive preparation remain unverified. Mission-process-death
protection remains a separate unresolved prerequisite for unattended operation.

Repository synchronization: navigation code is present online and deployed.
Jetson also contains additional sonar history; this deployment does not claim
the whole Jetson and online Git histories are identical or publish that history.

## Prepare status race found during live use

After physical VN calibration reported collected six-face counts11/11/9/8/15/42
(relative mode, volatile), the user encountered a failed Prepare job:
`Navigation maintenance; Prepare required before reuse`. Fresh bridge telemetry
showed the reset had actually succeeded: baseline1009.2704467773438hPa,
reset_counter2, surface state true, GPS qualified, valid DVL and fresh VN,
output permission true but transmission withheld pending north alignment.
GUI consumed a cached supervisor Fault from its own pause operation immediately
after the prepare service ACK. Fixed by requiring supervisor publication time at
or after that ACK before processing faults/alignment, with a10s bounded wait.
New faults are still rejected; no missing/stale status can grant permission.
Regression coverage includes cached fault, absent new status and real new fault.

Follow-up: the first publication-time fix was insufficient. The supervisor had
also latched the intentional pause as an operational fault; even new status
therefore retained the maintenance reason. Updated the production supervisor
callback to treat exactly this maintenance sentinel as non-latching only while
inactive and bridge output disabled. Existing faults are never cleared by this
exception; other faults and maintenance while active still latch. Added callback
regressions for pause/reset, preservation of a real fault and active-mode pause.

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

## GPS qualification cycling

Passive100-snapshot capture during the reported cycle:57 qualifying,26 GPS/DVL
displacement disagreements,12 excessive/unavailable horizontal accuracy,3 admitted,
one DVL continuity loss and one invalid DVL-motion observation. This capture did
not establish stale timing as the live cause; actual GPS/DVL quality issues exist.
Separately reproduced a software defect: status polling erased the qualification
dwell when a delayed but initially-fresh fix expired between normal5Hz packets.
Fix retains dwell only within the existing receipt-continuity bound; expired fixes
remain unqualified and never forwarded. New packets still pass all checks. Real
quality errors, receipt gaps, clock loss, VN/DVL/depth failures still requalify.
Added end-to-end estimator regression:5Hz fixes delayed250ms with20Hz status polling
now complete north alignment. No freshness, accuracy, angular or movement threshold
was relaxed. Added qualification progress/last-reset cause and observed GPS accuracy,
velocity uncertainty and speed to calibration diagnostics.150 Python tests and
Windows Node calibration-page check passed; WSL lacks Node (not a browser-test pass).
Physical moving alignment remains unverified and may remain blocked by poor GPS.

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

## Stationary known true-heading option

Added Calibration page method selector: GPS/DVL movement or Known true heading
(stationary). Operator must enter0<=heading<360 degrees clockwise from true north
and explicitly confirm the independently measured nose bearing. GUI validates it,
then Prepare suspends mission/output, sets surface baseline and waits2.2s before
requesting capture through /graey/navigation/align_heading. That service uses
rcl_interfaces/SetParametersAtomically with exactly one double true_heading_deg;
it is a dedicated checked service, not a persistent ROS parameter write.

Bridge independently requires fresh killed/disarmed/ESC-off controller telemetry,
Prepare output permission, uninitialized/unvalidated session, pressure-confirmed
surface, fresh VN/DVL,>=1.5s history with>=10 samples, speed<=0.05m/s, all angular
rates<=2deg/s, tilt<=15deg and yaw stability within3deg. Captures true minus current
relative VN yaw as session offset; freezes reference, records source/value/time
and rotates preinitialization history. Does not reset Cube origin, VN calibration,
arm/mode/ESC state or GPS accuracy gates. Qualified GPS still anchors position;
healthy Cube fusion is still required for mission permission. New Prepare/restart
invalidates reference; cannot alter a live initialized frame via this service.

161 Python tests pass and Node calibration-page test passes. Includes invalid
input/confirmation, movement/depth/stale rejection, service safety, cancel before
capture, immutable session reference and simulated stationary position anchoring
with0.49m/s GPS velocity uncertainty followed by underwater DVL/VN continuation.
No new full native GUI/SITL run or physical known-heading capture claimed. A mocked
service test initially lacked logger/status fixtures; corrected then suite passed.

2026-10-14 known-heading feature deployed over tether: navigation through236815a
merged on Jetson asa2a7ed7, package rebuilt,161 tests passed there. Installed
service/client imports and served Calibration HTML verified. Live unconfirmed
known-heading request rejected409 with no job/reference change. Fresh controller
killed/disarmed MANUAL/ESC-off. Core observation output off, no heading captured;
operator must supply actual independent true bearing. Backup at
/home/graey/deployment-backups/20261014-known-heading-234523. Main untouched.

## Controlled Pixhawk recovery after DVL fault

Live Cube heartbeat/attitude stayed fresh, but LOCAL_POSITION_NED went stale after
bridge latched DVL continuity loss and withheld odometry. DVL later returned valid.
No evidence yet distinguishes one invalid report from a longer interruption.
User confirmed kill/disarm. First stationary recovery aborted before mutation due
to heading movement. A later attempt reset bridge/captured retained-frame heading
but GUI failed on supervisor's old real DVL latch. Fixed Prepare ordering: after
bridge reset ACK, require fresh clean bridge status, wait old publications beyond
0.5s freshness gate, and explicitly suspend supervisor again while maintained.
Only explicit killed/disarmed Prepare does this; no automatic airborne/underwater
fault clearing, synthetic velocity or degraded odometry publication is introduced.

The recaptured heading in this recovery is derived from the existing validated
north offset plus uninterrupted live VN orientation, NOT a new independent
compass observation. Recovery JSONs under deployment-backups preserve provenance;
GUI's generic Operator measured label alone is not sufficient provenance here.
Automatic bounded DVL-dropout recovery remains separate and is not implemented.

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

## Bounded DVL reacquisition — 2026-10-14

Live passive ROS capture observed224 DVL samples in25s,2 invalid reports,
maximum interarrival0.381s, no>=0.5s gaps, no nonadvancing sensor timestamps and
max observer header age0.009s. Confirms intermittent invalid published measurement,
not persistent bottom-lock loss or stale transport. It does not distinguish native
velocity_valid=false from malformed/nonfinite reports; both publish invalid.
Capture retained on Jetson /root/robotx_ws/logs/dvl-continuity-observer-20261014.json.

Previously one invalid sample latched permanently after initialization. Now an
invalid measurement immediately withholds odometry, clears GPS qualification/history
and starts bounded reacquisition. Two subsequent valid velocities must agree to
within0.2m/s per component, and the entire gap/recovery must finish in<0.5s.
Missing translation is not invented/caught up. Position variance is enlarged by
(5m/s * missing interval)^2 on reacquisition: an engineering displacement budget,
not measured or guaranteed error. Continuing invalid reports restart the valid
sample count.>=0.5s outage, VN failure, Cube restart and existing hard faults still
latch and require explicit revalidation. Mission readiness requires healthy inputs;
this does not guarantee an active waypoint survives a real dropout or auto-restart
an aborted mission. No arm/mode/kill/watchdog behavior or pool registration changed.

167 regressions pass. Native isolated ROS/GUI/ArduSub waypoint2bbd195be6 completed
dive/transit/surface with underwater GPS outage and valid Cube estimate; synthetic
endpoint error0.121m is not physical accuracy. Separate native fault-injection
run dvl-recovery-efa7bef1 verified brief invalid report recovered to healthy
transmitting odometry, and sustained invalid reports latched with output withheld.
Both used ROS domain73/localhost and QGC forwarding disabled; owned processes cleaned.
Evidence copied to docs/evidence/dvl-recovery-2026-10-14. Reproducible standalone
injector: sitl/validate_dvl_recovery.py, run from repository root after sourcing ROS.
Initial nominal launch lacked repository import path; corrected using PYTHONPATH=.
Injector initially lost ROS import path due environment override; rerun with system
Python and original sourced ROS paths succeeded. Those attempts are not test passes.

2026-10-14 bounded DVL recovery deployed through ebdcf9c on Jetson as9f100a3.
ROS build and167 regressions passed on Jetson. Fresh API exposes dvl_continuity,
VN/DVL inputs fresh, no bridge fault; killed/disarmed ESC-off, RF selected.
Restart intentionally baseline0/output disabled; known-heading Prepare required.
Backup /home/graey/deployment-backups/20261014-bounded-dvl-001311. Native simulation
validated recovery; live postinitialization dropout recovery still awaits wet test.
No real mission started, no main changes; sonar history/untracked files preserved.
