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
