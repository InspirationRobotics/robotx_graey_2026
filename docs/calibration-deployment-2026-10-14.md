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
