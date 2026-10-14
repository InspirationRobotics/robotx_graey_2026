# Wi-Fi deployment and land preparation — October 13, 2026

User authorized deployment and checklist checks, reporting killed/disarmed.
User subsequently confirmed Graey is on land; water calibration and submerged
checks are deferred. Wi-Fi endpoint verified at 192.168.8.105.

Initial live controller: fresh, disarmed, MANUAL, SA kill active, ESC commanded
off, RF healthy, no mission claim. Tether watchdog did not receive replies from
192.168.2.1. No control-source change requested or performed.

Initial Jetson HEAD 1cc4364 included navigation through cc7772c and local sonar
history. Untracked sonar data/backups and other local files were preserved.
After reboot the temporary launch override had expired: legacy nav_ekf_bridge
was running, with observation supervisor and no valid calibrated depth.

Transferred navigation bundle through 6843f9d. Guarded deployment backed up Git
history/status and verified kill/disarm before mutation. Initial merge attempt
under sudo failed for missing root Git identity before changing tracked files;
retry used graey's existing identity. Merge succeeded and colcon build succeeded.
All 165 on-device regressions passed. Restart completed; final Jetson merge HEAD
is d0ca60c, containing navigation through 6843f9d and preserved sonar history.
Backup: /home/graey/deployment-backups/20261013-nav-183434.

Verified persistent service override selects continuous_fusion=true and baseline
0 (uncalibrated), retaining bridge output disabled and supervisor observation.
Scoped ExecStop targets core launch rather than all package processes. Readback
confirmed /etc/systemd/system/graey-ros.service.d/90-navigation-observation.conf.

Wi-Fi interrupted several SSH connections and ping. The existing deployment
connection later delivered the passing test result and successful restart.
Final passive preflight confirmed continuous bridge, fresh vehicle telemetry,
fresh VN data, matching Cube profile with no mismatches, disarmed and kill/ESC
off. VN yaw about40.972degrees is a sensor observation, not physical validation.
DVL invalid, depth uncalibrated, GPS unqualified, heading uncommissioned, output
disabled and supervisor observation remain blockers. Evidence on Jetson:
/root/robotx_ws/logs/waypoint-preflight-20261013-land.json inside container.
User requested waiting until Graey is floating with bottom lock; no water
baseline was established from the land measurement.

## Remaining checklist

- Done: merge/build/165 tests, persistent continuous launch, profile readback,
  passive preflight and final kill/disarm/ESC-off state.
- Recheck process counts, GUI waypoint route and full raw sensor status at next
  connected surface session; process-count command was interrupted by Wi-Fi.
- In water: calibrate confirmed-surface pressure, establish DVL bottom lock,
  independently validate heading, enable odometry with verified healthy Cube
  estimate, and observe disarmed submerged/surface continuity.
- Mission-process-death takeover gap from architecture review remains open.
- Approximate pool-image registration remains unsuitable for live image targets.
- No arm, waypoint mission, output enablement or Cube parameter write authorized
  by this land deployment has been performed.
