# Waypoint deployment and preparation — 2026-10-15

Date follows the operator's America/Los_Angeles calendar. Jetson wall clock
reported October 13; correlate telemetry using monotonic time/session identity.

## Completed deployment

- Tether SSH reconnected at 192.168.2.2. Operator confirmed floating with DVL
  bottom lock, killed/disarmed. Fresh telemetry verified disarmed MANUAL,
  physical kill engaged and ESC power off before changes.
- Preserved Jetson sonar history through `17397c8` and all untracked files.
  Merged tested navigation/UI commits through `44731b2`, producing deployed merge
  `20293431d607090dc153c9e4d3cde9f434fc6e17` on `tether_toggle_beichen`.
- Backup: `/home/graey/deployment-backups/20261015-waypoint-223525`, with
  predeployment Git bundle, controller/calibration state and service configuration.
- Stopped the hand-started `tools/august_bridge.py` and restarted the existing
  core launch. Observed one fused navigation bridge, supervisor and pilot control
  node; continuous launch override retained. MAVProxy remains serial owner.
- Jetson ROS build succeeded; 173 Python regressions passed. Build warnings were
  limited to disabled byte compilation. No physical mission was started.
- Before restart controller selected TETHER with TETHER_LOST latched. Restart
  selected RF, no inhibit, kill engaged, disarmed and ESC off. Operator must check
  desired RF/TETHER selection before testing; no connection recovery was requested.

## Verified correction

Temporary setup left `EK3_SRC1_POSZ = 1`, conflicting with the continuous bridge
profile. Restored only that differing parameter to `6` (external navigation).
Bar30 pressure is read by the bridge and becomes odometry depth. Before/after
parameter snapshots are in Jetson logs as `external-profile-*` JSON files.
All expected profile parameters passed readback; running bridge reported
`configuration_verified=true`, empty `profile_mismatches`, fresh pressure and
fresh VN/DVL inputs. Kill/disarm/ESC-off were checked before each write.

The first attempted correction was interrupted before a confirmed write; the
completed run returned `changes: {EK3_SRC1_POSZ: 6}, verified: true`.
See `docs/evidence/deployment-correction-2026-10-15.json`.

Build alone retained an older installed `tools/waypoint.html`. Backed it up to
`/root/robotx_ws/logs/waypoint-before-stats-20261015.html`, refreshed the installed
copy from committed source and verified the served page contains Cube estimate
stats and dated imagery controls. No further navigation restart was needed.
This verifies served content; full browser visual inspection remains pending.

## Operator calibration and consistency record — pending

Operator will run Calibrate and Prepare personally. At final deployment snapshot,
calibration job was idle, pressure baseline zero, calibrated depth unavailable,
odometry withheld and north alignment not established. These are expected
pre-Prepare conditions, not mission readiness. Relative VN mode was observed.

After Prepare, record independent true-heading references at multiple orientations,
corrected VN heading, Cube heading and wrapped disagreement. For position, record
stationary drift, Cube/GPS separation and a physically measured displacement with
timestamps. Agreement between Cube and bridge is consistency, not independent
accuracy. No post-calibration accuracy measurements or wet waypoint results have
yet been collected in this session.

Next: complete calibration/Prepare, verify the chosen controller source and fresh
navigation, document comparisons, then supervise a short clear waypoint mission.
