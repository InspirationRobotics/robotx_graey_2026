# Explicit DVL position recovery — 2026-10-15

## Operator workflow

On Calibrate / Prepare, use **Recover DVL position** after a DVL continuity fault.
Float stationary at the calibrated surface, disarm, engage kill and keep ESC power
off. Wait at least two seconds with uninterrupted DVL bottom lock. Check the
existing floating-surface confirmation and press Recover DVL position.

The operation retains the current bridge session's north alignment, GPS heading
adjustment, VN magnetic calibration and pressure baseline. No rotations, fresh
true-heading entry, or GPS/DVL alignment movement are required. Watch live
supervisor status and Cube stats before selecting a new waypoint.

## Implementation and limits

- `BridgeEstimator.recover_dvl` accepts only the specific DVL continuity fault.
  It requires a validated heading, continuous fresh VN, calibrated surface,
  unchanged Cube origin, fresh valid bottom lock, and at least 1.5 seconds of
  stationary samples in the last two seconds. Speed must stay at or below
  0.05 m/s and angular rates at or below 2 degrees/s.
- VN gaps/invalid samples are tracked even when DVL already owns the fault
  message. If VN continuity was lost, returned data alone cannot certify that
  the relative heading reference survived. Use Prepare and re-align north.
  This does not inherently require another magnetic six-face calibration.
- Recovery discards uncertain NE position/history and velocity bias, resets GPS
  admission, preserves clock mapping and waits for NEW qualified GPS. It cannot
  reconstruct unobserved underwater motion. Output remains withheld until the
  estimator is healthy and initialized; stale cached GPS cannot immediately
  restore a position. Heading offset/bias and calibrated depth are retained.
- `/graey/navigation/recover_dvl` checks fresh maintenance safety and Cube profile,
  increments the odometry reset counter, and does not issue arm/mode/origin/source
  commands. It does not clear unrelated faults.
- GUI `recover_dvl` job suspends old mission permission/reference, invokes recovery
  without the generic pause that would replace its fault, waits for post-reset
  bridge status, clears the supervisor's old latch and reuses checked commission.
  It never launches a second bridge or resumes an old mission.
- A bridge restart loses the session reference. Installing this new service needs
  one restart and one Prepare/north alignment afterward; subsequent DVL-only
  recoveries do not restart the bridge or require another alignment.

## Validation

179 Python regressions passed, including recovery heading preservation, GPS
re-anchoring at a displaced position, rejection while moving/submerged, VN loss,
unrelated faults, GUI job service order and unsafe/unconfirmed requests.
Both calibration and waypoint Node page checks passed, including disabled unsafe
controls and the new recovery POST action.

Native isolated ArduSub SITL passed short-drop recovery, long-loss latch and an
explicit call to the actual ROS recovery service. Fresh GPS qualified, odometry
resumed, reset counter advanced once, and heading reference/bias were unchanged.
Evidence: `docs/evidence/dvl-explicit-recovery-2026-10-15.json`.
QGC forwarding disabled; ROS domain73/localhost only. A simulated maintenance
status publisher was used after verifying the simulated vehicle was disarmed;
this tests the software path, not physical kill enforcement. No real recovery or
waypoint mission is claimed by this simulation result.

## Deployment

Installed on Jetson through `140a9c8`, tether branch, with fresh kill engaged,
disarmed MANUAL and ESC-off checks. Backup:
`/home/graey/deployment-backups/20261015-dvl-recovery-230552`.
Build and 179 source-tree regressions passed. Initial verification caught that
three installed Python modules were still old despite the successful build.
Explicitly backed up and refreshed bridge_estimator.py, fused_nav_bridge.py and
calibration_jobs.py. Other recent pilot/mission modules already matched source.
Installed-module backups: `/root/robotx_ws/logs/installed-before-dvl-recovery-20261015`.
Then 179 regressions passed from `/tmp`, importing the installed runtime.
Future deployments must compare installed contents, not rely on build success.

Read-only ROS discovery confirmed `/graey/navigation/recover_dvl` as Trigger.
Live bridge publishes `heading_continuity_lost`, profile verified, VN/DVL inputs
fresh, controller killed/disarmed ESC-off. Browser confirmed the served recovery
panel and instructions. No physical recovery was invoked: restart intentionally
left baseline0, north unaligned, supervisor Observation and odometry withheld.
Operator needs one Prepare/north alignment now; VN magnetic calibration was not
started or erased by the deployment. RF is selected after restart.
