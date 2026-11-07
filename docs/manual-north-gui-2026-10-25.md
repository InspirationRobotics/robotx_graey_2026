# Manual north alignment from the mission GUI

Implemented and deployed on 2026-10-25 at the user's request.

The main waypoint page now includes **Manual north alignment**. Point the bow at a known true heading, enter that heading (0 degrees north, 90 east), and click **Apply heading / align north**. The action sends no movement command. It uses the existing manual/no-active-autonomy service policy; it does not arm, restore ESC power, select a control source, or start navigation output.

The ContinuousEstimator applies the difference between the supplied heading and its current yaw through the existing heading correction function. This rotates local world-frame velocity, covariance and history around the current position, retains a VN yaw offset, and reanchors an existing geographic map without moving the current geographic position. The bridge increments its odometry reset counter through the existing heading-reset path. If no VN sample has initialized the filter, the requested heading waits for the first sample. No GPS or DVL qualification gate was added to this manual action.

The manual reference is valid for the bridge session. Restarting the bridge clears it. It relies on the operator's measured true heading; this is not an independent accuracy measurement. When output is paused, the service reports that Start Navigation is needed before the correction is transmitted. Cube/QGC yaw convergence remains dependent on Cube ExternalNav fusion and has not been physically demonstrated by this deployment.

Subsequent GUI or adapted QGC Go-to missions use the manual reference and skip the initial alignment travel. The normal automatic path retains its 10 m GUIDED alignment leg. This supersedes the unconditional automatic-leg descriptions in the earlier QGC integration report when a manual reference has been applied. Existing control/kill/failsafe authority is retained; no new mission abort condition was added.

## Implementation

- `/graey/navigation/align_heading` uses `rcl_interfaces/srv/SetParametersAtomically`, with double parameter `true_heading_deg`.
- GUI POST `/api/calibration` accepts `action: align_heading` and `true_heading_deg`, then exposes completion through the existing job status.
- Runtime changes: `continuous_estimator.py`, `fused_nav_bridge.py`, `guided_surface_mission.py`, `calibration_jobs.py`, and `tools/waypoint.html`.

## Verification and deployment evidence

Local affected Python tests and waypoint JavaScript tests passed. The full calibration test module encountered Windows temporary-directory permissions in unrelated VN calibration cases; the full module passed on the vehicle's Linux runtime.

Onboard tests: 5 heading correction, 10 calibration jobs, 22 bridge transport, 9 alignment fusion gate, and 7 QGC adapter tests: **53 passed**. Source/installed runtime files were verified. Deployment archive: `output/manual-north-gui-20261025.zip`.

Fresh disarmed MANUAL/no mission/no autonomy was verified before replacing runtime files and restarting `graey-ros`. MAVProxy was not restarted. Backup: `/home/graey/deployment-backups/manual-north-gui-20261024-031645` (vehicle clock differs from this report date).

Postdeployment read-only checks confirmed the served manual controls, the ROS service type, both systemd services active, a fresh bridge instance `da0fd31b-661f-44c0-bbb1-8e220685f569`, verified profile with no mismatches, and controller disarmed MANUAL/STARTUP/ESC off. Navigation output remained disabled with pressure zero unset. No arbitrary heading was applied and no motion was initiated during verification. These are deployment-time observations, not ongoing vehicle status.
