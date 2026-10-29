# EKF navigation with manual heading correction only

Deployed 2026-10-25 following the user's instruction to remove automatic north alignment from QGC and Jetson missions.

QGC Go-to now passes through MAVProxy to Cube unchanged. The QGC HTTP mission adapter was removed from the ingress wrapper, including its MANUAL mode request, command consumption, synthetic acknowledgments, periodic reply pump and Jetson mission launch. `scripts/qgc_goto.py` was removed. Native Cube handles Go-to/GUIDED navigation and uploaded QGC missions. The sole MAVProxy serial owner and existing pilot/mission input arbitration remain.

ContinuousEstimator now registers the first admitted GPS fix to the current EKF position with zero geographic rotation, without waiting for heading qualification. GPS updates continue through the existing measurement admission, delay compensation and EKF innovation handling. Automatic north track fitting and rolling GPS-course map rotation are disconnected. Legacy begin/finish alignment calls are inert. Mission status cannot start an automatic alignment. The reference uses VN/EKF heading until the operator applies a known true heading through the existing GUI manual control.

Jetson surface missions request STABILIZE and enter normal waypoint transit after arming. They have no initial straight alignment leg, stopped heading acceptance phase, heading validation requirement or alignment-corridor requirement. The old per-sensor DVL/VN startup vote was removed. The GUI describes direct EKF navigation and retains the manual heading input on the same page. Existing surface-start confirmation, navigation startup, geographic coordinates, estimator/control telemetry, pilot takeover and hardware/connection failsafe behavior remain; no new readiness or abort checks were introduced.

Manual heading correction still rotates EKF yaw, world velocity, covariance and history around the current position and retains a VN offset for the bridge session. It sends no movement command and does not automatically enable navigation output. Start Navigation enables the existing odometry path; Cube/QGC receives the correction when that output is running. A restart clears the manual reference. Actual heading accuracy and native Cube waypoint performance require physical observation and were not demonstrated by deployment tests.

## Verification

Local affected regression tests: 39 Python passed; waypoint GUI JavaScript passed. Updated tests now verify direct native command forwarding, GPS fusion without north qualification, legacy alignment messages having no effect, direct Jetson transit without manual alignment or corridor confirmation, retained manual correction and transport/reset behavior.

Onboard Linux runtime: the same 39 tests plus the full 10 calibration job tests passed, **49 total**. Source/installed Python and HTML equality and compilation were verified. The modified hook compiled against the actual installed MAVProxy source, with no QGC adapter reference.

Deployment package: `output/manual-only-ekf-20261025.zip`. Backup: `/home/graey/deployment-backups/manual-only-ekf-20261024-032321` (vehicle clock). Installed-tree backup: `/root/robotx_ws/logs/installed-before-manual-only-ekf-20261025.tgz`.

Fresh disarmed MANUAL/no autonomous mission was observed before service replacement. Both ROS and MAVProxy were restarted; the exact surviving old wrapper process was terminated before starting its replacement. Postdeployment checks observed one MAVProxy owner (container PID 65280), both services active, fresh bridge `9f3c3eb9-6726-4657-8158-e55b100428b4`, verified profile/no mismatches, served manual-only GUI, and no alignment collection. Controller was disarmed MANUAL/STARTUP/ESC off; navigation output was paused. No arbitrary heading, arming, parameter change or motion was commanded. These observations describe deployment time only.

This supersedes the automatic 10 m alignment behavior documented in the earlier QGC Go-to and manual north reports.
