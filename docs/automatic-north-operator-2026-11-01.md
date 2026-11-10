# Automatic north correction restored locally

User requested automatic alignment to run as an operator in the estimator.
ContinuousEstimator now calls CourseHeading after admitted GPS position updates,
using time-matched measured DVL velocity and orientation. GPS/DVL direction
comparison includes sideways motion; waypoint bearing is never a measurement.
The comparison is converted through the geographic map rotation first.

An accepted step adjusts EKF yaw and the persistent-in-session VN yaw offset.
Subsequent VN observations therefore retain the correction. This is a bounded
bias adaptation operator using already-fused GPS, not an additional independent
measurement: it does not shrink covariance. Geographic position, velocity, map,
mission target and heading-reset revision are not rotated. Manual heading/frame
reset remains a separate operation. Cube receives corrected odometry as before;
the mission reads subsequent Cube heading telemetry.

The existing operator's motion/uncertainty evidence requirements remain. Invalid
DVL or rejected GPS resets the evidence window, not the mission. Missing or stale
matched DVL/VN pauses correction. Stationary GNSS cannot establish nose heading.
The continuous estimator allows discrepancies larger than the old 25-degree
limit, but retains the 0.5-degree maximum step and at least 3-second interval.
Evidence accumulation can make the effective interval longer; large initial
errors may take minutes. Other callers retain the old 25-degree limit.
No separate forward calibration maneuver or new mission gate is added.

85 focused tests passed: estimator sensor-loss behavior, course evidence logic,
manual heading correction, waypoint transport/controller, bridge, and new
operator tests. New tests cover both correction signs, large initial mismatch,
no-DVL pause, fixed geographic XY/covariance, VN offset retention, rotated-map
handling, and invocation through normal admitted GPS updates. This is not a
native SITL convergence test or live vehicle validation. Large-error convergence
under GPS innovation rejection and real receiver bias remains unverified.

Local only. No deployment, vehicle commands, parameter changes or Git push.
