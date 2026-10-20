# Compass Learn stalled: confirmed firmware/profile conflict — 2026-10-19

Live Graey: ArduSub 4.5.7, reported commit 30257f01. Armed MANUAL, COMPASS_LEARN=3, all EK3_SRCn_YAW=1, active learning acknowledgement received, no firmware completion event or mode-zero transition, about 375 seconds elapsed. All horizontal position/velocity sources remain ExternalNav (6). Bridge output paused for calibration. GSF run/use masks both 3. This is not evidence of a slow successful calibration.

Exact upstream firmware source was downloaded and inspected at commit 30257f01:

- libraries/AP_Compass/Compass_learn.cpp: learner requires soft-armed, at least three seconds likely-flying, an available GSF estimator, positive yaw variance below 5 degrees and successful fixed-yaw magnetic calibration before it changes learning to zero and reports completion.
- ArduSub/ArduSub.cpp: calls set_likely_flying(soft_armed); Sub does support this flight-time prerequisite. It is not the identified configuration conflict.
- libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp, runYawEstimatorPrediction and runYawEstimatorCorrection: both return immediately unless horizontal position source is GPS AND GPS horizontal velocity is enabled. With our ExternalNav-only profile these update/correction functions do not run. The configured GSF masks do not override this check.

Sources:
https://raw.githubusercontent.com/ArduPilot/ardupilot/30257f01/libraries/AP_Compass/Compass_learn.cpp
https://raw.githubusercontent.com/ArduPilot/ardupilot/30257f01/libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp
https://raw.githubusercontent.com/ArduPilot/ardupilot/30257f01/ArduSub/ArduSub.cpp

Conclusion: the previously implemented workflow enables the compass learner while preserving a source profile that prevents its required GSF estimator from updating. More waiting or lowering GUI thresholds does not resolve this conflict. Earlier unit/GUI tests verified the adapter/state machine with synthetic completion events, not actual firmware learner convergence. Earlier instruction to repeat driving was incorrect without checking this firmware prerequisite.

User decision requested: permit a temporary native-GPS calibration profile in MANUAL only, restoring and verifying the bridge-only profile before Prepare; or retain the no-native-GPS policy and choose a different independent north-establishment method. No exception has been approved at the time of this note. No GPS source parameters, arm state, flight mode or propulsion commands changed in this investigation. Preserve the current session for read-only diagnosis; Cancel in MANUAL can end incomplete learning without claiming a north reference.

Required follow-up whichever route is chosen: validate prerequisites BEFORE starting learning, show an explicit configuration conflict rather than an endless running state, preserve original parameters with readbacks/rollback, block Prepare until final bridge profile and north evidence are valid, and verify actual firmware behavior with native SITL plus physical checks. A simulated completion message alone is insufficient validation of this workflow.
