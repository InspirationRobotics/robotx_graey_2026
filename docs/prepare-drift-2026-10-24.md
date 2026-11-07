# Prepare while drifting — 2026-10-24

User requested sensor/odometry health preparation without lake-motion rejection.
For fixed-local navigation, commissioning no longer requires GPS aiding qualification or a near-zero +/-0.1 m depth sample. It requires a fresh valid GPS fix (age under1.5s), healthy transmitting valid local odometry, verified profile, healthy Cube pose and existing heading consistency. Surface confirmation and killed/disarmed/ESC-off remain because Prepare resets position and pressure zero.
Pressure baseline uses the existing median of fresh samples; removed8hPa peak-to-peak rejection for bobbing. No stationary requirement in Prepare UI. Mission validation and GPS fusion rejection unchanged. Initial odometry can still depend on the estimator accepting a GPS fix; this patch does not force healthy status or fake initialization.

Validation:27 commissioning/calibration tests and calibration GUI tests pass. Initial test invocation used wrong working directory; rerun exposed obsolete wording assertions and Windows temporary-directory permissions. Updated assertions (including prior GUIDED wording stale after STABILIZE change) and reran with workspace temp directory successfully.

Local only; no vehicle deployment in this change.
