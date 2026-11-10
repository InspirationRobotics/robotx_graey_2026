# Prepare sensor-ingress recovery — 2026-10-30

Local change on navigation_new; not deployed or vehicle-tested.

Prepare now diagnoses recently observed backwards DVL/VN/pressure timestamps
and clears only the affected receive guard. It also reinitializes Cube/GPS
timing after the explicit Cube-reboot or GPS-clock-regression fault, then
requests fresh telemetry. This is an in-process warm restart of sensor ingestion,
not a process/service restart, sensor power cycle or full EKF reset. A missing
or dead bridge service still requires separate recovery.

The existing manual-control/no-autonomy Prepare authorization remains. Position,
covariance, heading correction, geographic registration and pressure baseline
are retained. Sensor quality rejection is not overridden. No Pixhawk parameters,
mode, arm state, hardware kill or ESC permission are changed. Success means the
data path was started, not that all sensors have subsequently recovered.

Bridge status exposes ingress_rejections and prepare_recovery. The Prepare
response reports which guards were restarted and that fresh samples are awaited.
Ordinary duplicate timestamps do not trigger recovery. No periodic restart loop.

Validation: 56 focused transport, estimator and guided-mission tests passed.
New tests exercise rejected DVL timestamps followed by successful ingestion,
Cube timing recovery, preserved estimator/heading/baseline, and unchanged GPS
quality/unrelated faults. Initial test run failed because the mocked clock lacked
monotonic_ns; fixture corrected and rerun passed. git diff --check passed.

The October 28 incident is not proven to have involved DVL timestamp regression;
the recorded live fault was a Cube restart. These changes cover observed code
failure paths without claiming a definitive physical DVL diagnosis.
