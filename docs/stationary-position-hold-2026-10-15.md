# DVL stationary position hold — 2026-10-15

User reports GPS wandering while bottom-locked DVL indicates almost no movement.
Added a stationary constraint to the single bridge estimator. Cube still fuses
bridge odometry through its existing external-navigation profile; no direct GPS
fusion, Cube origin reset or source switching is introduced.

After an initial qualified GPS position and north alignment, hold requires two
continuous seconds with valid fresh DVL, full body-speed magnitude <=0.03m/s,
fresh VN and pressure, angular rates <=2deg/s, no bridge fault and no pending DVL
recovery. Invalid DVL or stale VN/pressure clears the hold; motion above the
threshold or faster rotation releases it on the next DVL sample. It does not
extrapolate across missing DVL data. Recovery/Prepare clears it.

While active, bridge NE remains fixed and odometry velocity is zero. Calibrated
pressure depth and VN attitude still update. Fresh GPS still goes through quality,
timestamp and innovation checks, but it cannot translate position or train motion
biases while held. Raw GPS remains visible and logged. No covariance collapse or
claim of improved absolute GPS accuracy: the initial location may still be wrong.
Threshold deliberately treats very slow motion below3cm/s as stationary; assess
this limit in wet testing before relying on such slow manoeuvres.

The waypoint Cube stats panel exposes stationary hold state and its reason.
Cube's internal prediction may show small residual fluctuations while converging
to the fixed external pose; this is not a command to hard-reset the Cube estimate.

182 Python regressions passed, including slow GPS drift while stationary,
movement release, invalid DVL/stale VN release and first-anchor initialization.
Waypoint page regression passed. Isolated native waypoint result and deployment
verification are recorded below when available. Physical accuracy not established.

The first native run exposed a real integration bug: the supervisor required a
recent GPS correction, so holding translation prevented surface mission admission.
It timed out in WAIT_SURFACE. Evidence retained in
`docs/evidence/stationary-hold-initial-failure-2026-10-15.json`.
Fixed by distinguishing `stationary_validated_recently` from `applied_recently`.
Supervisor accepts the former only with qualified GPS, an active hold, healthy
estimator and transmitting odometry. GPS must still pass timing, quality and
position innovation checks. This is fresh measurement validation, not a fictitious
position correction. No other mission admission gate was removed.

The rerun passed through the actual GUI API: PREPARE/GUIDED/ARM/DIVE/TRANSIT/
SURFACE/DISARM/RECOVER_GPS/COMPLETE. GPS outage and valid underwater Cube pose
were observed; final simulated target error0.152m. GPS-decision logs include the
stationary-hold withholding event before movement. QGC forwarding disabled.
Evidence: `docs/evidence/stationary-hold-waypoint-pass-2026-10-15.json`.
