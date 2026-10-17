# DVL recovery and GPS rejection — 2026-10-17

## Verified problem

The running Jetson at `730fd75` already included recovery and GPS rejection.
It nevertheless stopped odometry after six DVL dropouts: cumulative missing
distance reached 0.753 m and tripped the fixed lifetime 0.75 m allowance. Fresh
DVL had returned, north remained validated, and the Cube profile matched, but
the continuity fault remained latched. Physical kill was engaged, disarmed,
ESC power off, no mission. This was not evidence of a broken DVL sensor.

GPS movement validation also cleared its accepted geographic anchors on invalid
DVL reports. A dropout could therefore weaken rejection of a displaced fix.

## Changes

- Keep bounded earth-velocity prediction for at most one second. Expose its
  predicted status explicitly. Thereafter withhold odometry rather than invent
  more travel. Reacquire after three consistent valid samples spanning 0.2 s.
- Remove the lifetime 0.75 m cutoff. Account for each missing interval in the
  existing position uncertainty and retain that uncertainty after reacquisition.
  Six brief dropouts no longer arbitrarily require IMU calibration. Noisy or
  interrupted reacquisition restarts the sample check.
- Retain the existing 5 m position-sigma validity ceiling. A genuinely excessive
  unobserved-position error still needs explicit position revalidation; good
  velocity alone cannot recover an unknown displacement. This ceiling is not a
  pool-clearance guarantee. The 0.2 m/s + 1 m/s² missing-motion envelope remains a
  provisional model requiring wet validation, not a measured error certificate.
- Preserve GPS movement anchors across DVL loss, allowing conservatively for
  unknown travel. Hold a stationary anchor beyond the rolling history window,
  so slow GPS drift cannot become valid simply by outlasting that window.
- Reject confidently reported GPS speed inconsistent with DVL speed (receiver
  speed sigma <=0.2 m/s and disagreement >max(0.3 m/s, three sigma)). Poor GPS
  velocity confidence remains unusable for north alignment; it does not alone
  invalidate an otherwise usable position measurement.
- Retain timestamp/clock checks, fresh valid DVL, receiver position accuracy,
  pressure-confirmed surfaced gate, qualification time, residual gates and
  stationary position hold. Rejected GPS never becomes a new trusted anchor.
- GUI displays recovery state and position uncertainty instead of the obsolete
  accumulated allowance. Navigation recovery never arms or releases kill, and
  an aborted mission does not restart itself.

GPS checks detect stale data, jumps and inconsistent movement. They cannot
prove absence of constant geographic bias or improve the M9N's physical accuracy.
Small jitter inside the consistency margin remains possible. Raw measurements
remain available separately from admitted navigation data.

## Validation

- 210 Python regression tests passed, including six short dropouts, prolonged
  silence/reacquisition, eventual genuine uncertainty exhaustion, GPS jump after
  DVL interruption, slow stationary drift and velocity disagreement.
- Both Node GUI regression scripts passed.
- Native isolated SITL `dvl-recovery-fee0ad0d` passed brief automatic recovery,
  1.3 s output suspension followed by automatic reacquisition, prolonged loss
  with position fault, and explicit surfaced position recovery preserving north.
  QGC forwarding disabled; localhost ROS domain 73. This does not test physical
  propulsion, kill wiring, EMI or actual DVL/GPS error distributions.

Deployment and full GUI waypoint result are recorded below when completed.

Native GUI waypoint `waypoint-7f690703db` passed the full surface/dive/transit/surface mission with injected DVL and supervisor interruptions; final simulated target error 0.1525 m. This is simulation evidence, not physical accuracy.

