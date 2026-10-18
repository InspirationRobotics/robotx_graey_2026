# Automatic surfaced DVL position recovery — 2026-10-17

The GUI's Cube marker became stale after a long DVL gap exhausted position
confidence. The bridge latched a position fault and stopped odometry even after
valid DVL returned. Cube local position became stale; GLOBAL_POSITION_INT alone
continued and coincided with raw GPS. This does not prove the current QGC marker
and GUI marker were using different coordinate conversions. User authorized
automatic recovery, debugging and deployment.

## Implemented behavior

Short DVL dropouts retain existing in-mission bounded prediction/reacquisition.
This adds recovery of the *latched position uncertainty fault*, without pressing
Recover or recalibrating an uninterrupted north reference:

- The only auto-clearable bridge fault is DVL continuity/position uncertainty.
- Fresh control status must report MANUAL, disarmed, kill engaged, ESC off,
  no autonomous mode and no mission claim. No motor/mode/arm commands are sent.
- Pressure must confirm the surface, Cube geographic origin must be unchanged,
  and VN orientation and the existing north reference must remain continuous.
- Existing two-second stationary recovery checks require fresh DVL bottom lock,
  speed <=0.05 m/s and VN rate <=2 degrees/s.
- A separate recovery GPS admission object checks timestamp age, fix, precision
  and independent DVL path before allowing the main fault to clear. Require GPS
  reported horizontal accuracy <=2 m and at least ten distinct fixes across three
  seconds, with geographic spread <=1 m. A rejected jump is not a trusted anchor.
- Reuse existing recover_dvl: retain north offset/bias and pressure baseline,
  reset position uncertainty/velocity bias, increment ODOMETRY reset counter, then
  require normal GPS requalification before sending odometry again.
- Arming/releasing maintenance state during requalification withholds restored
  odometry until safe again. Other faults and lost VN continuity remain blocked.
- Supervisor consumes completed recovery once, invalidates old mission references
  and clears only its corresponding DVL fault. It does not reactivate a deliberately
  suspended supervisor, clear other faults, or resume/start an aborted mission.
- Waypoint page displays the automatic recovery waiting/completed reason.

Strong GPS reception and real stationary bottom lock remain necessary. No amount
of automatic recovery establishes true position from bad GPS or repairs heading
drift; stable GPS bias can still pass. Recovery is deliberately blocked underwater
after actual position confidence is exhausted, because a returning velocity sample
does not reveal displacement accumulated during the loss.

## Validation

241 Python tests and Node waypoint GUI tests passed. Initial transport fixture
failures were caused by the AST adapter harness not initializing the new recovery
dependency/state; fixed the fixture and reran the full suite successfully.

Isolated native ROS/ArduSub SITL `dvl-recovery-4b69192a` passed: brief and extended
bounded gaps recovered; a long gap latched and withheld output; after the isolated
disarmed maintenance lease was provided, GPS-qualified recovery resumed odometry
automatically with one reset and unchanged heading. No recovery service request
was sent. QGC forwarding was disabled. Evidence is
`evidence/automatic-dvl-recovery-2026-10-17.json`. Its old fixture field
`explicit_recovery_passed=true` was a shared result label, not evidence of a manual
service invocation; the fixture label is corrected for future runs.

Deployment requires the usual guarded restart. A restart still needs initial
Prepare/north alignment; future eligible DVL faults recover automatically within
that running session. Hardware state/readiness must be verified after deployment.
