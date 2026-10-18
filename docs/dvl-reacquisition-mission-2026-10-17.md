# Bounded DVL mission recovery and surface measurements — 2026-10-17

Local development only on tether_toggle_beichen. No SSH, vehicle deployment,
live parameters, sensor restart or main changes in this task.

## DVL behavior

Previously the bridge could reacquire DVL while the mission irreversibly aborted:
the mission did not pause until readiness failed, at which point its independent
healthy-bridge hold prerequisite was already false. A real saved run stopped
with one valid reacquisition sample. Recovery of the sensor did not recover an
already ABORTED mission, correctly preventing an unexpected restart.

Now the mission notices bridge dvl_continuity.recovering immediately, including
while the supervisor still reports ready. It captures one Cube NED hold target
and replaces the travel target once. The hold is fixed, not chased as position
changes. The stage, geographic target, depth reference and north alignment stay
unchanged. Arrival dwell resets on validated resumption, so recovery cannot
accidentally complete a stage based on predicted/paused samples.

The bridge's prediction horizon is explicitly increased from1.0 to1.5seconds
total from the last good DVL measurement, including reacquisition validation.
This is a provisional wet-commissioning limit, not proof of position accuracy.
Existing conservative uncertainty growth, 5m modeled-sigma validity ceiling,
fresh VN/depth, valid Cube pose, unchanged mission reference/profile and control
permissions remain mandatory. Prediction stays labeled with reduced ODOMETRY
quality; it is never used to validate GPS or align north. One horizon does not
restart on invalid packets. Mission pause itself has a fixed1.5second deadline;
the bridge's older gap deadline can end it sooner.

Three consistent valid DVL samples spanning0.2seconds must clear recovery before
the mission can resume the ORIGINAL target. A delayed supervisor publication
can use the remainder of the same pause interval; it does not restart a timer.
Non-DVL publication interruptions retain their shorter0.7second allowance.

When the bounded healthy prediction ends, there is no automatic continuation on
an untrusted horizontal pose. Mission requests MANUAL/pilot takeover as before.
Real sensor/frame faults, excessive uncertainty, depth overshoot, changed origin,
watchdog failure, kill, pilot disarm/mode change and operator Abort remain stops.
An already aborted mission NEVER auto-resumes when DVL later recovers. Unknown
missed displacement still requires explicit position revalidation. We did not
introduce an unverified ALT_HOLD fallback or bypass a hardware failsafe.

The position hold relies on bounded predicted navigation and Cube control; it
does not guarantee a physical stop or cure the unresolved DVL mounting/controller
response issue. Independent physical frame and stopping tests remain necessary.

## Surface measurements before descent

After Start, WAIT_SURFACE collects a full20second stationary window before
PREPARE/GUIDED/arming. If the operator starts already armed in pilot mode, the
mission still waits and does not command autonomous station keeping during this
window; hold the vehicle still using the established procedure. No new movement
script or heading calibration is added.

Required evidence: fresh admitted GPS and VN, confirmed shallow pressure state,
healthy bridge, valid non-recovering DVL, receiver horizontal accuracy<=2m,
DVL speed<=0.05m/s and VN turn rate<=2degrees/s. Record distinct GPS fixes only;
repeated/stale samples cannot manufacture the window. At least40fixes are needed.
The current20second window must have maximum scatter radius<=1.5m and maximum
GPS displacement unexplained by independent DVL path length<=2m. DVL path length
comes from raw velocity, not GPS-aided bridge position, avoiding circular checks.
Invalid motion/sensors clear qualification; excessive scatter ages out only as
the rolling window actually becomes stable. No raw GPS coordinate is overwritten
or forced onto the map. No Cube origin is reset.

GUI shows progress, scatter and qualification reason, plus an explicit warning
that stability does not prove absolute accuracy. Mission status includes the
measurement result and GPS/DVL disagreement. Constant GPS bias can still pass,
as covered by a regression test. These starting thresholds need real data review;
they are not surveyed pool accuracy guarantees.

## Validation record

224 Python tests and both GUI Node scripts passed. Tests cover fixed hold/resume,
deadline exhaustion/no resurrection, pilot takeover, frame/fault safeguards,
full surface dwell, bad/moving/repeated samples, wandering GPS and constant bias.

Native GUI SITL waypoint-6e88a75bdb passed a southeast target with a1.05second
injected DVL interruption. Logged pause124.3188->125.7270monotonic (~1.408seconds)
and resumption without MANUAL during reacquisition; the later MANUAL was normal
RECOVER_GPS after surfacing/disarm. Surface qualification took20.0416seconds and
the original target was preserved. Final simulated target error0.0787m. This
used localhost/domain73 and QGC forwarding disabled. Synthetic matched axes
do not validate GRAEY's physical mounting or hydrodynamics.

Native dvl-recovery-e2c0a87b passed brief/extended automatic recovery and genuine
long-loss latching but FAILED explicit recovery on the simulated maintenance
authorization gate. The fixture had used a fixed delay rather than observing
the GUI authorization state. Updated only the fixture to wait for the actual
simulated maintenance lease before requesting recovery; production interlocks
unchanged. Retain this failed result. Subsequent validation recorded below.

Final225 Python tests and both GUI scripts pass. Second explicit-recovery run0a7753d5 reached the maintenance gate but failed the stationary-history gate: the new lease wait could finish immediately before the required history accumulated. The fixture now also collects2.5s stationary history. Final dvl-recovery-380fc670 passed all recovery paths preserving north; operator Abort runwaypoint-cb56e096af passed and requested MANUAL. Both failed fixtures remain in evidence. VN status rate is degrees/s (explicit regression); surface checks use raw DVL velocity, not GPS-corrected/clamped velocity.


## Tether deployment verified

User explicitly requested deployment. Fresh maintenance state confirmed killed,
disarmed,ESC-off/no mission. Preserved Jetson teammate commitsa400020 (survey,
pipeline and vision) in merge5bddc10; no main changes. Backed up at
/home/graey/deployment-backups/20261017-reacquisition-204149, built, verified all
tracked source/installed Python+GUI assets and repaired stale installed modules.
225 tests passed against installed runtime. Restarted core; exactly one bridge,
new1.5second horizon present, fresh VN/DVL, vehicle still killed/disarmed ESC-off.
Source/GitHub/Jetson5bddc10 at code deployment.

Postrestart baseline0/north unaligned/bridge disabled/Observation are expected
until surfaced Prepare and north alignment. Controller selection returnedRF.
Unexpected live configuration mismatch: EK3_SRC1_POSZ actual1 vs profile6.
No live parameter write performed as part of deployment. Asked user whether to
restore agreed ExternalNav6 or leave barometer1 for review. Mission not ready;
do not bypass the profile gate. Evidence reacquisition-postdeploy-2026-10-17.json.
No pressure calibration, north maneuver, arming or physical mission started.
