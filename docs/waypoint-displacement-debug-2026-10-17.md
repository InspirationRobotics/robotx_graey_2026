# Wrong endpoint: offline investigation — 2026-10-17

## User evidence

User supplied two screenshots and confirmed the first pink selection marks the
actual physical endpoint; the second is the commanded waypoint. They are not
evidence that the mission was commanded to the first point. The screenshots
were taken after disconnecting; no screenshot time series or physical trajectory
is available. User reports three runs: first dived, changed to MANUAL, surfaced;
second traveled to the wrong endpoint; third moved left and was disarmed to
avoid collision. Heading appeared good. Surface start appeared accurate.

Screenshot coordinate text:

| Point | Latitude | Longitude |
| --- | --- | --- |
| Actual endpoint marked by user | 32.9235546 | -117.0385309 |
| Intended target | 32.9235847 | -117.0385069 |

Difference is about3.35m south and2.24m west (~4.03m). The first screenshot's
blue Cube estimate remains displaced from the user-marked physical endpoint.
This is evidence of disagreement, not a measured survey or proof of a faulty
projection. A constant satellite registration offset alone cannot explain an
accurate start followed by incorrect estimated relative travel.

## What could be checked offline

- Browser selects geographic latitude/longitude; a subsequent click clears the
  cached plan. Start passes the selected point to the single-use plan API.
- API stores that point and mission subprocess receives its plan JSON. Mission
  converts it with the Cube origin into LOCAL_NED north/east. It retains captured
  yaw separately; it does not rotate the north/east target by the vehicle yaw.
- Added a browser regression replacing a southern selection with an eastern one
  before Start, verifying the newer point is sent. Added a mission regression
  verifying southeast targets at headings0,90,108,180,270degrees remain southeast.
- Added configurable north/east target to the isolated validator and assertions
  that the production mission plan exactly matches the selected geographic point.
- Native GUI SITL waypoint-344a552a2c completed southeast(-1mNorth,+3mEast), with
  brief DVL loss and supervisor interruption injected. Final simulated horizontal
  error0.08594m; no target replacement occurred.215 Python tests passed before the
  new heading regression; final216 tests passed plus GUI Node regression.
- These tests assume simulator sensor axes and vehicle model agree. They do NOT
  validate GRAEY's actual DVL mounting, thruster signs/mixing, VectorNav attitude
  frame, firmware external-navigation handling or pool hydrodynamics.

## Remaining hypotheses; no speculative changes

Correct displayed heading does not prove DVL velocity is in the body frame used
by the bridge. The driver currently publishes DVL axes unchanged, and bridge
uses them as body forward/right/down. A mismatched mounting orientation or sign
could produce wrong displacement while the heading looks reasonable. This is
an unverified assumption, not a newly diagnosed hardware fact. Do not insert an
arbitrary90/180degree correction or reverse lateral velocity without evidence.

Missed DVL travel during dropouts, external-navigation versus Cube frame/estimate
disagreement, Cube/thruster response and resurfacing GPS correction also remain
possible. The earlier mission's depth-overshoot MANUAL cause cannot automatically
be assigned to these three later runs. User disarm on the third run is explicit
pilot takeover and must remain effective. Do not bypass aborts to finish a run.

## Diagnostic addition and next evidence

Add WAYPOINT_TRACE every0.5s during DIVE/TRANSIT/SURFACE to the existing mission
log: plan, final target NED, last target actually sent (including captured yaw),
start/Cube/bridge NED, pressure depth, Cube yaw, raw DVL velocity/validity, bridge
heading, DVL recovery state, raw GPS and admission decision. This preserves the
time relationships missing from stage-only summaries. No extra sensor node,
control path, parameter write or navigation behavior change is introduced.

Vehicle unavailable: graey.local hostname resolution failed, tether192.168.2.2
SSH timed out. User explicitly has no access currently. No live changes or
deployment in this investigation. Source changes go only to tether_toggle_beichen;
user's untracked docs/nav_lgoic.md preserved.

When vehicle is available, retrieve all three original waypoint stage logs,
result JSON and the corresponding MAVProxy tlog or Cube BIN before another
powered test. Compare intended click -> recorded plan -> sent NED -> Cube motion
-> independently observed motion. A killed/disarmed measured forward and lateral
translation is the next useful frame check if logs do not establish axis mapping;
no need for another uncontrolled waypoint to discover that. Preserve the good
north calibration and record headings rather than recalibrating preemptively.

## Wi-Fi retrieval follow-up

Connected read-only at192.168.8.105. Current telemetry reported armed/MANUAL,
kill released/ESC-on; no software, control, sensor or parameter changes made.
Jetson source7ebb36c; offline diagnostics8d3c5e6 not yet deployed. Current bridge
has a new instance and no north alignment after restart; do not treat old session
preparation as current. Saved mission logs were copied locally before analysis.

Three consecutive runs after shallow-limit deployment match the user's sequence:

| Run prefix | Relevant evidence | Result |
| --- | --- | --- |
| 5d8280d8 | 0.4m depth request; 0.58736m pressure at abort, 1.90s after DIVE entry | Overshoot guard requested MANUAL before transit |
| c6c3c7af | Target1.157m south/5.133m east; reached TRANSIT for4.60s; DVL REACQUIRING with one valid recovery sample and odometry withheld | Permission loss requested MANUAL; waypoint never completed |
| 4d5705a4 | Target0.194m south/3.547m east; stayed in DIVE, never TRANSIT; fresh healthy bridge and navigation ready at stop | Pilot disarm; no mission-requested MANUAL preceding disarm |

The second run's estimated movement at abort was0.097m south/0.690m east; the
third run's estimated movement was0.375m north/0.496m west. These are Cube
estimates, not independent physical measurements. The second recorded target
(32.923583377,-117.038506347) is near the screenshot's intended selection,
not its marked actual endpoint. It was not an exact coordinate match because
the screenshot selection was made later; no claim of byte-identical match.

Third-run sideways motion occurred while the mission was still commanding
horizontal hold at the start during descent. That narrows the investigation to
position/attitude/velocity frames, depth-to-horizontal coupling and controller
or thruster response; changing geographic waypoint conversion alone would not
address that behavior. First-run overshoot despite verified shallow limit
readbacks also demonstrates speed parameters alone did not solve the physical
vertical response. Keep the pressure guard; do not hide overshoot by widening it.

Only the post-restart MAVProxy tlog remains at its fixed path. It does not provide
the pre-restart mission time series needed for full physical frame diagnosis.
The350MB persistent ROS log and saved stage logs retain decisions, but no new
WAYPOINT_TRACE was deployed for these runs. Older DVL observer capture at
monotonic15735..15760 is later than these111xx runs:224samples,2invalid,maxgap0.381s.
It cannot be used as evidence that DVL stayed valid during these missions.

Next read-only evidence is the Cube onboard BIN covering boot time11084..11154,
plus verified DVL sensor/body mapping. Do not repeat powered waypoint testing
to substitute for a disarmed frame and control-response verification.
Evidence summary: docs/evidence/physical-waypoint-runs-2026-10-17.json.
