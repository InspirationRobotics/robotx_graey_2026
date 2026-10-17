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
