# Miramar preparation — October 18, 2026

User requests: hold position during pre-mission GPS validation; lake satellite
coverage; north alignment separate from floating-surface Prepare; investigate
reported descent, MANUAL transition and RadioMaster buzzing. This session is
local only: no SSH, vehicle parameters, reboot or deployment.

## Mission change

Surface path: WAIT_SURFACE (initial healthy navigation) -> PREPARE (capture
reference/start/heading) -> GUIDED -> ARM -> CHECK_SURFACE -> TRANSIT -> DISARM
-> RECOVER_GPS -> COMPLETE. Stage publications can skip very short stages.

CHECK_SURFACE actively holds the captured starting north/east position using the
same bounded GUIDED velocity controller as transit. It does not select firmware
POSHOLD or introduce source switching. Start authorizes this armed hold, which
requires ordinary ESC/watchdog permission. Kill and pilot takeover remain effective.

The five accepted seconds of stationary GPS/DVL/VN measurements restart their
collection on entering CHECK_SURFACE; samples collected before arming cannot
satisfy the hold. Existing interruption tolerance and measurement limits remain.
After qualification, the fixed geographic target becomes active. While holding,
the waypoint goal remains metadata; outgoing horizontal setpoints target the
captured start. Hold is not a guarantee against wind, frame errors or GPS bias.

The prior upward-only Z requests, 0.20 m/s horizontal cap, pressure >0.08 m abort,
vertical-reference consistency checks, reference/origin validity and bounded
GPS/DVL interruption logic all remain. Holding cannot bypass these checks.
Timeout remains 90 seconds; it cannot indefinitely wait with propulsion enabled.
Legacy explicit dive missions retain the original qualification sequence.

## Separate north capture

Calibration page now has Capture north, independent of Prepare. With kill
engaged, disarmed and ESC off, align the nose along an independently known true
bearing, enter it and confirm. A separate bridge service checks two seconds of
fresh, level, low-rate and consistent VN orientation. No GPS, DVL bottom lock or
pressure calibration is required for this step. It does not enable odometry,
commission navigation, arm or start motion. Current corrected land heading is
displayed with the stored session reference.

Keep VN powered continuously while transferring to water. Capture is volatile;
restart, >=0.5 s VN gap, invalid attitude, detected orientation discontinuity or
active calibration invalidates it. Normal measured rotation while carrying is
tracked, not treated as a new true bearing. Relative VN drift is still possible.

At the water surface, choose Use captured land north and Prepare. Prepare still
requires the real floating-surface confirmation, fresh stable pressure, completed
VN calibration status and verified Cube ExternalNav profile. It creates a fresh
position estimator and retains only the continuously tracked north reference;
GPS/DVL/pressure/Cube health and commissioning must still pass. If the land
reference expires, preparation reports failure rather than claiming retained north.

Alternative GPS/DVL surface-movement alignment remains available. Selecting it
clears an existing land reference under maintenance so the old bearing cannot
silently override the chosen method. Automatic GPS/DVL alignment is not possible
on stationary land without another absolute bearing measurement. The dormant M9N
compass is not enabled by this work.

## Satellite map

Current Esri World Imagery is the default; the pool's 2023 archive remains optional.
Miramar overview centers approximately at 32.916, -117.099 for browsing, not as a
surveyed dock or navigation origin. It is not an inferred coordinate from the
user's perspective photograph. Added north/south/east/west pan and wider zoom
(overview to local maneuver scale). Fit map / vehicle returns to vehicle scale.
Map clicks continue to use georeferenced latitude/longitude; no screenshot
registration is used. The existing 0.3–20 m waypoint range remains unchanged.

Verified the public overview tile endpoint returned HTTP 200, image/jpeg:
https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/16/26416/11450
This checks service availability, not exact shoreline placement or surveyed
accuracy. The operator's browser still needs internet. No offline tiles are bundled.

## Reported behavior and next live checks

Unintended descent is NOT expected surface-mission behavior. A MANUAL transition
can be a deliberate mission abort, firmware failsafe, control-watchdog action or
another command sender; the transition alone does not identify its cause.
RadioMaster buzzing cannot be diagnosed from the sound report. Retrieve the
mission abort context, Cube STATUSTEXT/DataFlash and watchdog/radio evidence.
No new live evidence was available, so do not claim the reported incidents fixed.

For Miramar:

1. Confirm latest branch deployment and the exact running revision. Preserve logs
   from prior attempts. Do not run an older unfinished build by mistake.
2. Complete land north capture or choose surface movement alignment. Keep VN
   powered; compare corrected heading against an independent bearing.
3. Float killed/disarmed, Prepare at the actual waterline, check pressure near
   zero, bottom lock and Cube/bridge agreement. DVL altitude is bottom clearance,
   not depth below the surface. Deeper water or a steep bottom can remove lock.
4. Independently verify vertical response before another autonomous mission.
   Upward-only requests do not prove motor polarity, trim or controller behavior.
5. Observe raw GPS and Cube over time against a recognizable physical location.
   Open sky may improve reception but is not proof of accuracy. Do not offset the
   map or reset an origin simply to hide a GPS discrepancy. Keep the antenna clear
   of water and account for dock/shore multipath.
6. Use a short clear-water target with recovery line/tether slack, a pilot at kill,
   and no swimmers/obstacles/boat traffic in the path. Validate the armed hold
   before transit. Stop on sinking, unexpected thrust or heading inconsistency.
7. Confirm RF/tether selection and normal manual takeover before launch. Kill
   permissions are unchanged. Record the exact radio warning and GUI abort reason.

City information checked October 18: Miramar lists no swimming/wading and a
current algae-bloom water-contact advisory. Confirm reservoir staff authorization
for the robotic test and plan retrieval without entering the water:
https://www.sandiego.gov/reservoirs-lakes/miramar-reservoir

## Local validation

- 253 Python tests passed, covering north capture without DVL/pressure, continuity
  expiry, orientation-jump rejection, preserving north through Prepare, no
  commissioning from land capture, surface hold targeting, depth abort and Abort.
- Both calibration and waypoint JavaScript GUI suites passed, including land
  capture without surface confirmation, lake overview/pan and archive/current tiles.
- Native ArduSub/ROS/GUI SITL waypoint-9f24f15ce3 passed with CHECK_SURFACE before
  TRANSIT and an injected 2-second GPS outage; accepted hold exposure 5.1006 s;
  final simulated horizontal error 0.2136 m. These are simulation results only.
- First hold-Abort run waypoint-a358af6998 correctly reached ABORTED with operator
  reason and MANUAL confirmed, but the validator reported false because it
  incorrectly required a TRANSIT observation. Corrected that validation condition;
  the failed result is preserved rather than relabeled.
- Hold-Abort rerun waypoint-2533f81f7d passed: armed GUIDED hold observed, GUI Abort accepted and MANUAL confirmed before transit. Abort yields pilot control and does not automatically disarm. Evidence: docs/evidence/miramar-hold-abort-2026-10-18.json.

All native tests isolate localhost ROS domain 73 and disable QGC forwarding.
No vehicle state has been verified during this session. The previously proposed
slow GPS/DVL heading-consistency redesign is not implemented by this change.
