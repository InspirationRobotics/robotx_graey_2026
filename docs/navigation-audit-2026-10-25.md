# Navigation audit — 2026-10-25

Branch: navigation_new. Local work only; no vehicle connection, deployment, parameter change or push. Preserve unrelated docs/nav_lgoic.md.

## Startup ownership

GUI sensor group nav is core-managed: /api/start and /api/stop refuse independent navigation process actions. The spawn helper also rejects dvl_node, vn100_node, fused_nav_bridge, nav_ekf_bridge and navigation_supervisor. Start Navigation calls the existing /graey/navigation/prepare service, never launches a second bridge. The service is idempotent for ContinuousEstimator: it retains the filter object, position, map and reset counter. Core launch has one fused_nav_bridge, with respawn disabled. Linux flock ownership is acquired before node construction; tests exercise twenty concurrent contenders and crash release. Cooperating instances must share the lock directory/scope; this is not a guarantee against arbitrary third-party serial/odometry publishers.

## Remote operation clarification

User clarified GUI plus RadioMaster is intended, not new RC switch mappings. Ordinary Start Navigation no longer requires disarm, kill engaged or ESC-off. It permits fresh MANUAL or STABILIZE control with no mission claimed and autonomy false. The same condition is enforced by GUI jobs, bridge service and calibration page. Surface confirmation is still required to initialize pressure zero correctly. Repeated startup does not reset that baseline or the EKF. No action arms, restores ESC power, changes kill, or clears a control fault.

The unchanged physical VN six-face calibration is a pre-launch activity with propulsion disabled; software cannot perform physical rotations, repair a cable or guarantee remote recovery from hardware failure. Normal mission startup, remote arm/disarm/kill/takeover and sensor reacquisition do not require reaching the vehicle. Full sensor/firmware restarts are not the same operation as Start Navigation and were not silently made safe while armed.

## Preserved math and maps

No changes to track_north.py, course_heading.py, geographic_frame.py, straight_alignment.py or the navigation filter/heading correction math from 552673c. Fixed Cube-local frame; map x=north/y=east; initial 10 m forward STABILIZE leg with lateral feedback; GPS/DVL geographic heading fit; ongoing bounded correction; neutral vertical 500; continuous target travel and estimated arrival dwell remain.

Miramar saved tiles remain under tools/vendor/miramar; pool.png, pool_reference.png and neighbor_pool.png remain. Packaging recursively installs vendor tiles. GUI still offers saved Miramar, world satellite, dated pool satellite and the old approximate screenshot view. The old screenshot remains view-only for live targets. Tests explicitly verify installed Miramar/pool assets and GUI selection/coordinate functions.

## Validation

300 Linux/ROS Python regression tests pass; waypoint and calibration JavaScript suites pass. Focused Windows calibration tests encountered sandbox temporary-directory permission errors; the same suite passed on the production-like Linux environment. An initial targeted Linux module invocation collided with the test namespace; corrected to unittest discovery.

Native SITL uses ArduSub 4.7.2, production bridge/mission/GUI, ROS domain 73, localhost only and QGC forwarding disabled. Site selection changes simulator latitude/longitude; it does not model pool walls, lake shoreline, waves or measured currents. Injected DVL error is a valid-tagged [3,-3,0] m/s velocity for 5 s, rejected by normalized innovation. GPS is disabled for 5 s then restored; measured acceptance loss is checked, not just the injection request. The full sequence covers mode change/arm, 10 m alignment, navigation, estimated arrival, stop and disarm.

Pool run a5ddd76657 completed with DVL rejection and GPS loss/recovery confirmed. Miramar first attempt b67a3ba096 exposed a simulator fixture omission: missing controller revision made GUI correctly report offline. Stopped before mission, added the required revision field to the isolated fixture, and reran. Production watchdog already supplies revision. Final Miramar result recorded below after completion.

The Miramar harness additionally calls GUI startup twice, requires ready acknowledgement, checks unchanged bridge instance, and counts exactly one fused_nav_bridge process before starting the mission. No manual north heading, hand-driven alignment or GUI intervention is required during the scripted run.

Final Miramar run 47b8597a39 PASSED: full mission COMPLETE/disarmed, DVL innovation rejection observed, GPS loss observed and all three sensors accepted again at completion. Two GUI startup requests acknowledged; one bridge process and unchanged bridge instance. Initial track 9.90017 m; injected +40 degree VN offset fitted -40.03363 degrees. Pool fitted -40.02423 degrees. Sampled vertical inputs were exclusively 500 in both runs. Exact summaries: navigation-audit-results-2026-10-25.json. Browser rendered Miramar-area imagery with no page script errors; saved tile packaging and pool selection were separately regression tested.

No new RC commands or switch assignments were introduced. User explicitly clarified that GUI commands are acceptable. Ordinary GUI navigation startup in armed MANUAL is unit/UI tested; native startup tests used the simulator's disarmed startup state. Neither is a live-vehicle certification. Current-disturbance limitations from the previous session still apply; these geographic-site runs have no lake-current model.


## Test-distance adjustment
User requested a 5 m initial straight alignment leg (4.9 m completion tolerance) and 1.5 m estimated arrival radius. Updated mission, GUI, API wording, and local rendered sequence diagram. Heading fit and one-second arrival dwell remain unchanged. Fourteen focused Python tests and both GUI suites pass. Local only; not deployed.


## Tether deployment — 2026-10-25
Deployed navigation_new code plus 5 m initial leg and 1.5 m arrival radius to Jetson. Existing source and installed runtime backed up at /home/graey/deployment-backups/navigation-new-20261024-001504 (Jetson clock date differs from session date). Overlay preserves unrelated sonar/control work and existing branch; this is not a Git branch synchronization. Core rebuilt and restarted; 159 installed Python/GUI/launch assets checked against source. Fourteen focused onboard tests pass. An initial import check without ROS workspace environment failed; rerun with sourced environment passed. Live HTTP waypoint verified 5 m /1.5 m and GPS record controls. Exactly one fused_nav_bridge and one navigation_supervisor, no august bridge. Fresh MANUAL/disarmed/kill engaged/ESC off verified before and after. Cube configuration_verified=true, profile_mismatches empty. Bridge disabled pending Start Navigation surface pressure zero; VN accepted, DVL not valid, raw GPS fix4 accuracy0.875m observed but not admitted before surface initialization. No parameter writes, arming, navigation start or physical mission test.

## Deferred task
- Clean up Jetson code and reconcile deployed overlays with Git history. User explicitly deferred this work; do not perform cleanup until requested. Local working branch verified as navigation_new.
