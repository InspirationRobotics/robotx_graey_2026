# One-metre waypoint segments — local changes, 2026-10-22

Requested in the side conversation: replace continuous waypoint travel with
approximately one-metre travel/check segments; increase arrival radius to 1.5 m.

Both initial straight north alignment and waypoint transit now brake after one
metre of fixed-local horizontal displacement (DVL/VN-aided Cube estimate).
Braking distance is additional; this is not a guarantee of exactly 1.00 m travel.
The check requires two seconds with speed <=0.08 m/s, VN yaw rate <=3 deg/s,
and current qualified GPS. Movement/GPS failure resets the check dwell, not the
accumulated alignment track. Existing bounded GPS/navigation/depth aborts remain.
Transit has a 240 s overall bound; alignment has a 90 s overall bound and the
existing 7 m maximum-distance guard. Initial alignment still needs its multi-metre
baseline and finishes braking near 4.5 m; a one-metre leg alone does not prove north.

Sensor fusion remains continuous. GPS/DVL course evidence is gathered while
moving, but a bounded queued heading correction is applied only at an authorized
stopped TRANSIT_CHECK. At most one update per check; stale evidence over ten
seconds is discarded, new missions clear queued corrections, and corrections
remain frozen within two metres of the target. Stationary pauses retain the
long position baseline; invalid motion/turning still invalidate unsuitable data.
No correction is forced when confidence is insufficient. Map position aiding
continues under existing admission/stationary gates.

Arrival: estimated distance <=1.5 m, speed <=0.15 m/s, and current GPS for one
second; brake, verify stopped, then disarm. GUI wording and mission status expose
the new radius and segment checks. No physical accuracy guarantee is implied.

Validation: 19 focused Python tests passed, including closed-loop segmented
missions at 0/73/-120 degree map offsets, repeated alignment/transit stops,
GPS/dropout/abort rejection, dwell reset, retained baseline, stale/near-target/
duplicate correction rejection, and bridge authorization/motion gating. Waypoint
JavaScript harness passed. The first test run exposed an old short test target
that reached the new arrival radius before a transit stop; extended the test
route to exercise multiple segments. No native SITL, live mission, deployment,
firmware flash, commit or push was performed in this side conversation.

Parent-thread firmware migration edits in odometry_profile.py, waypoint_mission.py,
sitl/run.sh and test_firmware_profile.py were preserved. The previously recorded
depth/firmware issue is not fixed by this segmented-travel change.
