# Continuous-only migration — October 13, 2026

User authorized permanent continuous navigation and removal of source switching.
Core/SITL now launch only fused_nav_bridge. ContinuousSupervisor is standalone;
removed legacy bridge/state machine, source commands/ACK handlers, alignment
publisher, Lua reporters and old executable registration. Waypoint uses continuous
permission only. GUI no longer describes legacy switching and counts fused bridge.
ROS node/lock name nav_ekf_bridge is retained for ownership compatibility only.
The continuous_fusion compatibility parameter defaults true and false is rejected.
Historical docs/evidence remain historical; not runtime instructions.

117 current regression tests passed (retired switching tests removed; new
continuous-only and permission contracts added). Both GUI tests passed. Native
GUI/API nominal mission waypoint-ac2745ffc4 and Abort waypoint-70ea88f590 passed,
including GPS outage underwater, Cube pose, and MANUAL on Abort. QGC forwarding
disabled. Evidence stored under docs/evidence/continuous-only-2026-10-13.

Live tether: killed/disarmed, ESC off. User confirmed floating at surface.
Sensor restart restored valid DVL; VN fresh; observed surface pressure median
997.84228515625hPa from98 unique samples. Cube profile matched. A prior supervisor
timing fault was latched; restart and sustained observation required. No physical
heading validation yet. Mission-process-death safeguard remains outstanding.

Deployment/post-calibration outcomes will be appended after verification.

## Deployment and surface checks
Jetson merge35eaec8 includes81fd138 and preserves local sonar history. Build and
117 tests passed. Backup: /home/graey/deployment-backups/20261013-nav-185955.
Persistent startup is continuous observation; session override95-surface-session.conf
sets997.84228515625hPa and expires at reboot. No live Cube parameter writes.

Initial post-calibration check latched DVL continuity loss despite recovered
samples. Bridge was stopped, then the guarded core restart restored it (stateful
bridge intentionally does not respawn). Final12-second passive preflight passed
sensor health, GPS admission, depth0.032m and Cube profile. Evidence on Jetson:
/root/robotx_ws/logs/waypoint-preflight-requalified-20261013.json.
Earlier failed snapshot retained as waypoint-preflight-calibrated-20261013.json.

VN yaw offset readback0.0, flip_180True. User reported physical68deg; subsequent
VN263deg/Cube12deg disagree. Awaiting stationary two-direction validation; no
heading correction guessed. Odometry output and supervisor activation remain OFF.
Cube horizontal position is not healthy while output is off; this is not evidence
of commissioned fusion. No arming, ESC enablement or mission launch performed.
Old colcon launcher archived to /root/robotx_ws/logs/retired-nav_ekf_bridge-20261013;
ros2 executable list now contains fused_nav_bridge and no legacy bridge launcher.

Stationary physical34deg follow-up: fresh VN261.746deg, Cube22.106deg. Requested controlled90deg clockwise rotation to distinguish mounting offset from response failure. No offset applied.
