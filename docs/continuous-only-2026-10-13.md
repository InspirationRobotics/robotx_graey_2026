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
