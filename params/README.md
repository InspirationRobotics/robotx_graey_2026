# Cube parameter files

Graey's Cube runs **ArduSub 4.7.2** (since Oct 23 2026). A file written for another firmware can
use names that no longer exist (4.5 `GPS_TYPE`, `WPNAV_SPEED_DN` became 4.7 `GPS1_TYPE`, `WP_SPD_DN`
in m/s), so check the version line below before loading one in QGC (Parameters → Tools → Load
from file, Graey killed and disarmed, then reboot the Cube).

| File | Firmware | For | What it is |
|---|---|---|---|
| `graey_4.7_rfd_competition.parm` | 4.7 | Cube | RFD telemetry stream rates (`MAV1_*`) and `MAV_SYSID 3` for the competition radio |
| `graey_pool_waypoint_limits.parm` | 4.7 | Cube | Slow vertical limits for shallow pools (same as `odometry_profile.SHALLOW_LIMITS`) |
| `navigation_sources.parm` | 4.5 and 4.7 | Cube | Candidate EKF source sets: external nav underwater, GPS at the surface |
| `graey_4.5.7_autonomous_validated.parm` | 4.5.7 | `sim/` only | Full backup from Jul 23 2026 (976 parameters, the first autonomous runs). `sim/inside.sh` loads it into the 4.5.7 simulator with `sim/sim_overrides.parm`. Don't load it on the 4.7.2 Cube. |
| `sitl_graey.parm` | 4.5.7 | `sitl/` only | SITL EKF sources + simulated GPS. Never on the real Cube. |

Removed Nov 13 2026 (in git history): `graey_4.5.7_dvl_ekf.parm` and `graey_4.5.7_vn100_ekf.parm`,
Jul 22 backups identical to each other and to `autonomous_validated` except
`EK3_SRC1_YAW 1` (compass yaw; `autonomous_validated` has 6, external yaw), `PSC_ACCZ_IMAX 100`
(500) and `WPNAV_SPEED 100` (45), plus barometer ground pressures and run counters.

## Saving a current 4.7.2 backup

MAVProxy writes the Cube's full parameter list to `/root/robotx_ws/logs/mav_3_1.parm` each time it
connects. From the laptop on the tether or WiFi:

    ssh graey@graey.local "docker exec graey cat /root/robotx_ws/logs/mav_3_1.parm" > params/graey_4.7.2_$(date +%F).parm
