# Jetson observation deployment — October 11, 2026

User authorized SSH deployment and preparation for waypoint testing, then confirmed
Graey floating at the surface, disarmed and stationary.

Connected over tether to 192.168.2.2; graey.local did not resolve. SSH host key
matched the existing local known-host record. Jetson also could not resolve
GitHub, so verified branch history was transferred using Git bundles.

Jetson started at 2b30056 with thirteen local sonar commits beyond the common
ancestor and four navigation commits missing. Preserved those commits and all
untracked files. Backed up Git history, status and installed workspace at
/home/graey/deployment-backups/20261011-bridge/. Merged a5d9973, then 121a25a
(pressure launch argument and read-only profile diagnostics). Final observed
Jetson merge HEAD e04b831 includes both navigation and local sonar work.
Online tether branch contains 121a25a; whole checkout identity is not claimed,
and unrelated Jetson-only sonar commits were not published by this deployment.

156 regression tests pass on Jetson. Colcon build succeeds with byte-compilation
disabled warnings. Restart used a scoped stop targeting the core launch only;
sonar launcher/map processes retained their original PIDs. MAVProxy remained active.
Fresh disarmed, ESC-off, no-mission telemetry was checked before each restart.

Runtime override:
/run/systemd/system/graey-ros.service.d/90-navigation-observation.conf
starts continuous_fusion:=true with surface_pressure_hpa:=1013.745361328125.
This override is temporary until reboot. Default installed launch still uses
legacy observation. Do not persist the surface baseline across future sessions
without revalidation. Bridge enabled=false and supervisor active=false.
No Cube parameter writes or live navigation commands were made.

Confirmed one fused_nav_bridge and no legacy nav_ekf_bridge executable. VN
yaw_offset_deg remains 90; physical heading reference is not validated.
Raw pressure captured 111 unique samples over 12 seconds, span 0.095 hPa,
median 1013.745361328125 hPa. After application supervisor reports Surfaced,
depth approximately -0.006 m. DVL updates arrive but validity is false;
GPS admission correctly reports fresh valid DVL motion unavailable.

Cube profile differences (actual -> proposed): GPS_AUTO_SWITCH 1->0;
SRC1_POSZ 1->6; SRC2_POSXY 3->6; SRC2_POSZ 1->6;
SRC3_POSXY/VELXY/YAW 0->6; SRC3_POSZ 1->6.
No changes applied. Missing valid external output means the Cube estimate is
not ready for waypoint use; observation-only deployment is not mission readiness.

Final controller: disarmed MANUAL, RF selected, SA kill active, ESC commanded off,
no mission/autonomous claim. Restart changed selection from the earlier TETHER
state. Operator must deliberately select the intended control source during later
testing; no fault recovery or arming was performed by this deployment.

Next actions before waypoint execution:
1. Restore stable valid DVL bottom lock while remaining disarmed.
2. Validate physical forward direction/north against VN and Cube; do not mark
   mounting commissioned merely because the software offset exists.
3. Finish full ROS/native and moving mission validation, then back up/apply/read
   back the external-only Cube profile under controlled commissioning.
4. Enable bridge output and verify Cube position/depth/heading continuity through
   a disarmed surface/submerged transition before enabling mission permission.
5. Confirm map registration, a short in-pool target, intended control source and
   stop/abort response. No waypoint mission is ready yet.
