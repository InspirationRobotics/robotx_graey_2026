# GUIDED initial alignment — 2026-10-25 checkpoint

User requested autonomous 10 m along current Cube yaw in GUIDED, surface level retained, GPS displacement establishes north; deploy afterward.

LOCAL implementation: ALIGN_GUIDED/ALIGN_FINISH, local line advances at0.1m/s to10m; fixed yaw; streamed position+velocity target at captured pressure-corrected surface local Z. Qualified raw GPS start/end displacement supplies north offset relative to initial Cube yaw, applied to geographic transform without Cube frame reset. Bridge acknowledges run before STABILIZE waypoint transit. Initial GPS/depth/vertical-limit readiness required; pressure depth>0.25m aborts toMANUAL. Input-owner lease refreshed with neutral MANUAL_CONTROL while GUIDED receives targets.

Focused synthetic state checks pass for local line, fixed surface Z, GPS bearing, mode handoff, depth abort; syntax checked. No full nativeSITL or live movement verification. Old STABILIZE-focused regression expectations need revision. Endpoint uses admitted individual GPS fixes, not endpoint averaging yet.

Archive staged only: /home/graey/guided-initial-10m.zip. NOT extracted, activated or restarted: repeated fresh telemetry armedMANUAL,ESC-on. User requested overriding delay but no disarm was observed. Existing live system unchanged by this work. Resume with fresh disarm, backup six archive paths/source+installed, install/restartcore, verify profile/onebridge/GUI. Pending homepage route change is outside this archive; do not claim deployed.

## Deployment completed — 2026-10-25
Supersedes staged-only checkpoint above. User confirmed disarmed; fresh telemetry independently confirmed disarmed and no autonomous mission before stopping core. Six archive files installed, colcon build succeeded, all six installed copies verified against source, Python syntax checked. Restarted graey-ros. Source backup: /home/graey/deployment-backups/guided10m-20261024-013358 (Jetson clock naming).
After restart: fresh MANUAL/disarmed telemetry, ESC command off, STARTUP inhibit; exactly one fused_nav_bridge and one navigation_supervisor, no August bridge; configuration_verified=true and profile_mismatches={}. Live /waypoint serves 10 m GUIDED alignment and 1.5 m arrival wording. Bridge disabled/not transmitting and surface_pressure_hpa=0 after restart: Start Navigation at the water surface is still required. No parameter writes, arming, or live movement test performed. Pending gui_node homepage routing change was not deployed.
