# Navigation deployment — October 17, 2026

User requested deployment of today's code. Connected through `graey.local`.
Jetson clock/logs reported October 13; this record uses the user's October 17
session date and preserves the actual backup directory name.

Initial Cube telemetry was unavailable: MAVProxy's serial reader had died with
SerialException while systemd remained active. User confirmed physical kill,
disarm and powered Cube. Restarted MAVProxy only, then fresh telemetry confirmed
kill engaged, disarmed, ESC-off and no mission before stopping navigation.
No second serial owner or live parameter writes were introduced.

Jetson HEAD `d2d1e4a` contained newer pipeline-follow, sonar and vision commits.
Exported its history, merged it locally into `tether_toggle_beichen` without
conflicts as `b3b830f`, and passed 205 local regressions. Preserved all Jetson
untracked files and the user's local `docs/nav_lgoic.md`. Main was not changed.

Backup: `/home/graey/deployment-backups/20261017-navigation-203454`, including
pre-deployment Git bundle and controller/navigation snapshot. Installed-runtime
archive: `/root/robotx_ws/logs/installed-before-navigation-20261017.tgz`.

Stopped `graey-ros`, fast-forwarded Jetson to `b3b830f`, built successfully and
verified tracked Python/GUI files against installed copies. Build again left
some installed files stale; explicitly refreshed differing copies (including
teammate pipeline/sonar modules) and checked byte equality. New north_maneuver
entrypoint exists. Ran **205 tests successfully against installed runtime from
/tmp**, then restarted `graey-ros`.

Post-deployment verification:

- Fresh telemetry: killed, disarmed, ESC-off, no mission, RF selected.
- Exactly one continuous bridge plus expected supervisor, DVL, VN and GUI nodes.
- `/calibration` serves automatic north alignment; `/waypoint` serves pause UI.
- Live VN `flip_180=true`, `yaw_offset_deg=0.0`. No new mounting correction was
  implemented/deployed; the mounting-transform correctness remains to verify.
- Restart cleared session pressure/north preparation: pressure baseline0,
  north unaligned, bridge disabled and supervisor Observation. VN input fresh;
  DVL input invalid at inspection. No floating-surface assumption was made.
- One Cube profile mismatch: `EK3_SRC1_POSZ` actual1, expected6. Left unchanged:
  code deployment does not authorize a separate live parameter adjustment.
  Bridge correctly withholds output; Prepare will not pass until reconciled.

Evidence: `docs/evidence/deployment-2026-10-17.json`. This is a deployment record,
not a wet commissioning or physical waypoint success claim. No mission, automatic
alignment movement, pressure calibration, magnetic calibration or arming started.

Next: reconcile the live vertical source with the agreed bridge-pressure profile,
establish valid bottom lock at the floating surface, run Prepare/north alignment,
verify turn direction and heading stability, then supervised short waypoint test.
Do not treat code installation alone as mission readiness.
