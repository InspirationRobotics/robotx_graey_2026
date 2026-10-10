# GRAEY session report — October 9, 2026

Session date is Pacific time; capture filenames use UTC October 10. Report completed October 10. All vehicle status below is historical, not a claim about current status.

## Outcome

Removed duplicate navigation processes and verified fresh valid DVL telemetry. Manual pool tests showed consistent DVL/Cube relative movement and close VN/Cube heading agreement. Absolute geographic position, true heading, and live source switching remain unverified. Supervisor remained in Observation/Fault; no autonomous mission was run.

## What we did

- Connected to Jetson through tether at 192.168.2.2. MAVProxy remained the sole Cube serial owner.
- Inspected deployed processes, driver code and selected Cube parameters. Live checkout was `tether_toggle_beichen`, HEAD `1e09b328576dfb8c5341fe10daab1c23e3e92ade`, older than local hardening work. No current GitHub synchronization was established.
- Captured stationary GPS: 60 samples over 62.8 seconds, scatter 0.423 m north and 1.215 m east, reported horizontal accuracy 1.442–1.481 m. Scatter is repeatability, not surveyed accuracy.
- Compared approximate 90-degree rotations. Initially VN and Cube differed by roughly 180 degrees while agreeing on rotation increments.
- With permission, removed GUI-owned duplicate navigation processes and temporarily restarted VN with a +180-degree heading offset. Restored installed driver source byte-for-byte afterward. This offset was runtime-only, not permanent mounting calibration, and must not be assumed to survive later restarts.
- Duplicates recurred: GUI and core launch each ran actual VN, DVL and bridge executables. After fresh checks confirmed disarmed, kill engaged and ESC commanded off, removed GUI VN24743/DVL24741/bridge24742. Retained core VN24767/DVL24769/bridge24765 without restarting them; verified one executable per node.
- DVL initially remained stale despite Ethernet reachability and an established TCP16171 connection. Following the user's correction and cleanup, fresh valid reports returned. Exact cause of the earlier interruption was not established.
- Captured two analyzed manual movement tests. Several capture attempts aborted at safety guards; another was superseded at user request. These are not additional successful validation runs.

## Pool measurements

Evidence is under workspace `output/dry-land-20261009/`.

| Measurement | First trial | Repeat trial |
| --- | --- | --- |
| File | `manual-20261010T044222Z.jsonl` | `manual-20261010T045125Z.jsonl` |
| Duration / samples | 64.7 s / 222 | 64.9 s / 222 |
| Fresh valid DVL samples | 219/222 | 222/222 |
| Integrated horizontal travel | About 1.15 m | About 1.53 m |
| DVL estimated net movement | Not used for calibration | About 1.29 m |
| Cube local net movement | About 0.68 m | About 1.30 m |
| VN minus Cube heading | -0.83 to +1.14 degrees | -0.98 to +1.23 degrees |
| Heading change | About 27 degrees net | About 15 degrees range |

Repeat DVL displacement, rotating polled body velocities using VN heading, was approximately north -1.272 m and east -0.214 m. Cube local displacement was north -1.281 m and east -0.236 m. Raw GPS shifted north -0.100 m and east +1.936 m over the same capture, substantially different from the relative motion.

These are approximate GUI/API samples, not atomic high-rate sensor logs. Rotation and lack of independently measured exact endpoints prevent scale calibration. DVL may be feeding Cube, so agreement is not an independent accuracy check. Do not tune DVL scale from these runs. Close heading agreement does not establish true north; good relative movement does not establish correct map placement.

## EKF and supervisor

Earlier selected parameter readback showed EK3 enabled; SRC1 horizontal position, horizontal velocity and yaw each 6 (ExternalNav); SRC2 corresponding values each 0; SRC_OPTIONS=1; SCR_ENABLE=0. This is configuration, not selected-source or fusion proof. No NAV_SRC feedback was observed in the earlier readback.

Later pool captures contained fresh Cube local position and GUI-reported healthy global position. Supervisor still reported Observation/Fault with reason `Navigation bridge restarted; external frame must be revalidated`, confirmed source Unknown, configuration unverified and navigation not ready. No supervisor fault was cleared, origin reset, source set commissioned or Cube parameter changed during the live session.

Local source-switching work and isolated SITL results are documented separately in `source-switching-session-2026-10-08.md`. Those results do not establish vehicle deployment or physical accuracy. No new SITL run was performed for these pool measurements.

## Remaining checklist, in recommended order

- [ ] **Permanently resolve GUI/core duplicate ownership.** Both can still start navigation bridges on deployed code, and also duplicate VN/DVL. Review existing local ownership hardening against deployed code, then deploy with authorization. GUI must not spawn competing owners. Test repeated clicks, GUI restart, core startup and recovery; verify exactly one executable and one external-navigation publisher. Today's process cleanup is temporary.
- [ ] **Verify current VN configuration and mounting.** Read runtime offset after the intervening restart; validate forward/right/down axes, quaternion convention, rates and acceleration transformation. Do not permanently apply +180 degrees solely to match Cube.
- [ ] **Validate relative motion against measured endpoints.** Repeat straight forward/backward and lateral movements with heading held fixed, recording fresh sensor timestamps and invalid intervals. Check axis signs, rotation, scale and stationary drift before tuning.
- [ ] **Get a complete current Cube DataFlash .bin log and full parameter backup.** Collect matching Jetson logs, firmware version and event times. Diagnose actual yaw/position fusion, innovations, resets and source transitions. The selected parameter readback is not a full backup.
- [ ] **Commission underwater/surface source sets and feedback.** Review firmware-specific semantics and bridge compatibility. Verify selected-source reporting and command acknowledgement, not just configured parameters. GPS recovery must not imply arm, fault reset or ESC restoration.
- [ ] **Revalidate the navigation frame, then recover the supervisor deliberately.** Establish one healthy bridge, consistent frame/reference and verified configuration before clearing its latched fault. Do not reset just to make the GUI green.
- [ ] **Validate calibrated depth and surface detection.** Check pressure reference, data freshness, hysteresis and dwell. Latest supervisor depth/surface state was unknown. Test bobbing and prevent repeated switching.
- [ ] **Diagnose absolute map offset.** Compare raw GPS, Cube global/local position and origin over time against an independent reference. Separate image registration error, receiver bias and estimator error. QGC appearance alone does not diagnose GPS fusion.
- [ ] **Tune GPS acceptance from logs.** Evaluate freshness, uncertainty, innovations, sustained quality and disagreement with DVL motion. Test jumps, stale data, dropout, recovery and continuity both ways. Switching cannot repair wrong heading or map registration.
- [ ] **Clarify the navigation GUI/map.** Distinguish raw GPS, Cube estimate, mission reference and pool registration; display freshness, uncertainty and live/SITL identity. Preserve pool scale and verify geographic anchor/north alignment. Keep raw GPS accessible alongside overall navigation.
- [ ] **Fix/check kill-switch and control behavior.** Verify RadioMaster SA, RF availability, latched inhibit and ESC commanded state against actual behavior. Explain released input versus latched kill clearly. Resolve earlier neutral-control/uncommanded-thruster concerns before powered testing.
- [ ] **Reconcile local repository, remote branch and Jetson.** Record exact revisions and dirty files, preserve existing changes, and obtain current authorization before pushing/deploying. Cached upstream counts are not synchronization proof; keep main unchanged unless authorized.
- [ ] **Progress to powered and waypoint trials only after the above prerequisites.** Verify neutral controls and thruster response, then bounded manual driving, then dive/travel/surface and source recovery with validated navigation readiness.

## Changes and evidence

Workspace `output/dry-land-20261009/` contains the baseline, selected parameters, diagnostic scripts, captures and its detailed README. `GRAEY_CONTEXT.md` retains dated findings. Existing local source edits were preserved. This report does not claim a commit, push, main change, permanent duplicate-prevention deployment, or numeric live EKF tuning.
