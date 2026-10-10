# Wi-Fi navigation deployment and offset investigation — October 10, 2026

User authorized deployment, repository/Jetson synchronization and investigation of suspected offset corruption; user reports VN-100 hardware is verified good.

## Deployment

- Connected through `graey.local`. Pre-deployment controller confirmed fresh disarmed MANUAL, ESC commanded off, no mission, STARTUP inhibit. No arming, inhibit recovery or Cube parameter writes were performed.
- Jetson checkout was `1e09b32`, four commits behind `d1dfbeb` at initial fetch. GUI/core duplicate VN, DVL and bridge executables had recurred.
- Backed up tracked GUI changes, status and installed package under `/home/graey/deployment-backups/20261010-nav/`. Kept a Git stash of the tracked GUI change, fast-forwarded, and reapplied it cleanly. Local sonar tab and untracked sonar work were preserved. These unrelated files remain Jetson-local; a clean identical whole checkout is not claimed.
- Stopped the core stack using a temporary scoped systemd stop override rather than the deployed broad `pkill -f robotx_graey_2026`; signaled identified leftover GUI-owned navigation children. Removed the temporary override afterward. MAVProxy remained running and the sole Cube serial owner. No intentional sonar/pose process stop was issued.
- Built with colcon and restarted core. This restarts the external-navigation frame; no claim of mission-reference continuity is made. Supervisor remains observation-only, readiness false. Its post-start reason was `Depth reference unavailable`, with source Unknown and configuration unverified. Prior in-memory bridge fault is no longer the current status after this authorized whole-stack restart; it was not evidence of a validated frame recovery.
- Final executable revision `87e8788` includes source-switch confirmation work, GUI duplicate rejection, Linux ownership locks and startup-only offset settings. A subsequent documentation-only commit records this deployment.

## Offset findings

The previous temporary +180-degree offset was not made persistent. Prior to deployment a responding VN node reported 0 degrees and flip=true; bridge reported 0 degrees, but duplicate names made that readback ambiguous. After deployment, exactly one node of each type was verified and the readback was unambiguous: VN yaw offset=0, flip_180=true; bridge yaw offset=0. Fresh post-start VN/Cube heading was approximately25.16/25.01 degrees. This is agreement, not independent true-heading calibration.

Found a real software consistency bug: offsets and VN flip are cached during construction, but parameters previously accepted runtime writes without updating those cached values. Such a write could make ROS readback disagree with the transform actually published. Fixed by marking those parameters read-only after startup; startup overrides remain supported. No physical mounting transform or offset value was changed in this deployment.

Initial protection reused a mutable ROS parameter descriptor. Live DescribeParameters exposed an incorrect flip name/type; corrected by allocating a separate descriptor per parameter and added regression assertions. Final live readback: yaw descriptor DOUBLE/read-only, flip descriptor BOOL/read-only. This metadata issue did not change the selected offset value.

Duplicate publishers and competing serial readers were confirmed. Their contribution to earlier heading anomalies is plausible but not proven by this inspection. No claim of VN hardware failure, demonstrated serial-byte corruption or fully validated body-frame mounting is made. Full axis/heading validation remains outstanding. The ROS CLI daemon also returned `!rclpy.ok()` during initial parameter queries; direct service clients bypassed that diagnostic failure without writing sensor settings.

## Verification

- Colcon builds succeeded (byte-compilation-disabled warnings only).
- Final complete Jetson test suite: **97 tests passed**, including isolated ROS frame-parameter tests with serial/MAVLink mocked and Linux ownership tests. ROS tests used separate domain93/local-only; no SITL or autonomous live mission was run here.
- Installed GUI, VN, bridge, supervisor, supervisor logic and process ownership files matched their source after the first build; the final VN metadata correction was rebuilt and verified through live descriptor readback.
- HTTP `/api/start?group=nav` and `/api/stop?group=nav` both returned409, without process actions. GUI groups report managed navigation with one instance each.
- Final PIDs: VN21919, DVL21246, bridge21250, supervisor21252. These are historical evidence, not identifiers to reuse for future process actions.
- Fresh valid DVL and fresh VN data observed after deployment. Controller remained disarmed with ESC commanded off at the post-start check.

## Remaining

Depth reference, selected-source feedback, live source-set configuration and frame validation must be commissioned before enabling supervision. Obtain full Cube logs and parameter backup for fusion/GPS analysis. Preserve local sonar work during future synchronization. The original service's broad stop command remains a separate maintenance concern; this deployment avoided it with a temporary scoped override.

## Final concurrent-work reconciliation

During publication, remote branch received the sonar team's merge. Preserved it with a normal merge (`7160cab`), then fast-forwarded Jetson. All incoming sonar files already present on Jetson were byte-identical to the remote versions; backed up/stashed overlaps before fast-forward. Final tracked Jetson diff was empty, navigation installed files matched source and installed GUI matched the merged GUI. Untracked backups/data remain preserved. This supersedes the earlier statement that the sonar additions remain only local.

Sonar commit `2c007c8` records a team tilt test: nose-down produced positive VN pitch and negative Cube pitch, with roll agreeing. Its downstream sonar code compensates pitch via `VN_PITCH_BACKWARDS`. This is reported team evidence, not a test repeated in this session. A future upstream VN mounting-transform correction must coordinate removal of downstream compensation to avoid double correction. The yaw-offset consistency fix does not resolve this separate pitch issue.
