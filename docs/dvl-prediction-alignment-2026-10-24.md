# DVL recovery and ten-metre alignment — 2026-10-24

## Scope and status

Local changes on `tether_toggle_beichen`, based on `27b3dc6`. Not deployed or
physically tested. User requested simulation be skipped in favor of onboard
testing. Teammate is currently using Graey: wait for explicit reconnection and
deployment go-ahead. Tether SSH timed out and graey.local failed to resolve.
Do not assume old killed/disarmed states remain current.

## Changes

- Current fixed-local estimator no longer latches a DVL continuity fault at
  five metres of predicted position uncertainty. Valid measurements reacquire
  automatically after three consistent samples spanning at least 0.2 seconds.
  VN/frame faults remain distinct and are not cleared by returning DVL.
- Bridge sends an odometry packet only for a new, fresh, valid DVL sample after
  reacquisition. It does not send zero velocity or extrapolation as a fresh DVL
  measurement. Local extrapolation retains growing covariance; its five-second
  integration bound remains. GPS map corrections no longer erase uncertainty
  accumulated in unobserved Cube-local displacement.
- Supervisor and mission distinguish unavailable odometry measurements from a
  healthy, fresh Cube estimate. An initialized, enabled, correctly configured
  bridge with intact VN/depth/frame and healthy Cube permits continued travel
  during DVL/GPS gaps. Cube health, stale telemetry, kill, takeover and mission
  limits still apply. No guarantee is made that Cube will remain healthy for an
  arbitrary dropout duration.
- When north is not established, Start Mission runs approximately 10 m along the
  starting local heading in STABILIZE. Nominal forward speed is 0.1 m/s, slowing
  near the endpoint. Fixed-line cross-track position and lateral velocity feedback
  use Cube's DVL-aided estimate; during a gap they use its prediction. Heading is
  held relative to VN. Vertical manual input stays neutral at 500.
- Stops at 9.9 m projected forward displacement (10 m nominal target), samples
  GPS, and accepts only a qualified GPS/DVL fit after the existing two-second
  stopped check. No candidate means remaining stopped, then timing out rather
  than extending the corridor. Alignment allows 180 seconds and at most 12 m
  radial displacement; cross-track departure beyond 1 m aborts.
- DVL gaps clear unaccepted alignment evidence so predicted displacement cannot
  silently serve as measured heading evidence. A gap can therefore prevent this
  run from producing a qualified north fit, without requiring VN recalibration.
- Offset changes only the geographic transform. Cube's local frame and VN
  quaternion remain unchanged; no EKF restart. Existing waypoint metre-by-metre
  checks and arrival behavior remain after alignment. Already-aligned sessions
  skip the initial alignment run. GUI corridor text now states 10 m plus stopping
  clearance.

## Verification

- 44 focused Python tests passed: prediction policy, fixed-line cardinal axes,
  mission step behavior, DVL reacquisition, existing bridge regression,
  STABILIZE neutral vertical control, transport and GUI backend.
- Waypoint JavaScript page checks passed.
- Initial transport test failed because its fixture still expected the old
  dropout deadline and lacked the fixed-local frame. Updated the fixture to the
  new policy and verified that an unhealthy Cube still rejects continuation.
- Simulation and native SITL deliberately not run at user request. No claim of
  physical heading accuracy, straightness, or full mission success.

## Deployment handoff

1. Wait for user: teammate currently operating vehicle. Reconnect over tether.
2. Verify fresh disarm, physical kill and ESC-off before replacing/restarting
   control software. Back up actual source, installed files and service config.
3. Inspect live changes before updating: last documented live base is d670267
   with targeted GUI/Prepare patches, not this branch's full STABILIZE code.
   Preserve teammate changes; do not blindly overwrite or reset the checkout.
4. Install/build current source, verify only one bridge, confirm STABILIZE mission
   transport and GUI assets, then restart core. No firmware or parameter changes
   are included in this patch.
5. Check fresh Cube/VN/DVL/GPS/pressure and ExternalNav profile; Prepare only with
   confirmed water-surface baseline. Verify zero vertical command and that sensor
   reacquisition does not change Cube frame or clear unrelated faults.
6. Operator-supervised moving test needs a clear 10 m corridor plus stopping
   distance and effective kill/takeover. Record local position/velocity, DVL
   validity, accepted GPS track, Cube health, commands, north fit and mission
   transitions. Do not inject hardware sensor failures during powered motion.

Outstanding: physical performance, measured packet cadence, Cube prediction and
reacquisition on actual firmware, and full mission completion. Previously noted
temporary battery prearm exception remains a separate issue; this change does
not modify it. Preserve unrelated `docs/nav_lgoic.md`.

## Deployment completed — 2026-10-24

User authorized overwrite and restart with fresh disarm verification despite kill
released/ESC initially enabled. First guarded attempt detected re-arming and made
no source/restart changes (a subsequent build of unchanged source completed).
After renewed disarm confirmation, installed 29 files over teammate merge
366a94fd without resetting its branch or touching unrelated files. Backup:
/home/graey/deployment-backups/dvl-align-20261024-050918.
Build passed. First restart found obsolete tools/august_bridge.py PID41704 holding
navigation ownership; stopped that exact process after fresh disarm check and
restarted core. Verified one fused bridge and one supervisor, source hashes and
served ten-metre GUI wording. 44 onboard tests pass using unittest discovery;
initial dotted test imports failed due to the onboard test package namespace.

With separate user approval, restored GPS1_TYPE0->1 and EK3_SRC1_POSZ1->6 via
MAVProxy UDP14554; both PARAM_VALUE readbacks verified. Before/after record:
/root/robotx_ws/logs/dvl-align-params-20261024.json. No firmware changes.
Latest read-only check: profile verified/no mismatches, VN/DVL inputs fresh,
bridge disabled with pressure baseline0, GPS messages fix0/time0/coordinates0.
Prepare and a usable GPS fix are still required; no live mission or moving test
has been performed. User intends to arm manually, so no further live parameter
writes/reboots/restarts are authorized by this handoff. Core startup inhibit
remained in last observation; do not automatically clear it or arm the vehicle.
Local changes remain uncommitted/unpushed; deployment does not imply GitHub sync.
