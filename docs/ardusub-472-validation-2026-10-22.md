# Official ArduSub 4.7.2 compatibility tests

No physical firmware, vehicle parameters, or Jetson deployment changed.
Candidate: official `Sub/stable-4.7.2/sitl/ardusub`, source
`fb773567896014da007bca58358704d4a0d8516f`, executable SHA256
`454abc3e16857f48fc43db63c82f5033569a7b3996aa9e55948053d9a40a6170`.

The candidate source includes the upstream neutral-throttle repair. That alone
does not establish a successful Graey mission or physical descent diagnosis.

## Compatibility changes under test

- `GPS1_TYPE/GPS2_TYPE` replace `GPS_TYPE/GPS_TYPE2`.
- `WP_SPD_DN=.10`, `WP_SPD_UP=.15`, `WP_ACC_Z=.50` replace the centimetre-based
  WPNAV limits. No enlargement of physical limits.
- Readback comparisons tolerate float32 roundoff (e.g. 0.10000000149).
  A materially excessive limit such as 0.11 versus 0.10 still fails.
- `GRAEY_FIRMWARE_PROFILE=4.7` is explicitly restricted to localhost SITL,
  ROS domain73 and `GRAEY_RUNTIME_SCOPE=sitl`. Live/default remains4.5.

Three focused compatibility/transport tests pass. These verify old defaults,
isolation, renamed/scaled limits, float32 acceptance and oversized-limit rejection.

## Run evidence, including failures

- `waypoint-d477518a46`: Windows-mounted source stalled during mission process
  startup; deadline expired. No control validation obtained.
- `waypoint-f9bdf0b815`: temporary Linux copy interrupted by WSL shutdown.
  Temporary logs disappeared after reboot; result inconclusive, never a pass.
- `waypoint-4c37afb026`: persistent Linux run reached WAIT_SURFACE but failed
  limit readbacks. Float32 precision bug subsequently fixed. This copy also
  included concurrently edited mission behavior; do not use it as the pinned
  baseline comparison.
- `waypoint-0a705b1cd0`: pinned committed baseline94c7b35 plus compatibility
  edits reached GUIDED, ARM, CHECK_SURFACE and ALIGN_MOVE. Aborted after about
  0.6m when configuration verification became unavailable. Pressure was0m at
  abort; this is not a completed waypoint validation.

Subsequent diagnostic run uses the same pinned copy with configuration-readback
logging only; no relaxed admission or depth limits. Result recorded below.

- `waypoint-82146466f0`: failed on expired `COMPASS_LEARN=0` readback, age just
  over3s. Value was correct. Parameter-reply starvation is a transport issue,
  not a changed EKF profile. Kept the3s check; replaced16-request bursts once
  per second with4 requests every200ms (full cycle about1.4s).
- `waypoint-48982f704e`: **PASS**, complete initial north alignment, transit,
  arrival, stop and disarm on official4.7.2. Estimated final target error0.135m;
  maximum sampled pressure depth0.030m. No configuration-readback-loss diagnostic.
  Simulated GPS/DVL data and benign water model are not physical accuracy or
  thrust validation. The0.08m depth abort was not relaxed.

Unrelated workspace edits (heading, segment navigation and GUI) were preserved.
SITL QGC forwarding disabled; physical vehicle never controlled by these tests.

Final validation:
- `waypoint-1569ce2763`: PASS, GUI Abort during transit confirmed MANUAL takeover.
- Final pinned-baseline regression: 297 tests, 2 skipped, zero failures/errors (9.320 s). Initial regression had three obsolete polling-fixture failures and a diagnostic-only logger mock error; updated the bounded-queue test for the production 200 ms cadence and removed temporary diagnostic instrumentation. No production health threshold was relaxed.
- Evidence copied to workspace `output/ardusub472-evidence.tar.gz`, including failed runs and regression logs.

Conclusion: official 4.7.2 is a promising tested upgrade candidate for baseline94c7b35 plus the compatibility fixes. The native full mission and Abort checks pass. This is not firmware flashing authorization or physical validation. Concurrent segmented-mission changes are excluded from this pinned test and require integration testing separately. The 4.7 profile remains restricted to isolated SITL; live default remains4.5. Before vehicle deployment, review the actual parameter migration, back up parameters/firmware, explicitly approve flashing, and verify sensors, frame conventions, kill/takeover and neutral vertical behavior on the vehicle.

