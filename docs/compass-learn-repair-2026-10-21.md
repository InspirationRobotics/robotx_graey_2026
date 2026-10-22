# Compass Learn idle/flicker repair — 2026-10-21

Live request reached the bridge but failed: primary `COMPASS_USE=0`, although
acquisition was enabled and device 590114 was detected. The GUI then overwrote
the failed job message with the bridge's idle status. Failures are now shown
persistently in both the operation banner and compass status.

Completed the previously approved calibration-only native GPS exception:
Compass Learn temporarily selects GPS horizontal position/velocity, barometer
height and compass yaw in all source slots. The installed ArduSub learner needs
GPS aiding for its GSF yaw estimator. Odometry/waypoint permission stays withheld.
The script sends no arm, disarm, mode or propulsion commands. Routine start and
passive capture accept armed MANUAL with fresh controller telemetry and no mission.

All original bridge-profile readbacks must match before start. After an actual
firmware completion event, mode-0 readback and two seconds of stable paired
Cube/VN heading, the complete ExternalNav profile is restored and verified.
Cancel also restores the complete profile. Missing source acknowledgements retry;
failure after ten seconds or no completion after ten minutes requests restoration
in MANUAL. No unverified north reference is invented. Operator takeover remains
effective; if MANUAL/telemetry is lost, restoration waits for valid MANUAL control.

Validation: 293 Python tests and both GUI suites pass. Native firmware evidence
`sitl/logs/compass-learning-20261021-213810/result.json` passed: actual completion,
121 paired samples over 2.001 seconds, captured offset -2.40 degrees and all
24 bridge fields restored. Simulation elapsed 13.83 wall seconds, about 41
simulated seconds; this is not a promise of physical calibration time/accuracy.
No QGC forwarding or live endpoint was used for that test.

Deployment requires killed/disarmed/ESC-off for the service restart. The live
selected compass and bridge-depth prerequisite repair is limited to enabling
`COMPASS_USE=1` for verified primary 590114 and restoring `EK3_SRC1_POSZ=6`, with
fresh readbacks and a saved before-state. Learning itself remains operator-started.
