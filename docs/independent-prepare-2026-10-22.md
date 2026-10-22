# Independent Prepare and north alignment — 2026-10-22

Prepare now completes its surface sensor reset without waiting for the movement script. It withholds waypoint permission if north has not been established. With valid continuous VN north, Prepare retains both alignment and heading bias and separately verifies Cube fusion. Lost VN continuity invalidates retention.

The 5 m alignment has its own start, cancellation, status watchdog and completion job. It requires a prepared pressure baseline and qualified sensors, but no running Prepare job. Completion never automatically commissions navigation. Disarm/kill then click Prepare to verify fusion.

DVL remains the metric motion reference. GPS track fit now tolerates scale 0.5–1.5 and up to 2 m RMS residual, versus 0.75–1.25 and 0.75 m. Reported angular uncertainty limit is 25 degrees versus 12. This is a provisional tolerance, not a claim of accurate north; shorter tracks intrinsically weaken the estimate. No GPS-based scaling of DVL distance. Existing fresh-fix, jump, surface, valid-DVL/VN, tilt, rate, controlled stop and travel limits remain.

Focused validation: 24 job tests, 19 bridge adapter tests, 5 track tests and calibration GUI test pass. No full SITL rerun. Physical alignment accuracy still requires verification.
