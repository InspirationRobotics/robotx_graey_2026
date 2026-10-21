# Parameter readback starvation fix — 2026-10-20

The repeated full parameter burst could overflow the available reply processing
and continually miss the tail fields. `FusedNavBridge.request()` now deduplicates
the required navigation/compass names and rotates batches of at most 16 per
one-second poll. A schema change restarts the schedule; all current fields are
requested over two ticks, within the existing three-second freshness requirement.
No parameter writes, source changes or control actions were added.

Validation: bounded-reply-queue regressions and the full 289-test Python suite
pass. `sitl/validate_parameter_polling.py` exercised the production request method
against isolated, unarmed ArduSub on loopback, without ROS, MAVProxy or QGC.
Evidence: `sitl/logs/parameter-polling-20261020-184521/result.json` (passed).
All 24 navigation and 28 combined compass/navigation fields refreshed; maximum
observed field ages were 2.001 seconds or less. This validates readback delivery,
not compass-learning convergence or the deferred temporary GPS calibration profile.

Earlier execution was blocked by tool approval authentication; the subsequent
authorized run completed. No live vehicle access/deployment was performed.
