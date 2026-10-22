# Automatic north capture during manual compass learning

User clarified that the problem was the disarm requirement for capturing north,
not driving in MANUAL. This revision separates passive reference capture from
maintenance-only parameter writes. No vehicle access/deployment in this session.

## Behavior

1. Initial setup remains killed/disarmed/ESC-off because it temporarily configures
   Cube compass yaw and COMPASS_LEARN=3. Values1/2 of COMPASS_LEARN are not used.
   EK3_SRCx_YAW=1 is a separate parameter meaning compass yaw; it is not deprecated
   Compass Learn mode1. The compass must be identified, enabled and healthy.
2. The operator drives in armed MANUAL. Learning does not require staying disarmed.
3. After actual learning-start readback, firmware Finished, learning readback0 and
   two seconds of valid paired independent Cube/VN offsets, CubeNorth automatically
   records the circular mean north offset. It has no arm/disarm requirement for
   this passive capture. It sends no control or parameter commands during capture.
4. GUI shows captured state, offset and the current VN heading with that reference.
   No manual bearing entry or Capture button is needed in the normal workflow.
5. After operator stops, disarms, engages kill and disables ESC power, the bridge
   automatically restores all ExternalNav yaw sources6 and learning0. Fresh control
   status and an independent maintenance check precede every write batch. Writes
   retry at most once per second pending acknowledgements; ready requires exact
   fresh readback. It never disarms or switches yaw sources while the pilot drives.
6. Floating-surface Prepare still establishes pressure zero and commissions fusion.
   Capturing north does not establish a water-depth baseline or arm a mission.

Valid compass/GPS, freshness, actual completion, independent yaw and agreement
checks remain: removing them would claim a north reference without evidence.
VN gaps/recalibration invalidate a pending reference. Compass setup with
COMPASS_ENABLE=0 cannot be bypassed by merely setting COMPASS_LEARN=3.

## Validation

Regression tests cover automatic capture without an armed/disarmed argument,
armed-control suppression of parameter writes, subsequent automatic safe
restoration, retransmission pending readback, and no propulsion/mode commands.
Existing learning/source/invalid-sample checks retained. Both GUI test suites pass.
No new native Compass Learn convergence or physical north accuracy claimed.
Ordinary mission/vertical controller unchanged from the previous validated release.

Final result: 266 Python tests and both GUI suites passed. No native SITL rerun for this isolated capture/maintenance change; native Compass Learn support still requires commissioning.
