# M9N compass assessment — 2026-10-17

## Scope and verified acquisition

User authorized stationary compass/VN sampling and conditional surface alignment
integration. Tether SSH worked. Fresh controller checks confirmed disarmed, kill
engaged, ESC command off, no mission. No propulsion was commanded.

Initially `COMPASS_ENABLE=0`; 65 seconds of SCALED_IMU/2/3 contained zero magnetic
vectors. Saved IDs in that state did **not** prove the external compass absent.
After explicit user approval, enabled acquisition (`COMPASS_ENABLE=1`) and rebooted
Cube. All `COMPASS_USE*` remained zero and every EKF source set remained the existing
ExternalNav profile, including yaw=6 and vertical position=6.

After reboot, compass 2 was detected as device 658945 (IST8310), marked external.
Compass 1 remains device 590114 (AK09916), marked internal. Driver/type decoding
uses ArduPilot AP_HAL DeviceStructure and AP_Compass_Backend device definitions.
External compass upright/forward mounting was subsequently confirmed by the user.
No physical wiring inspection was performed.

## Stationary survey before compass calibration

65.13-second acquisition-enabled survey, disarmed throughout:

| Signal | Samples | Level heading std | Level heading peak-to-peak | Field magnitude |
|---|---:|---:|---:|---:|
| External IST8310 / SCALED_IMU2 | 630 | 0.164° | 0.804° | 750.9 mG mean, 1.17 mG std |
| Internal / SCALED_IMU | 648 | 0.184° | 0.977° | 2243.6 mG mean, 5.65 mG std |
| VN ROS orientation yaw | 2500 | 0.512° | 1.807° | Not measured |

These magnetic headings use atan2(-Y,X) without tilt or declination correction:
variation statistics are useful, absolute values are **not** commissioned north.
VN changed from 167.681° to 165.878°; this observation alone cannot separate motion
from drift. The earlier acquisition-disabled sample had 11.282° VN variation.
No independent physical heading was supplied for either sample.

The external compass was at identity calibration: offsets/off-diagonals zero,
diagonals one, scale zero, orientation zero. Automatic declination readback was
0.19117327 rad (~10.95°). Low noise cannot establish low magnetic bias. Field
magnitudes and absent calibration warrant calibration before alignment acceptance.
No powered-thruster interference test has been performed.

## Calibration results and unresolved north validation

User confirmed upright/forward mounting and readiness to rotate the assembled
vehicle. Started external-only calibration with mask 2, autosave disabled, no
autoreboot. ArduPilot requires `use_for_yaw` for calibration, so `COMPASS_USE2` is
temporarily one during this explicitly authorized operation; helper restores zero
in a finally block. EKF yaw remains ExternalNav.

First attempt timed out at 240 seconds and 78% coverage; cancelled and USE2
restored zero. Retried with a 900-second allowance. Second attempt returned
MAG_CAL_SUCCESS (4), fitness 13.702692 mG, orientation confidence 4.5713,
original orientation 0, proposed orientation 6 (yaw 270 degrees).
The helper incorrectly read `orientation` instead of `new_orientation`, threw
before its explicit ACCEPT command, and restored USE2=0 in its finally block.
The local helper now checks old/new orientation and rejects an unverified change.

**Correction:** despite autosave=0 and the helper's exception, readback showed
the fit AND orientation 6 persisted. Inspection of installed MAVProxy explains
this: its calibration module's MAG_CAL_REPORT handler automatically sends
DO_ACCEPT_MAG_CAL independently of the helper. Do not claim the fit stayed unsaved.
Before another conditional calibration, control this auto-accept path through a
scoped supported module configuration, preserving MAVProxy as sole serial owner.

Saved calibration: offsets (97.39735, -66.07222, -426.47552), diagonals
(0.9889923, 1.0450687, 1.0447650), off-diagonals (0.00561238, 0.02085272,
0.09265845), scale 1.0611719, orientation 6. These affect compass acquisition;
compass EKF use is disabled. User confirmed upright/forward arrow, so the mounting
discrepancy remains unresolved. No extra heading offset was applied.

User corrected the independent heading from 256 to **356 degrees true**. A fresh
15-second sample gave approximately 335.3 degrees after declination (without tilt),
field 474.0 mG and angular std 0.315 degrees. A subsequent time-matched VN tilt
calculation, removing VN yaw, gave **337.8 degrees true**, std 1.03 degrees,
peak-to-peak 3.76 degrees. The approximate 18-degree difference does not pass north
commissioning. Motion/time correspondence and phone accuracy remain uncertain;
VN yaw also changed during these windows. Do not attribute all variation to drift.

Compass-to-bridge integration is **not deployed or enabled**. Static noise was
promising but north validation did not pass. No mission/GUI behavior changed.
A local integration sketch was removed from runtime sources when this gate
failed; its patch remains under workspace output, not as tested software.
Final readback confirms ENABLE=1, all USE*=0, complete ExternalNav profile
unchanged, killed/disarmed and ESC-off.

## Diagnostic failure retained

First acquisition parameter write succeeded, but batched profile verification
dropped the last three parameter replies and stopped before reboot. Separate
readback confirmed ENABLE=1 and USE*=0. Pacing requests by 20 ms fixed verification;
complete profile passed before the approved reboot and again afterward.

## Evidence and next gate

Compact statistics/readbacks: `evidence/m9n-compass-survey-2026-10-17.json`.
Full raw samples remain under workspace `output/compass-survey-20261017.json` and
`output/compass-postreboot-survey-20261017.json`. Read-only/acquisition/calibration
helpers are retained in workspace output and Jetson workspace logs.

Before enabling compass-based surface alignment: successful calibration and
readback, post-calibration stationary/rotation validation, tilt compensation from
VN roll/pitch, true-north declination, explicit device/frame/freshness gates, and
surface-only capture frozen for VN propagation underwater. Keep GPS/DVL alignment
as a fallback/cross-check. A quiet stationary sample cannot certify motor EMI.

Sources:
- https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_HAL/Device.h
- https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Compass/AP_Compass_Backend.h
- https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Compass/AP_Compass_Calibration.cpp
- https://docs.holybro.com/gps-and-rtk-system/m8n-m9n-m10-gps/micro-m10-m9n-gps/pinout
