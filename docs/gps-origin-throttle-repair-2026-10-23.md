# 4.7.2 saved-origin and MANUAL throttle repair — 2026-10-23

Live report: mission says prepared local navigation/qualified GPS required; QGC shows a baseball field; arming MANUAL causes upward thrust. No physical arming/thrust test performed by the agent.

## Verified causes and repair

- GPS_RAW_INT was fresh near32.92363,-117.03851, while GLOBAL_POSITION_INT/GPS_GLOBAL_ORIGIN were near32.92440,-117.02010 (about1.7km away). AHRS_OPTIONS had migrated from0 to the new Sub default16, enabling reuse of stored non-GPS origin; the stored AHRS_ORIGIN coordinates were the wrong site. All EKF source slots remained ExternalNav6 as intended. Restored AHRS_OPTIONS=0, preserving stored coordinates but disabling automatic reuse. After safe reboot, GLOBAL_POSITION_INT and raw GPS matched exactly in the captured sample (32.9236352,-117.0385266). The checked bridge profile now requires AHRS_OPTIONS=0 to catch recurrence.
- RC3_TRIM was1100, with RC3_MIN1100/MAX1900 and physical neutral input1492. Exact4.7.2 ModeManual uses (channel_throttle.norm_input()+1)/2; norm_input uses trim. This yields about0.745 throttle vs neutral0.5, explaining positive thrust from nominally centered input. Set RC3_TRIM=1500 with readback verification. At1492 the same formula is0.49; at1500 it is0.5. Small radio centering error may still require calibration; no armed test was conducted.
- Fresh killed/disarmed/ESC-off status checked before stopping core, writing those two parameters and rebooting. No other parameter changed. Prearm check accepted and no STATUSTEXT error in postreboot sample. Core reset means surface Prepare must be repeated.

The mission start error is a combined admission check, not proof of failed GPS receiver configuration. Live Prepare had completed and bridge GPS admission was true at inspection, but a prior last-rejection cited stale Cube fix timing. Split the API error into specific local-navigation or GPS-admission reasons without relaxing health gates. Map/north registration remains part of the initial mission movement; a missing north transform before that movement is expected.

Other audit findings retained for separate review: COMPASS_USE1 remains enabled from earlier calibration work, but all yaw source slots are ExternalNav6 and COMPASS_LEARN0. MNT1_TYPE changed0->1 and ATC_INPUT_TC0.15->0.10 across firmware defaults. These were not changed speculatively. GPS serial4 protocol5 and GPS1_TYPE1 correctly acquire fresh fixes; serial baud230 is working. No direct GPS fusion was enabled.

Evidence on Jetson /root/robotx_ws/logs: 472-gps-parameter-audit.json,472-control-gps-audit.json,472-gps-control-repair.json,472-gps-control-postreboot.json. Pre-change GUI snapshot in deployment-backups/20261023-ardusub472/gps-control-before-repair.json. Earlier panic remains unresolved as a root cause despite clearing on reboot; do not bypass arming checks.
