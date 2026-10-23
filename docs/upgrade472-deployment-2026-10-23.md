# ArduSub 4.7.2 deployment — 2026-10-23

User explicitly authorized flashing, updating and applying the new mission corrections. Tether SSH and fresh MANUAL/disarmed/physical-kill-engaged/ESC-off state were verified before stopping navigation. No arming or physical mission was performed.

## Software validation and changes

- Combined regression: 301 tests, two skipped, zero failures.
- Native 4.7.2/ROS/GUI run `waypoint-1686167fac` passed alignment and completion, estimated error1.282m within the requested1.5m radius.
- Longer run `waypoint-3745e1e24a` completed multiple TRANSIT_CHECK stages but failed its harness outcome because the independent supervisor heartbeat snapshot still said armed after the mission snapshot reported disarmed. Preserved as a failed test. Harness now waits for observed disarm; it does not waive the disarm requirement.
- Repeat `waypoint-65065ca3f5` passed multiple alignment/transit checks, arrival, stop and disarm, estimated error1.253m.
- Runtime commit a1dbd13 includes firmware parameter names/units, float32 tolerance, paced parameter polling, one-metre movement/check segments, stopped heading updates and1.5m arrival. The explicit GRAEY_FIRMWARE_PROFILE=4.7 selection now supports deployment; default remains4.5 for old installations.

## Firmware and deployment

Official CubeOrange4.7.2 firmware commit fb773567896014da007bca58358704d4a0d8516f. APJ SHA25632aebe5b675da1a6f10dd9d22b837a288a7cd25770d8b158e26cab9ce0f69954. Uploader verified board140, programmed and verified CRC, then rebooted successfully. One initial programming attempt retried; the final upload verified and returned0. A surviving MAVProxy daemon was found after service stop and explicitly terminated before restarting a single managed instance. This should be considered when doing future maintenance: service-stop alone was insufficient to prove serial ownership released.

Complete preflash backup989 parameters and postflash1054 parameters saved. Live AUTOPILOT_VERSION reports67568383 (4.7.2), custom hash fb773567. Verified28 external-navigation/profile/shallow-limit values; automatic migration already matched all of them, so no repair writes were needed. No blind full-parameter restore.

Source fast-forwarded from3fb653b to a1dbd13; package built and installed runtime/assets compared against source. Persistent service override95-firmware472.conf selects4.7 for the entire core process. After restart, one MAVProxy, one fused bridge and one supervisor were observed; navigation profile verified with no mismatches. Controller remained MANUAL/disarmed/killed/ESC-off.

Backups on Jetson:
- /home/graey/deployment-backups/20261023-ardusub472 (parameter snapshots, migration diff, flash log, code bundle, service backup)
- /home/graey/deployment-backups/20261023-upgrade472-195243 (code deployment snapshot/bundle; suffix uses vehicle clock)
- rollback official4.5.7 APJ at /home/graey/CubeOrange-4.5.7-rollback.apj. Rolling back requires reviewing/restoring old parameters and selecting4.5; do not just flash and run.

Migration changed several firmware defaults (including ATC_INPUT_TC0.15->0.10 and AHRS_OPTIONS0->16) and automatically renamed many controller parameters. Full diff retained; real control response remains unverified. Ground pressure recalibrated on reboot; no old pressure baseline restored. GPS/DVL validity, VN calibration, surface Prepare and controlled wet verification remain necessary before a mission. Software simulations do not prove physical heading, depth or thrust behavior.
