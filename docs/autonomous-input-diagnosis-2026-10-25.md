# Autonomous weak-motion diagnosis — 2026-10-25

Read-only diagnosis; no movement, parameter changes, or restart.

Run f3253588dad9431dad0ca0c0414a90d1: 75 TRANSIT trace samples over42.3s;74 commands,61 at forward cap150,none zeroXY;medianDVL speed0.02168m/s. Disarm terminated the run, but does not explain weak prior motion.

MAVProxy mav.tlog bounded using RC_CHANNELS boot time2363000–2405000ms: MANUAL_CONTROL source255/1902032 messages versus mission255/198265. Most255/190 inputs(0,-15,484,0)1222 and(23,-15,484,0)520; also172 full-forward inputs. Cube RC feedback mostly(1500,1497,1500)210 or(1504,1497,1500)86 for forward/lateral/yaw; mission(1530,1470,1480)9. Conflicting GCS joystick commands overwrite mission commands. Component190 identifies a GCS-style sender; QGC is likely, physical application identity not independently confirmed.

Recorded parameters JS_GAIN_DEFAULT0.5, RC4_DZ40, RC5/6_DZ30, RC4/5/6_MIN885/TRIM1500/MAX2115. ArduSub4.7.2 joystick.cpp scales axes by0.4*gain. Mission forward150 becomes30PWM atdefaultgain, normalized~0.049. mode_stabilize.cpp yaw deadzone isRC4_DZ*gain; yaw cap100 maps to40*gain exactly atdeadzone, hence zero requested yaw rate absent trims. Forward usesnorm_input, so do NOT claim forwarddeadzone eliminatesit.

Fix direction: single motion-input owner during autonomy, with explicit immediate pilot takeover/kill preserved; correct command conversion for Cube scaling/deadzones; verify with competing-input SITL and Cube RC feedback. Do not merely increase thrust while competing streams remain. No fix deployed by this diagnosis.

Sources: https://raw.githubusercontent.com/ArduPilot/ardupilot/Sub-4.7.2/ArduSub/joystick.cpp and mode_stabilize.cpp in same directory.
