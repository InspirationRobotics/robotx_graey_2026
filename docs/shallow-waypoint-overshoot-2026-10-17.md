# Latest MANUAL transition and raw GPS — 2026-10-17

Mission d355c3905c5d426abd37036637057ff3 requested depth0.4m. It aborted at
pressure depth0.57818m with CubeDown1.19453m; the captured vertical offset was
0.64874m and offset change only-0.03238m. This time DVL TRACKING, bridge healthy,
reference current and navigation ready were true. The pressure overshoot guard
explicitly requested MANUAL. It was not a heading fault or source switch.

Read-only live limits: WPNAV_SPEED_DN150cm/s, WPNAV_SPEED_UP250cm/s,
WPNAV_ACCEL_Z100cm/s². These allowed an aggressive shallow descent. Prepared a
pool commissioning profile10cm/s down,15cm/s up,50cm/s² acceleration. This is a
conservative starting profile, not a guarantee of physical stopping distance.
No PID, heading or EKF gain changes. The pressure overshoot guard stays active.
ArduSub guided waypoint control uses waypoint navigation speed/acceleration
limits: [firmware implementation](https://github.com/ArduPilot/ardupilot/blob/master/ArduSub/mode_guided.cpp).

Mission adapter now reads all three limits before starting and continuously
checks fresh readbacks; excessive/missing limits prevent startup and cause
takeover during a run. It never silently writes parameters. The isolated SITL
uses the same limits. Unit regression checks missing/excessive/stale limits.

The subsequent raw GPS sample reported accuracy6.808m, speed7.59m/s and speed
sigma3.962m/s. Bridge admission was false. User confirmed the raw marker wandered
and the blue marker disappeared after DVL loss. The raw receiver is not a valid
replacement for a lost Cube estimate. Later DVL gaps exhausted actual modeled
position uncertainty (~5.00m); brief-dropout recovery had worked during the run.
An unknown missed displacement cannot be erased by declaring bottom lock good.
The existing surfaced Recover DVL action preserves north calibration but needs
its stationary/fresh-position checks. No thresholds were loosened to accept bad
GPS or invent missing DVL position.

GUI now colors non-aiding raw GPS gray, labels it NOT AIDING and shows rejection
reason plus receiver-reported position accuracy. This corrects misleading
presentation, not the receiver's measurement. Low-quality resurfacing fixes
remain excluded; timing/depth/DVL/residual gates unchanged. A constant GPS bias
still requires an independent reference to measure.

215 Python tests and waypoint GUI tests passed. Native GUI SITL
waypoint-dbc0152678 passed full dive/transit/surface with injected DVL and
permission interruptions; vertical limit readback verified, final simulated
horizontal target error0.249m. No live mission performed by the agent. Deployment
and parameter readback results follow below when verified.

