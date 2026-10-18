# Surface waypoint descent: local fix and validation

## Confirmed incident, October 17

Run `b103d375d141410d80b507f414873389` requested surface mode and depth zero.
Its frozen Cube down target was 0.002464 m, not a requested 0.2 m dive. It aborted
when pressure depth reached 0.195785 m; Cube local down was 0.168635 m. User reported
the vehicle stayed level. The bridge was healthy at abort. The older guard waited
until pressure exceeded 0.15 m and could overshoot between samples.

The old surface mission still sent a full XYZ GUIDED position destination, using
the underwater mission's 3D waypoint-control path. This is verified code behavior;
it is **not proof** that a positive-depth command caused the physical descent.
Vertical motor polarity, controller gains/integrator, trim/buoyancy and actual
output response were not established from the available evidence.

## Changes

- Surface travel now streams velocity setpoints at approximately 10 Hz. All XYZ
  position fields are ignored by the MAVLink mask; explicit legacy dive missions
  retain their previous position interface.
- Jetson computes north/east velocity from position error plus measured-velocity
  damping, with a 0.20 m/s horizontal magnitude limit.
- Surface vertical requests are upward-only in NED: -0.03 m/s normally, increasing
  gently with positive pressure depth, capped at -0.10 m/s. Transport rejects any
  positive/downward velocity request. It does not modify EKF origins or sensor
  calibrations.
- Surface pressure >0.08 m aborts and requests MANUAL takeover. The pilot must
  remain ready to take control; a software threshold cannot prevent all physical
  overshoot or automatically bring a vehicle back up after takeover.
- Trace logs record the actual requested surface velocity alongside the waypoint
  goal and Cube/bridge/pressure measurements. The existing `target_cube` and
  `last_sent_target_ned_yaw` entries remain mission-goal metadata for this mode;
  the active velocity request is `last_sent_surface_velocity_ned`.

ArduSub's GUIDED velocity implementation still runs its own Z controller. Therefore
upward-only **velocity requests are not a guarantee of upward-only motor thrust**.
The fix removes the previous fixed XYZ waypoint interface, constrains requests,
and tightens the stop guard; physical vertical response remains a required check.

## Validation on October 18 (no vehicle access)

- 245 Python tests passed, including surface mask/field encoding, no positive Z
  request, bounded horizontal speed, braking near target and the 0.08 m abort.
- Node waypoint GUI tests passed.
- Native isolated ArduSub/ROS/GUI mission `waypoint-cc5e87539a` passed with a
  two-second GPS dropout. Reached COMPLETE/disarmed, no DIVE/ascent stages;
  final simulated Cube horizontal target error 0.1207 m.
- 28 mission trace samples: maximum pressure depth 0.0400 m; 27 recorded velocity
  requests, down component -0.0620..-0.0300 m/s; maximum horizontal request
  0.2000 m/s. Evidence under `docs/evidence/surface-*2026-10-18.json`.

- Native GUI-integrated operator-Abort run `waypoint-ad3bbb8925` passed: transit stopped, mission ABORTED, and the validator confirmed MANUAL takeover. Evidence: `docs/evidence/surface-velocity-abort-2026-10-18.json`.

All simulations used localhost ROS domain 73 and disabled QGC forwarding. These
results establish simulated behavior, not physical buoyancy or motor correctness.
The earlier test launch was blocked by automatic approval review when usage was
exhausted; no test/deployment occurred in that blocked call. Tests were retried
normally after availability returned, without bypassing approval.

## Deployment status and next physical test

**Not deployed:** user explicitly reported Graey unavailable and requested local
completion. No SSH, vehicle reboot, parameter changes or physical test occurred
on October 18 for this work. Last verified vehicle software was `9c1eab0`, the
automatic surfaced DVL recovery release.

When Graey returns: deploy under fresh killed/disarmed/ESC-off checks; Prepare and
establish north; verify surface pressure zero and actual vertical response before
another waypoint. Use a short clear surface target with immediate pilot/kill
supervision. Stop if it descends; do not tune around an unverified motor direction.

Firmware reference inspected:
https://github.com/ArduPilot/ardupilot/blob/ArduSub-4.5.7/ArduSub/mode_guided.cpp
(guided velocity control includes Z controller), and
https://github.com/ArduPilot/ardupilot/blob/ArduSub-4.5.7/ArduSub/GCS_Mavlink.cpp
(SET_POSITION_TARGET_LOCAL_NED velocity path).
