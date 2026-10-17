# Incorrect water-depth source — 2026-10-17

## Evidence and root cause

User reported descent to approximately 0.7 m without stopping. The current
mission `293b00f54d3642e7b7f86f5fc9fb1da8` actually requested 0.4 m. Its frozen
Cube surface Down was 0.6513 m; target Down was 1.0513 m. A nonzero fixed local
zero is supported and is not alone proof of a control defect.

Read-only analysis of Jetson's MAVProxy telemetry log over Cube boot seconds
9558–9570 revealed the decisive error:

| Message | Observed pressure, hPa | Behavior |
| --- | --- | --- |
| SCALED_PRESSURE | 977.57–978.83 | Little change |
| SCALED_PRESSURE2 | 977.84–979.09 | Little change; incorrectly used as water depth |
| SCALED_PRESSURE3 | 992.60–1118.20 | Strong rise during descent, then decline during ascent |

The bridge had hardcoded SCALED_PRESSURE2. It therefore reported only about
0.008–0.021 m through this interval instead of tracking submergence. DVL altitude
is bottom clearance and is not the mission depth input. The measured channel-3
pressure rise corresponds to more than the user's approximate depth estimate;
do not treat the phone/operator estimate as a calibrated depth measurement.

This same bad depth also affected GPS rejection: the pressure gate could keep
accepting GPS while physically submerged because it falsely indicated shallow
water. This does not prove all geographic position errors came from that gate.

The mission aborted approximately 5.4 s after DIVE entry when DVL interruption
exceeded its prediction horizon. No successful physical waypoint is claimed.

## Fix

- Explicit read-only `pressure_message` configuration defaults to the verified
  water-responsive `SCALED_PRESSURE3` on this GRAEY installation. Accept only
  supported pressure message names; request the corresponding MAVLink message
  ID at 10 Hz. Never silently fall back to an air-pressure channel.
- Include pressure source in bridge sample status; the same selected measurement
  supplies Prepare baseline, calibrated depth, GPS exposure gate and ODOMETRY Z.
- Remove the supervisor's redundant channel-2 stream-rate request. It observes
  bridge depth rather than selecting another pressure sensor independently.
- Abort navigation to MANUAL/pilot takeover if fresh pressure depth exceeds the
  requested mission depth by more than 0.15 m, or Cube Down minus pressure depth
  changes by more than 0.15 m from the captured vertical offset. Check before
  temporary permission hold. Log depth/local pose/offset in abort evidence.
- A stable nonzero Cube/pressure offset remains supported. Guards do not promise
  instantaneous braking, automatic ascent or disarming underwater.

The physical sensor identity was inferred from its water-responsive trace, not
from a wiring inspection. Verify the configured pressure channel after hardware
changes. MAVLink defines pressure units as hPa and identifies pressure3 as the
third barometer: [official message reference](https://mavlink.io/en/messages/common.html#SCALED_PRESSURE3).

## Map findings

User confirmed the issue is on Satellite/live waypoints, which uses geographic
tiles directly and does not use screenshot calibration. Raw GPS is M9N receiver
data, not a DVL measurement; raw fixes are displayed even if rejected for fusion.
The collected receiver fix claimed 1.426 m accuracy and 0.534 m/s speed sigma.
These are receiver estimates, not verified error bounds. Cube local position
was stale/untrusted at collection. No independent surveyed position was supplied
for this snapshot, so imagery registration versus constant GNSS bias cannot yet
be separated. Projection and tile coordinate round-trip tests pass; this does
not certify the imagery's absolute geographic accuracy.

## Verification

215 Python tests and both GUI Node tests passed. New cases reject wrong pressure
channels, verify channel-3 depth conversion/submerged GPS gating, permit explicit
channel configuration, and test vertical-reference/overshoot takeover including
the temporary-permission-hold path. Integrated simulator results and deployment
are recorded below. Synthetic SITL depth alone could not have exposed the live
sensor-instance mapping error; transport regression uses the observed mapping.

Final native GUI waypoint simulation waypoint-b857686e92 passed with DVL and supervisor interruptions, target0.5m and final simulated horizontal error0.105m. GUI now names the selected pressure channel and distinguishes DVL bottom clearance. No physical dive was run by the agent.

