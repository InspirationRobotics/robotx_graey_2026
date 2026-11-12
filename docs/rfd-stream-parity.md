# GRAEY / Crusader shared-RFD stream parity

## Required architecture

Crusader's Pixhawk telemetry and QGC path use MAVProxy over IP. Its RFD is a
separate Jetson-owned endpoint carrying bounded application `TUNNEL` frames.
GRAEY retains its authorized physical wiring through Cube TELEM1, but the
logical split is the same:

- WiFi/tether: native Cube telemetry, QGC and the laptop TCP GUI.
- Shared RFD: companion system 43 application traffic plus only the minimum
  Cube telemetry required for the explicitly selected QGC/RadioMaster workflow:
  truthful 2 Hz RXL heartbeat, acknowledged mission transfer, RunStart/ACK and
  bounded status/error keepalives.
- The Cube relays the application frames. Its generated TELEM1 groups must be
  bounded to the approved QGC minimum rather than mirroring the USB stream.

This is the concrete submarine adaptation to Crusader. GRAEY needs mission
loading after leaving WiFi range; Crusader does not. That difference does not
authorize an unbounded telemetry stream.

## Identities

The fleet allocation is:

| Vehicle | Autopilot | Companion |
|---|---:|---:|
| Ekko | 1 | 200 |
| Crusader | 2 | 42 |
| GRAEY | 3 | 43 |

On November 12, 2026 the operator approved GRAEY system 3. Runtime command
targets, Cube-source filters, gateway admission, maintenance tools, mission
tools and current SITL now share the dependency-free identity constants in
`api/pixhawk/identity.py`. Historical system-1 parameter backups and evidence
scripts remain unchanged as dated evidence. Identity does not by itself fix
bandwidth: stream control does.

## Stream controls

- GRAEY MAVProxy starts with `--streamrate=-1`, matching Crusader: it never
  sends a broad `REQUEST_DATA_STREAM` request. `telemetry_gateway` owns the
  specific high-rate USB intervals used by navigation.
- QGC remains bidirectional on RFD for the archived QGC/RadioMaster manual-drive
  workflow. Shared OCS blocks only `REQUEST_DATA_STREAM` and
  `SET_MESSAGE_INTERVAL` writes on the RFD leg, regardless of target identity;
  other pilot commands continue through the learned RFD route and the same
  broadcast stream-control frame may still reach Crusader over WiFi. The
  blocked count is visible as `dropped_stream_control` in `/fleet`.
- `params/graey_4.7_rfd_competition.parm` uses ArduSub 4.7's live parameter
  names (`MAV_SYSID` and the first telemetry instance's `MAV1_*` groups) to set
  the approved Cube identity 3 and minimal QGC profile: 1 Hz `MAV1_EXTRA1`,
  `MAV1_POSITION` and `MAV1_EXT_STAT`; ADS-B, parameter, raw-control, raw-sensor,
  RC feedback and the remaining broad groups are zero. The overlay deliberately
  does not change serial baud/protocol/flow control. Routed companion TUNNEL
  traffic and the fixed autopilot heartbeat remain.
- The normal radio liveness source is the handbook's 2 Hz RXL heartbeat. The
  private mission-status beacon is change-driven with a 5 s keepalive; while a
  truthful RXL heartbeat cannot be built it uses a 1 Hz route-presence signal.

## Coordinated commissioning

Do not deploy the identity change separately from its Cube/runtime/OCS
configuration. The repository migration is coordinated for Cube system 3,
onboard components under system 3, and companion application 43/201.

1. Kill and disarm GRAEY, leave propulsion inhibited, and save the current Cube
   parameters.
2. Deploy the matching GRAEY software and shared OCS configuration.
3. Set the approved identity consistently, apply the competition profile and
   reboot the Cube once. The reboot clears
   per-message `SET_MESSAGE_INTERVAL` runtime state that parameter writes do not.
4. Verify the selected Cube source/component and source 43/201 for the GRAEY
   radio application. Confirm the approved QGC control path still works and
   cannot alter GRAEY's TELEM1 stream-rate policy.
5. Measure GRAEY alone, Crusader alone, and both together for at least 90 s.
   Record Cube SERIAL1 TX bytes/s, good/bad frames, GRAEY heartbeat rate,
   Crusader application rate and longest gaps. Do not claim completion until
   both vehicles retain their expected rates without a rising bad-frame count.

## Software validation

The system-3 migration passed the complete GRAEY suite (509 tests, 5 expected
skips), the shared OCS adapter/routing suite (207 tests), and the isolated
RoboCommand E1-E13 workflow (13/13). The first E2E run correctly exposed a
system-1 synthetic Cube fixture; migrating that fixture to the production
identity restored UUV1's 2 Hz heartbeat, declaration, RunStart, retained
submerged survey, surfaced delivery/PUBACK and clean run end. These are software
and loopback results, not physical shared-air bandwidth evidence.

## Live GRAEY deployment — November 12, 2026

The system-3/bounded-RFD correction was deployed to GRAEY while the Cube was
fresh, disarmed, in MANUAL, with ESC power off and the operator-confirmed
physical kill engaged. The original tracked patch, Git bundle and complete
source tree are retained at
`/home/graey/robotx_ws/releases/pre-sys3-20261112-01`. The active install is the
atomic release `releases/3194729-20261112-sys3/install`; the previous
`d4ca830-20261110-refresh` install remains available for rollback.

Live pre-write reads corrected two assumptions in the initial profile:
ArduSub 4.7.2 exposes `MAV_SYSID` (not `SYSID_THISMAV`), and the RFD is on the
first MAVLink telemetry instance. Before deployment, `MAV_SYSID=1` and every
broad `MAV1_*` stream group was 4 Hz. `SERIAL1_PROTOCOL=2`,
`SERIAL1_BAUD=115` and `BRD_SER1_RTSCTS=0` were retained unchanged. The
allowlisted deployment changed only identity and the ten MAV1 stream groups,
then rebooted once. Post-reboot readback from source 3/1 exactly matched the
profile with no missing or mismatched values.

The vehicle checkout contained a legacy, untracked component-199 radio stack.
It is preserved in the backup/source checkout but is no longer launched or
admitted; the active bounded endpoint is companion 43/component 201. A live
outbound trace showed navigation and controller traffic at system 3, and only
the intended 43/201 TUNNEL traffic on the radio path. It also found and fixed
the last literal system-1 parameter target in `nav_ekf_bridge`. The MAVProxy
service required a direct-entrypoint path fix after identity centralization;
its real systemd invocation and a standalone `--help` launch both pass.

Validation on the merged vehicle source: 495 tests passed with 14 expected
skips; focused post-fix identity/transport/polling tests passed 34/34. A
90-second live sample kept both services active, Cube telemetry fresh,
disarmed/ESC-off state unchanged, and the gateway refusal counter constant.
The 18 retained refusals were stale frames from the deliberate verification
client teardown, not identity errors. A final ROS-only restart cleared the
diagnostic counters; the following 30-second normal-operation sample stayed at
zero refusals. GRAEY's WiFi OCS client was disconnected
because no OCS listener was running at `192.168.2.1:37564`. Laptop COM5 was
already owned by another process, so it was not forcibly opened; Crusader was
not present. Actual receive bytes/s and the simultaneous Crusader/GRAEY
90-second shared-air acceptance test therefore remain required.

