# GRAEY RFD mission relay

Status: implemented and software-tested locally on 2026-11-11. It has not been
deployed to GRAEY or exercised through the physical RFD radios.

## Purpose

The laptop remains the mission planner. The Jetson executes GRAEY's existing
mission implementations. The Cube is the single radio middleman between them:
it transports MAVLink messages, supplies vehicle state, and accepts the normal
ArduSub mode/control commands, but it does not interpret or store the GRAEY
application mission.

This supports all current GUI mission choices:

- Waypoints: ordered plotted geographic points.
- Pipeline Survey: plotted start/end, using the waypoint route logic while
  submerged; camera/sonar recording remains operator-controlled.
- Survey Ruth: its existing plotted survey route.
- 5-2-5: the existing local relative sequence, with no geographic point list.

QGC AUTO missions remain separate from these Jetson-executed, heading-aligned
GUIDED missions.

## End-to-end flow

1. The operator plots/selects a competition mission in the GRAEY laptop GUI.
2. `GRAEY_GUI.cmd` sends a canonical mission document to `fleet_link` on UDP
   14572. The document is compressed, checksummed, split into bounded MAVLink
   `TUNNEL` chunks, and retransmitted until the Jetson acknowledges it.
3. The shared OCS routes companion system 43 traffic over the RFD link. The
   Cube relays it to the Jetson through the existing MAVLink path.
4. `radio_mission_node` reassembles and validates the document, stores it
   atomically under `/root/robotx_ws/missions/radio/<sha256>.json`, and invokes
   the same checked local mission API used by WiFi/tether. Duplicate requests
   are idempotent; a reused request ID with different data is refused.
5. The existing mission owns preparation and arming. For a competition mission,
   it enters actual Cube `POSHOLD`, publishes no GUIDED target, and reports
   `STATE_AUTO` to RoboCommand because the autonomous mission is claimed.
6. GRAEY waits in POSHOLD until a verified `RUN_START` from OCS system 250,
   component 190 targets GRAEY companion system 43 and matches the prepared
   mission's run. Early RunStart messages are intentionally not acknowledged,
   so the existing OCS retry mechanism keeps them pending.
7. GRAEY acknowledges the matching RunStart, latches admission, switches to
   GUIDED, and runs the existing selected mission. Loss of WiFi or RFD after
   admission does not revoke or interrupt the onboard mission. A changed/ended
   competition run still revokes the gate through the existing run logic.

## Ownership and compatibility

- MAVProxy remains the sole Cube serial-port owner.
- `telemetry_gateway` remains the sole ROS FCU gateway.
- `radio_mission_node` uses `Link('ros://graey/fcu')`; it opens no serial or UDP
  Cube endpoint.
- GRAEY uses companion MAVLink identity 43/201. The laptop application client
  uses 254/191. The OCS adapter uses 250/190.
- The gateway admits only the required mission protocol and standard RXL
  heartbeat/ACK payloads for those identities and destinations.
- A private, non-competition status beacon establishes route availability even
  when a complete competition heartbeat cannot be built. It never fabricates
  vehicle telemetry. The beacon and ROS status name the exact heartbeat hold
  reason when a required measured field is unavailable.

The common gateway/Link ownership, fleet routing, RXL RunStart/ACK behavior,
task-state reporting, and OCS retry model follow the inspected Crusader main
implementation (`e44573034f590c60b55b4e5ec50e2ebc77e7d13f`) and the shared
team OCS main (`08ec1a29de6d78c8ed1edc3f7c83549ba4bc9889`). The compact mission
document and GRAEY mission dispatch are concrete submarine adaptations: these
missions execute on GRAEY's Jetson and must be editable after launch over RFD.

## Robustness rules

- Canonical JSON plus SHA-256 gives an operator-visible mission revision.
- zlib payloads are bounded to 32 KiB compressed/128 KiB raw, at most 384
  chunks, with CRC, version, total-length, duplicate, conflict, and timeout
  checks.
- The receiver accepts out-of-order chunks and ignores identical duplicates.
- Replies are cached by request ID for retry-safe acknowledgement.
- The laptop retries all chunks every 750 ms and does not report success until
  it receives the matching acknowledgement.
- Invalid source, component, target, payload type, mission kind, coordinates,
  run ID, or stale/incomplete message is rejected.
- No GPS, tether, or RFD presence is added as an artificial blocker to loading
  a mission. Actual mission preparation still retains GRAEY's existing hardware
  kill, ESC, arming, navigation-health, pilot-takeover, and control checks.

## Local validation

- GRAEY full suite: 504 tests passed, 5 skipped.
- Shared OCS focused configuration/socket/adapter suite: 205 tests passed.
- Shared OCS full suite: 1,145 tests passed, 1 skipped.
- Official-server shared E2E: 13/13 stages passed, including UUV1 at 2 Hz,
  RunStart, task-report PUBACK, submerged retention, reconnect and run end.
- Mission radio tests cover encoding, corruption, out-of-order delivery,
  duplicates, request idempotency, backend dispatch, terminal cleanup, early
  RunStart retry, matching admission, mismatched-run rejection, laptop request
  acknowledgement and explicit no-ack failure.
- Navigation tests cover real POSHOLD waiting, `STATE_AUTO` publication, GUIDED
  execution after admission, and retaining the admitted run across link loss.
- Survey tests cover forwarding the competition flag through its first route.
- Python compilation, mission JavaScript syntax, and Git whitespace checks pass.

## Required hardware acceptance when GRAEY is available

1. Start killed/disarmed with ESC power off and verify the sole MAVProxy/gateway
   ownership before connecting radios.
2. Verify and, only with operator authorization, apply a unique GRAEY Cube
   `SYSID_THISMAV=3`. Historical evidence shows GRAEY and Ekko both used system
   ID 1; the shared fleet configuration now reserves autopilot 3 for GRAEY.
3. Verify the current Windows RFD COM port, baud, node IDs, and bidirectional
   clean packet transport; historical COM3/115200/node 4 is not current proof.
4. Perform a disarmed surface rehearsal: send each mission type, compare
   revision/type/point count, deliberately resend it, and verify exactly one
   prepared mission.
5. Declare a test run, verify GRAEY is in actual POSHOLD while RoboCommand sees
   UUV1 Auto, then deliver matching RunStart and confirm the transition to
   GUIDED. Do not permit propulsion until the operator completes the normal
   physical safety checks.
6. During a benign admitted mission, disconnect the RFD application path and
   confirm the onboard mission continues; reconnect and confirm status recovery.

Until these steps are completed, the physical radio route, sysid change, mode
transition, and actuation behavior remain unverified.
