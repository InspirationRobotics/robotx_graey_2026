# GRAEY adaptation to current Crusader and Ekko

**Current OCS:** [Shared team OCS](shared-team-ocs.md) supersedes the former patched OCS/skip-fleet instructions below. Native GRAEY reporting now works with unchanged team main; no OCS patch is required.

**Unified operator update:** [Missions, Sensors and Record guide](unified-operator-guide.md) supersedes the earlier separated Map/Mission/5–2–5 workflow and Communications-tier location. Declare/End now use the shared OCS API from Missions.

Local development, November 9, 2026. No hardware deployment, vehicle parameter changes, commit or push. This document supersedes the earlier SD-only connection/startup instructions in the operator and WiFi guides.

## Inspected sources

The user selected **main plus communications branches**. References were fetched into separate clean checkouts; the existing dirty Crusader, Ekko, OCS and GRAEY work was preserved.

| Reference | Pinned commit | Applied to GRAEY |
|---|---|---|
| Crusader main | `abf7d8214a7ef3194d4ec59ca5025795436adae8` | Laptop/vehicle GCS split, exact `gcs_wire.py`, discovery helpers, mission-active AUTO heartbeat |
| Crusader `feat/runstart-gate-tiers` | `86d5d8a913e751115b9a8860146120a45282959c` | Native run snapshot and four-task tier request/reply interfaces |
| Ekko main | `fb913f40d2bbc09082a031771a4d182d02d671a5` | Current baseline inspected; this pointer matches the archived SD main |
| Ekko `ocs-rxl-uplink` | `2fc255e67c61691574c362eea8620b01ac997aa2` | Shared task-token normalization; invalid tokens do not silently stand down a task |
| OCS main | `9ccbb8c2538d1d7636b05852161ab7c4b5366ef7` | Current run, tiers, declaration and discovery behavior; local UUV compatibility patch rebased |

The Ekko communications branch is divergent and older than main; it was used for relevant communications contracts, not substituted wholesale. The existing validation codec matches current OCS's codec. Source IDs and hashes are recorded in the parent workspace's `competition-comms/current-reference-2026-11-09/manifest.json`. Pre-sync local communications/OCS sources are preserved in `pre-sync-local.zip` there.

## Laptop to vehicle

```text
Laptop browser http://127.0.0.1:8152
  -> stdlib GRAEY laptop app
  -> one TCP connection, GRAEY_WIFI_IP:14559, Crusader gcs_wire framing
  -> GRAEY gcs_bridge -> loopback GUI backend :8090
  -> existing checked ROS command interfaces

GRAEY competition_link_node -> OCS laptop :37564 -> RoboCommand
                   discovers OCS through team-matched UDP :37565 beacons
```

The GCS bridge starts with the local core launch. It does not touch serial, MAVLink or controller state. On the laptop, from a copy of this repository with standard Python installed:

```powershell
python tools/graey_gcs.py --host GRAEY_WIFI_IP
# Or tools\GRAEY_GUI.cmd --host GRAEY_WIFI_IP
```

Open `http://127.0.0.1:8152`. No ROS, pymavlink or Python packages are needed on the laptop. Multiple known vehicle addresses can be comma-separated with `--host`; a working connection is retained rather than switching on every poll. The page shows laptop-local link age/round-trip status. A request with an unknown outcome is not replayed automatically. Existing direct vehicle HTTP remains available for compatibility.

This uses the **exact Crusader frame format**, with a GRAEY `http` request operation that retains its checked submarine backend. It is not a claim that Crusader's boat app/BT XML operations can control GRAEY. Unsupported wire operations are refused. Logs/assets/state and the allowed auxiliary camera/sonar services travel through the bridge. Camera viewing remains opt-in and tab-gated; MJPEG is sampled into bounded JPEG frames for this transport. Hardware camera/sonar commissioning remains pending. The bridge assumes the existing backend is on 8090; a custom GUI port requires matching `gcs_bridge --backend-port`.

Vehicle bridge default: TCP 14559. Laptop HTTP binds loopback only. `--vehicle-port`/`--port` change the laptop defaults. Core `gcs_bridge_enabled:=false` disables the bridge; `gcs_port:=14559` selects its listen port. Core-node stop controls remain protected, and power actions now require fresh disarmed telemetry and stopped missions.

## OCS connection and run handling

Core now starts the reporting node, as current Crusader does. `communications_enabled:=false` disables it. Its default `ocs_host:=auto` follows team-matched beacons, accepts fresh addresses, retries a busy discovery port, and retains an established connection. Optional `ocs_fallback_hosts` in the node's YAML accepts known team IPv4 addresses; none are guessed. An explicit team hostname/IP still works. Course-network destinations are rejected, including when resolving a hostname.

Native `ocs_run` snapshots are event-driven. A snapshot remains authoritative during that TCP session rather than expiring after five seconds. Disconnect invalidates admission/report transmission until the new session learns the current run. Legacy SD run snapshots retain their old lease behavior for transitional fixtures. A new declaration or matching end revokes an admitted competition run; underwater communication loss alone does not. Local training/manual missions remain independent. The boat branch's all-mission gate and bench release were deliberately not adopted because the user chose competition-only gating.

An armed active GRAEY mission reports STATE_AUTO while it holds the existing fresh controller mission lease, including GUIDED execution and the RunStart hold. Kill/ESC inhibition still wins. This is reporting behavior; it does not grant control permission or restore power. Task names accept case/whitespace normalization, with invalid names rejected without overwriting the last valid value. GRAEY retains the freshness timeout of its real periodic mission producer rather than Ekko's temporary persistent bench-task behavior.

## Per-run tiers and retained reports

In Communications, **Load current OCS tiers**, edit the desired fields, then **Send tiers to OCS**. All four tasks are explicit because the tiers belong to the fleet, not just the sub. This sends one `ocs_tiers` control request; it is never queued across disconnection. Source freshness/deadlines also apply between the local HTTP and ROS processes. OCS accepts/refuses and provides its reason in `ocs_run.reply`; declaration stays centralized in OCS. Requests are refused while a run is DECLARED/RUNNING.

Stock current OCS can supply discovery, run state, tiers and native heartbeats. Durable underwater observations also need the small supplied [compatibility patch](competition-ocs.patch): it advertises `graey_report_v1` in UUV1's native snapshot and checks the retained report's run ID before ingestion. Without that capability, observations remain stored and the GUI explains the hold; unknown envelopes are not sent to a stock parser. The external RoboCommand protobuf remains unchanged. SQLite sequence/observation IDs, previous-run quarantine and broker PUBACK receipts are retained.

The parent workspace's `competition-comms/OCS` is now current OCS main plus this patch and UUV WiFi configuration. The separate pre-existing `rx26_ocs` was not overwritten. The patch is checked against the pinned current main, not the old SD tree. It has not been installed on an operator laptop.

## Validation and remaining commissioning

Local tests cover native snapshot silence, disconnect/end/redeclare handling, retained observations and current OCS PUBACK, tier acceptance/refusal, discovery freshness/fallback/team/course guards, real browser HTTP through the single TCP bridge, origin checks and no mutation replay after a lost response. ROS uses synthetic FCU frames on an isolated domain; browser data is explicitly synthetic. Build and installed launch argument checks run without starting vehicle nodes. Detailed final results are in the parent workspace's `competition-comms/standardization-verification.json` and logs.

Physical WiFi range, real hardware camera/sonar, sensor freshness and control commissioning must be checked when GRAEY is available. RFD reporting and Ekko-to-GRAEY target reception are not enabled. Navigation tuning, MAVProxy serial ownership, hardware kill and RF/tether failsafe selection were preserved.

Final result: **PASS**. GRAEY 443 tests (5 skips), current OCS 614 (2 skips), fleet 533 (1 skip), WiFi 9 and Linux contracts 11 pass. ROS reporting, direct and real-TCP browser checks, colcon, installed launch discovery and 128 installed file comparisons pass. See [validation record](validation/current-fleet-adaptation-2026-11-09.json).
