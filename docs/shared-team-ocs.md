# Shared team OCS

Use tools/scripts/OCS.cmd in this repository, tools/OCS.cmd, or OCS.cmd in the parent RX 2026 GRAEY folder. The wrappers locate the sibling team-ocs checkout (or rx26_ocs in a normal teammate layout), use its existing run/laptop.toml if present, and otherwise retain its stock configuration. It calls team-ocs/OCS.cmd from the published InspirationRobotics/rx26_ocs main branch, unchanged. This is the same fleet OCS application and startup as the team: fleet_link plus ocs_web, opening http://127.0.0.1:8099/map. It does not create an independent GRAEY competition run.

The local run/laptop.toml config uses the supported [map] and [[map.gui]] options: Singapore overview and GRAEY localhost8152. A map center is not a course or vehicle position. Stock vehicle identities, fleet setup and RoboCommand processing are retained. The upstream fleet configuration currently names COM8; if this laptop's actual base-radio port differs, use the team's FLEET_LINK.cmd --serial PORT before OCS.cmd. No radio port was guessed or opened here.

The local launcher makes team-ocs/.venv/Scripts available to the fleet launcher because this laptop's normal python command is an unavailable Windows alias. Runtime imports for OCS and fleet are verified. The local venv uses already-installed dependencies via an ignored .pth; for a standalone teammate install, use the upstream requirements and fleet pymavlink/pyserial prerequisites. Source and tracked configuration stay unmodified.

GRAEY uses the team's native TCP report contract, so no custom OCS patch is needed. Current-run/surface/outbox checks remain on GRAEY. Standard reports have the same run-transition/in-flight semantics as Crusader; the native wire format has no GRAEY-specific run tag. Do not apply docs/competition-ocs.patch: it describes the superseded local fork. Earlier OCS directories remain preserved, not active.

Verified against published main 9ccbb8c2538d1d7636b05852161ab7c4b5366ef7: team source clean; 610 OCS tests pass (1 skip); eight real-TCP GRAEY integrations pass including local MQTT PUBACK. GRAEY unit suite passes 464 tests (5 Windows skips). No vehicle/radio connection, deployment or online push was performed.

Final shared-OCS validation: 610 team OCS tests pass (1 skip), all 533 fleet tests pass after restoring the SD team Mesh generated dialect at its normal Mesh/dialect/gen/robotx.py location, eight GRAEY-to-stock-OCS integrations pass (including actual local MQTT PUBACK), and 464 GRAEY tests pass (5 Windows skips). OCS/fleet CLI and runtime imports pass. Team OCS Git working tree remains clean. No serial/radio was opened.

The generated Mesh dialect is copied byte-for-byte from the SD team Mesh directory (SHA256 ada11869479f9a69738a2d7bf48e6d8e1e63267daab7f5e830c2341e3bfab0f7); stock fleet_link discovers it automatically. No dialect source was edited.
