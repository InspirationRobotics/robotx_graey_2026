# Unified GRAEY mission and transport policy — 2026-11-12

## Decision

GRAEY has one operator workflow and one mission implementation regardless of transport:

- **Auto** selects a fresh WiFi/tether connection first and uses the acknowledged RFD application path only when IP is unavailable.
- **WiFi/tether only** and **RFD only** are explicit operator overrides.
- Transport selection does not change competition policy. A competition mission arms under the existing checks, enters actual Cube `POSHOLD`, reports RoboCommand `STATE_AUTO`, and remains held until a verified matching `RunStart` is admitted.
- A mission prepared over IP is claimed by the onboard RFD RunStart receiver from its ROS mission status. Losing WiFi therefore does not make a correctly prepared competition mission invisible to RoboCommand.

The laptop UI displays WiFi/tether, RFD, RadioMaster RC, onboard OCS client and RoboCommand/RunStart state separately. None of those indicators substitutes for another.

## Unified mission workflow

All mission types use the Missions page's same sequence:

1. Select and configure the mission.
2. **Load to GRAEY**. GRAEY validates and durably stages the exact document and returns its SHA-256 revision. Loading never arms or moves the vehicle.
3. **Prepare mission** over the same currently selected link. A changed document or an Auto failover to a different link requires another Load, so the operator cannot unknowingly prepare a document that was never acknowledged on that path.
4. Local/training missions start after existing readiness checks. Competition missions reach POSHOLD/Auto and wait for verified RunStart.
5. **Stop mission** uses the selected link. The onboard mission continues independently through laptop-link loss; link loss itself does not abort AUTO/GUIDED autonomy.

Retained mission types are Waypoints, plotted Pipeline Survey, Ruth Survey and 5–2–5. The Chris handoff concepts are registered as explicit additional mission types rather than replacing those semantics:

- Chris Simple Dive (`dive_here` compatibility key): travel 1 m along the captured heading, DVL-altitude dive with a pressure-depth limit, eight headings (one full turn), surface, disarm and mission-done inhibit. A successful Competition run publishes one randomly generated two-color `ResourceDeliveryRequest` (resource first, delivery circle second) through the existing durable RoboCommand outbox. Practice runs display the generated result but do not report without a verified run.
- Surface Route: up to 15 surfaced geographic legs.
- Dock → Pipeline → Dock: outward surfaced route, bounded DVL dive/spin, reverse route and captured dock return.
- Pool Dive Test: operator-entered pressure depth, eight headings, surface and disarm.

### Headless SE / RoboCommand operation

The production core launches one lightweight `waypoint_mission` instance for
Chris Simple Dive. This is an adaptation of prequal's SE gesture into the
production DiveSpin/waypoint executor; it does not run Chris's radio process,
component IDs, GUI, or parallel transport stack.

1. The process waits without claiming autonomy. SE must be observed low and
   then high on RC channel 9, so booting with a held-high switch cannot start it.
2. After the edge, the mission uses fresh Cube local position, velocity and
   attitude directly. It does not require the GUI, Load/Prepare, RFD, GPS,
   navigation-supervisor commissioning, laptop presence, or a healthy RF link.
3. It selects GUIDED, arms and captures the current pose/heading, then enters a
   real Cube POSHOLD. The existing competition heartbeat consequently reports
   UUV1 as `STATE_AUTO` while it waits for the verified Wi-Fi RoboCommand
   RunStart on `/graey/comms/status`.
4. The exact matching RunStart switches the Cube back to GUIDED before the
   adapted 1 m / dive / spin / surface executor advances. The explicit switch
   avoids treating the intentional POSHOLD readiness state as pilot takeover.
5. A fresh onboard-run lease tells the existing watchdog not to disarm solely
   because RF disappears after SE. This exception cannot clear an existing
   inhibit, grant arming/ESC power, bypass SA, hide FC/Cube telemetry loss, or
   survive a dead mission process. SA/pilot takeover, Cube-control collapse,
   bounded-stage timeout and ordinary mission completion retain their existing
   behavior.
6. After exit the core respawns a new waiting process. It again requires an SE
   low-to-high edge before another run.

The headless plan retains the current Chris defaults: 3.5 m DVL altitude above
the floor with a 5.0 m maximum relative pressure-depth change. Missing bottom
lock at that bound follows the existing safe behavior: surface without spinning.
The mission start never depends on RFD; the normal team OCS/Wi-Fi path remains
the sole RunStart and RoboCommand report path for this mode.

The Cube remains the GUIDED/POSHOLD motion controller and MAVLink relay; the Jetson remains the mission executor. MAVProxy remains the sole Cube serial owner. The retired Chris component-199 GUI/protocol and duplicate recording owner are not revived.

## Navigation and recording

Start Navigation uses the same Auto/IP/RFD selector and calls the existing preparation/calibration service. It never arms or starts a mission.

The Record page uses the same selector for camera, sonar and rosbag start/stop. Recording is vehicle-owned and continues through laptop-link loss. ROS topic discovery and saved-session browsing remain WiFi/tether setup operations; routine RFD status omits the potentially large topic catalogue. RFD carries bounded acknowledged commands and compact active-session results, not a new broad telemetry stream.

## RFD behavior

The standard truthful UUV RXL heartbeat remains the normal 2 Hz liveness/status source. The laptop decodes and validates that heartbeat before using it as a GRAEY route indication. Private mission status remains change-driven with a 5 s keepalive when heartbeat is available (1 Hz only when the standard heartbeat is withheld because measured fields are incomplete). Unknown/foreign TUNNEL traffic no longer marks GRAEY connected.

## Validation and remaining field work

Local Windows regression on 2026-11-12: **524 tests passed, 5 expected skips**. This includes loopback TCP/UDP fleet transport, RoboCommand integration, standard heartbeat decode, IP-prepared/RFD-RunStart admission, bounded recording status, all existing navigation/recording tests, and new pure-logic tests for depth limits, DVL floor handling, abort/kill, pilot takeover and dock-return routing. `missions.js` and `record.js` also pass Node syntax checks.

No live vehicle, water, modem airtime or firmware parameters were changed by this implementation. Before competition acceptance, deploy with GRAEY killed/disarmed, then test WiFi-only, RFD-only, Auto failover, IP Prepare followed by RFD RunStart, recording start/stop on both links, and a simultaneous Crusader/GRAEY radio soak while measuring offered bytes/s and bad frames.
