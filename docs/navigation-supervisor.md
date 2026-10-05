# Navigation supervisor and dive reference

Implemented locally October 2, 2026. This is an observation-first implementation,
not a record of deployment or successful in-water commissioning.

## Ownership and data flow

```text
M9N on GPS2 ------------------------------> Cube EKF
Bar30 / Cube inertial sensors ------------> Cube EKF
DVL A50 -> dvl_node -- atomic sample --+
VN-100 -> vn100_node ------------------+-> nav_ekf_bridge -> ODOMETRY -> Cube EKF
                                                ^                         |
                                          one-time alignment              |
                                                |                         v
Mission intent -> Jetson navigation supervisor <- MAVProxy telemetry/Link
                         |                    |
                         |                    +-> source selection command
                         +-> status -> existing GUI backend
                         +-> saved dive reference -> mission coordinate offset
```

MAVProxy remains the sole serial owner. The supervisor uses component 196 on
loopback UDP 14557; existing node endpoints remain separate. The systemd unit
already launches the updated `scripts/start_mavproxy.sh`.

The supervisor never arms, disarms, resets a kill, restores ESC power, resets the
Cube origin, or writes EKF parameters. The optional mission adapter owns its
mission interruption response. Existing RC abort and pilot mode override retain
priority. GUI pages display status and have no navigation-control authority.

## Source profiles

| GUI label | Horizontal position | Horizontal velocity | Vertical position | Yaw |
|---|---|---|---|---|
| Underwater | Integrated external position | DVL external velocity | Barometer | VN-100 external yaw |
| Surface GPS | GPS | DVL external velocity | Barometer | VN-100 external yaw |

`params/navigation_sources.parm` contains candidate source sets. Vertical
velocity is not explicitly sourced; unused source-set velocities are not fused.
The Cube still uses its internal inertial propagation. These parameters have
**not** been applied to Graey by this implementation.

External position remains enabled underwater: a stationary ArduSub 4.5.7 SITL
experiment did not establish a valid local position at startup using only the
external velocity selection. Retaining the current position feed avoids treating
that unsupported assumption as a working configuration.

## States and gates

The supervisor runs at 10 Hz. It requires fresh Cube heartbeat, local position,
EKF health, DVL velocity/validity, VN-100 orientation, bridge health, calibrated
pressure depth, and mission intent. It waits one second for a healthy estimate
and stable selected-source report before continuing. Depth thresholds are
commissioning defaults: shallower than 0.15 m for two seconds confirms surface;
deeper than 0.40 m for two seconds confirms underwater. Between them it retains
the previous classification. A DIVE/UNDERWATER intent requests underwater aiding
even while the vehicle is still shallow.

GPS requires advancing receiver timestamps, a 3D fix, reported horizontal
accuracy greater than zero and at most 3 m, valid coordinates, displacement
consistent with fresh valid atomic DVL samples, and three seconds of continuous qualification. Reported
accuracy is a receiver estimate, not proof of the true position. EKF health adds
a separate consistency gate; selected-source telemetry is not proof of actual
GPS fusion. Innovation/log inspection is still required during commissioning.

| Situation | Behavior |
|---|---|
| Initializing/unhealthy/stale intent/depth | No switching or mission permission; report reason |
| Surface, GPS qualifying | Keep existing source; wait for qualification |
| Surface, qualified GPS | Recommend/select Surface GPS if all activation gates pass |
| Brief absence of new GPS measurements | Keep Surface GPS selection; allow at most 3 seconds of DVL continuation while other health checks pass |
| Explicitly rejected GPS or longer dropout | Revoke mission permission; request aligned Underwater external aiding if all health/configuration gates pass; remain there until GPS requalifies |
| GPS returns | Require qualification again; one good fix cannot renew the grace window |
| Dive requested | Require a new qualified surfaced GPS measurement, healthy Cube pose, selected Surface GPS and saved mission reference |
| Returning to Underwater | Align integrated external position once to Cube NED, wait for bridge acknowledgment, then command source change |
| Source command pending | Require accepted ACK and a fresh report of the requested selected source; permission withheld |
| Rejected/missing confirmation, alignment timeout, position jump | Latch fault; do not automatically retry/resume |
| Cube reboot/origin change or bridge restart | Invalidate permission/frame; operator revalidation required |

The bridge consumes one atomic DVL sample containing velocity and validity;
legacy DVL GUI topics remain. It rejects stale/replayed samples and invalid
attitude/rates. It withholds combined ODOMETRY when DVL or VN-100 fails, rather
than sending invalid DVL as zero velocity. This also withholds external yaw.
Independent heading-only transport is not implemented, so bottom-lock loss can
block surface navigation even with good GPS.

### October 3 telemetry-driven hardening (local, not deployed)

The GPS motion screen compares accepted geographic anchors over a rolling
10-second window against integrated DVL speed (path length, including vertical
motion, a conservative displacement bound), with a 2 m allowance. It requires
valid atomic DVL samples less than 0.5 seconds old and never uses invalid zero
velocity as evidence of stationarity. Rejected points do not replace trusted
anchors. Configurable ROS parameters: `gps_max_accuracy_m` (3),
`gps_motion_margin_m` (2), `gps_motion_window_s` (10). These are initial screening
defaults, **not fitted EKF noise parameters or validated pool tolerances**.
After a DVL gap the motion anchors clear and normal GPS dwell is required by
the supervisor following rejection. Constant GPS bias and sufficiently slow
drift cannot be distinguished from truth by this check alone.

An explicit bad fix receives no missing-measurement grace. A source change
still requires healthy external navigation, frame alignment, commissioning,
intent, ACK and source feedback. If those are absent, mission permission stays
blocked and the supervisor cannot promise GPS has been removed from fusion.
Switching is asynchronous: the M9N feeds the Cube directly, so the Jetson cannot
veto each individual GPS measurement before the Cube sees it. Existing alignment
uses current healthy Cube pose; it preserves continuity but does not undo an
already accumulated geographic error. No automatic re-zero or origin reset was
added. Cube innovation rejection remains necessary.

The GUI now withholds the blue global marker unless fresh local pose and EKF
horizontal health support it; raw reported global coordinates remain readable.
The raw GPS marker and display-origin capture require known accuracy <=3 m.
The supervisor page shows the GPS admission reason. These GUI changes do not
alter QGC or Cube fusion. See `gps-telemetry-review-2026-10-03.md` for evidence
and the remaining live commissioning requirements.

## Mission zero and GUI

On DIVE intent, the supervisor captures a new qualified GPS fix paired with a
fresh Cube local NED pose (receipt-time separation at most 0.5 s). It atomically
persists the reference before permitting the dive. This is receipt-time pairing,
not hardware clock synchronization. The reference belongs to a mission run and
vehicle-frame epoch; restored files are historical only.

```text
mission_NED = Cube_NED - captured_Cube_NED
Cube_target = mission_target_NED + captured_Cube_NED
```

The GPS point is marked on both Grid and Streets maps at `/navigation`.
`/navigation-supervisor` is a separate same-window page showing state, reasons,
readiness, source names, calibrated depth and reference. Existing sensor cards
are preserved. Display reset clears map tracks and waits for a new GPS fix; it
does not change the mission reference or Cube origin.

Existing prequalification missions use their established forward/right heading
frame translated to the captured reference when `navigation_supervised` is on.
Depth targets then also use the captured down offset. The existing `pos_server`
continues to expose Cube coordinates; external planner clients are not silently
converted. A Task 2 UAV-target receiver and full Task 2 mission are outside this
change.

## Activation and remaining commissioning

Core launch starts the supervisor with `active=False`. The standalone
`navigation_supervisor.launch.py` is an alternative; do not run a second instance
alongside core. Mission integration defaults to `navigation_supervised=False`.
Observation is not a global shadow mode: the bridge freshness fixes apply when
this code is run, including with the supervisor observing.

Before enabling source commands on a disarmed bench vehicle:

1. Back up/read back actual firmware and EKF parameters. Verify the candidate
   profile against installed firmware; reconcile yaw and DVL coordinate axes.
2. Identify which pressure message really carries Bar30 and calibrate
   `surface_pressure_hpa` at the water surface. Default zero deliberately blocks
   depth classification. Set water density for the test site.
3. Verify selected-source feedback. Optional
   `deploy/scripts/navigation_source_report.lua` emits `NAV_SRC` at 5 Hz using
   the Cube's actual selected-source getter. It requires working/enabled Lua
   scripting on that firmware. No script has been installed. Missing feedback
   leaves the source Unknown and blocks activation; ACK alone is insufficient.
4. Verify Cube NED and bridge external FRD axes agree, including magnetic/true
   north and yaw offset. Test one-time alignment/reset-counter behavior and pose
   continuity. Only then enable bridge `allow_alignment` and supervisor
   `vehicle_validation_complete` plus `active`. Parameter readback must match.
5. Validate source transitions, GPS loss/recovery, sensor interruption and node
   restart while disarmed, then controlled water tests with an operator.

The opt-in mission adapter requires explicit
`navigation_abort_policy=pilot_takeover` for non-dry supervised runs. Any loss of
permission after WAIT_NAV latches the mission stopped and repeatedly requests
MANUAL until the mode changes. It never automatically resumes, disarms on this
navigation interruption, or changes hardware kill state. **This conservative
policy also interrupts a running mission during a source transition that
withholds permission.** Seamless autonomous transition/hold behavior is not
commissioned; do not enable this adapter for an unattended Task 2 mission.

Further limitations: bridge process restart still starts its integrated pose at
zero; supervisor detection blocks mission permission but is not a guarantee that
no new odometry reached the Cube first. Unknown covariance remains in the legacy
ODOMETRY stream. These need vehicle/log testing before active deployment. A
healthy EKF flag and the current bounded position-continuity check are not a
complete statistical fusion test. Thresholds are configurable starting values.

## Validation evidence

Run `python -m unittest discover -s test -p 'test_*.py'` from the repository.
Tests exercise the production pure supervisor, reference persistence/conversion,
GUI display reset/reference separation, and production transport callback bodies
with mocked ROS/transport. The randomized supervisor test runs 20,000 ticks.
These tests do not simulate hydrodynamics, DDS, actual sensor noise or thrust.

Separate local evidence is in the workspace's `output/gps-sitl-2026-10-02/`:
official ArduSub 4.5.7 stationary SITL, synthetic ODOMETRY, no arming. It confirmed
local-position startup with external position and source-command acceptance;
selecting GPS with GPS disabled yielded an uninitialized EKF flag. Runtime timing
under tracing limited the experiment. It did not validate full transitions,
heading/frame alignment, real hardware, or in-water performance.
