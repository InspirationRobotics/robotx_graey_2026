# GUI point selection uses native Cube Go-to

Deployed 2026-10-25 after the user confirmed QGC Go-to works and requested the same behavior from the GUI. The user briefly cancelled, then explicitly requested continuation.

On the main navigation page, click a point and press **Go to selected point**. The GUI sends native MAVLink COMMAND_INT / MAV_CMD_DO_REPOSITION (192), with CHANGE_MODE flag, global integer latitude/longitude, and the current Cube GLOBAL_POSITION_INT altitude. Speed is unchanged (-1); yaw is unspecified (NaN). Cube handles GUIDED travel and arrival. It does not autoarm or automatically disarm on arrival. There is no Jetson waypoint process, automatic north alignment, preparation sequence, alignment corridor confirmation or per-sensor startup wait in this GUI action. Manual heading correction and the existing EKF data path are unchanged.

The GUI's **Take control / MANUAL** button sends the existing MANUAL mode request. It does not disarm. Pilot/kill/connection authority is unchanged. Native Go-to has no added target-distance cap; the existing optional distance-entry helper still offers its previous 0.3–20 m range, while map clicks send their selected coordinates directly. The old unregistered pool screenshot remains view-only for live commands.

## Implementation

`api/gui/native_goto.py` queues HTTP requests for the GUI's existing ROS/MAVLink pump, through Link and the sole serial-owner MAVProxy. No HTTP worker writes serial. `/api/waypoint` accepts `goto` with a `point` and optional `request_id`; duplicates reuse the queued request. `goto_stop` requests MANUAL. Native status appears as `native_goto` in the normal waypoint GET response.

Actual Cube COMMAND_ACK results are shown as accepted, rejected or awaiting acknowledgment. An ACK timeout reports uncertainty without changing vehicle mode or retrying automatically. The GUI does not infer physical arrival from command acceptance. Current Cube mode/arming state appears alongside status. Subsequent map points can be selected while native Go-to is active. The prior Jetson preview/start endpoints remain for existing callers; the main GUI button no longer invokes them.

Pinned local ArduSub 4.7.2 source inspection confirmed DO_REPOSITION supports mode change and global destination handling. Its location parser requires finite altitude, so the GUI uses Cube's current global altitude to retain the current depth/altitude datum. No GPS, DVL, VN, covariance, heading qualification or manual-alignment prerequisite was added to native Go-to; Cube decides whether to accept the navigation command.

## Verification and deployment

55 affected local Python tests plus waypoint JavaScript passed. Onboard: 6 native Go-to tests, 17 navigation view, 3 Cube geographic frame, 23 bridge transport, 5 heading correction, 1 native QGC forwarding and 10 calibration job tests: **65 passed**. A real pymavlink packet serialization check verified COMMAND_INT, frame 5, command 192, mode-change flag and exact E7 coordinates using an in-memory sink, with no socket or vehicle transmission. Tests cover one command per request ID, explicit retargeting, Cube rejection, absent/wrong-destination ACK, MANUAL takeover, no arming/disarming, coordinate input and GUI routing without preparation or mission launch.

Deployment package: `output/gui-native-goto-20261025.zip`. Runtime changes are `native_goto.py`, `gui_node.py` and `tools/waypoint.html`. Backup: `/home/graey/deployment-backups/gui-native-goto-20261024-035550` (vehicle clock), including before/after JSON. Initial onboard tests exposed missing source fixtures in the package; these were added. A repeated Docker copy initially nested the directory; copying its contents corrected the test stage. All final checks passed before restarting the GUI. The finish script's printed `gui-native-goto-test-repair` label was not the actual backup; the dated directory above was independently verified.

Source/installed equality and Python compilation were verified. Only the GUI process was restarted (PID 71433 to 83013). ROS core, MAVProxy, Cube mode and navigation bridge were not restarted. Bridge instance `0aa75d8c-9d64-46b7-8e05-94b71adf6f10` and yaw offset -92.9341797927508 were unchanged; bridge remained enabled, transmitting, profile verified and in cube_ned coordinates.

Postdeployment read-only checks confirmed the native endpoint/status field, new served button, absence of automatic startup/alignment checkbox and `native_goto: null` (no native GUI command submitted). Both systemd services were active. Controller remained disarmed GUIDED, mission_claimed false, autonomous true as reported for its existing native mode. No Go-to, arming, takeover or movement command was sent for deployment testing. Physical GUI waypoint travel is not yet verified; the user had separately confirmed QGC Go-to works.
