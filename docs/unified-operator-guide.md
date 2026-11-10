# GRAEY Missions, Sensors and Record

Local implementation, November 9, 2026. This replaces the separate Map / waypoint / 5â€“2â€“5 operator workflow. It follows the inspected Crusader mission catalogue, course plotting, OCS tier and recording conventions, adapted to the submarine. No vehicle installation, deployment or online push has been performed.

## Connect

The common laptop app/page now come directly from Crusader. See [current alignment and provenance](crusader-alignment-2026-11-09.md).

Double-click `tools/scripts/GRAEY_GUI.cmd`, or the parent workspace `GRAEY_GUI.cmd`, for localhost8152. Run parent `OCS.cmd` for localhost8099. On the same router, override the address with `GRAEY_GUI.cmd --graey GRAEY_WIFI_IP`; default graey.local. Set GRAEY_OCS_PORT for a different local OCS port. These follow Crusader's .venv/Python launcher workflow. The former --host/--ocs-port and custom PowerShell launcher instructions are superseded. Vehicle backend installation remains pending access and authorization. Offline UI has unavailable vehicle data; it does not simulate motion.

## Missions

- **Waypoints:** click the course to add ordered surface points. Coordinates can also be entered numerically, replaced, reordered or removed.
- **Pipeline Survey:** plot the pipeline start and end. GRAEY surfaces to the start, descends to the existing 4 m pressure-depth target, traverses the plotted line using waypoint mission logic, then surfaces. There is no automatic visual pipeline following. The depth is the existing mission setting, not DVL clearance.
- **5â€“2â€“5:** uses the existing fixed sequence: forward 5 m, descend 2 m, forward 5 m, surface, stop and disarm. It follows the starting heading in the Cube local frame.

The course boundary comes from OCS. The blue arrow shows the fresh Cube-derived geographic pose and attitude heading, including underwater when that estimate is available. Last-known pose and unavailable sources are marked stale rather than extrapolated. Without a course, geographic coordinate plotting and the vehicle pose still work; this is a north-up course/grid view rather than a satellite-image requirement.

Use **Fit course / route**, **Follow GRAEY**, drag to pan and scroll to zoom. Saved missions are stored on GRAEY and can be opened from the same page. Saving/opening does not execute them. Mission names are restricted to safe file names. The library lives under `~/robotx_ws/missions`, configurable with `GRAEY_MISSION_DIR`.

Check the route and supervision acknowledgement, then **Start mission**. Waypoints and pipeline routes pass through the existing reviewed-plan API, frame checks, controller permissions and commissioning gates. Stop uses the existing onboard mission abort interface. No new arming, ESC recovery or Cube serial path was introduced.

The Competition checkbox retains the user's competition-only RunStart policy. A competition mission can prepare and hold autonomously until OCS confirms the matched run; local training remains independent. Losing the browser/WiFi alone does not terminate an admitted onboard mission.

### Declare / End

These buttons are in the Missions header and operate the **shared fleet OCS run**, including other active vehicles. They call OCS's existing declaration/end endpoints; GRAEY does not create a separate competition run or synthesize RunStart. OCS's vehicle floor, course, UAV fence and pilot requirements remain authoritative. The page shows the active fleet, refusal reason and run state. A pilot callsign can be supplied when required; leaving it blank uses the OCS default.

The next-run tier editor is on Missions, not Communications. Load the current four OCS tiers before changing them, then send the complete set. OCS answers/refuses through the established tier protocol. Mutations are not automatically retried after a lost reply; check the displayed OCS state before repeating an uncertain action.

## Sensors

The Sensors tab combines numerical and visual measurements with the existing camera and sonar tools. It shows Cube roll/pitch/heading, a horizon, pressure depth history, DVL velocity vector/lock/clearance, VN-100 heading/IMU rates and acceleration, GPS fix/position/accuracy, Cube NED pose/velocity and EKF consistency metrics. Each source has its own age. Stale sources blank their live visuals/readouts; they do not remain green because another source is healthy.

Camera viewing remains opt-in. Existing camera and sonar start/stop tools are in Sensors; they retain device ownership. Starting a viewer is separate from recording. The GUI subscribes to the existing VN-100 IMU topic and never opens an additional sensor serial connection.

## Record

1. Start the camera service and/or sonar **map** tool in Sensors if you want those sources.
2. Open Record, name the session, and independently select **Camera stream (Down + Front)**, **Sonar sweeps**, and/or **Sensor topics / rosbag**.
3. Refresh/select the desired advertised ROS topics. Heavy image/point-cloud types are identified; the GRAEY sensor shortcut selects light sensor/navigation telemetry rather than control command topics.
4. Start recording and check each source's confirmed state. Partial failures are shown individually; an unavailable camera is not presented as a successful recording.
5. Stop recording and wait for finalization. Changing pages, stopping a mission or losing laptop WiFi does not itself stop the vehicle-owned recording session.

The recording coordinator runs slow service/disk operations off the ROS GUI executor. It uses Crusader's rosbag subprocess and measured growth helpers, with SIGINT finalization. It does not open a second OAK/sonar connection, change sonar settings or take over another operator's recording. Short unique ownership tokens account for the existing services' filename truncation. Changing ownership is detected before stopping a source.

Session telemetry/manifest/rosbag are saved under `~/robotx_ws/recordings`, configurable with `GRAEY_RECORD_DIR`. Camera and sonar files stay in their existing service directories; the manifest records their file/session names. The Record page lists saved session manifests. Displayed session-folder size/rate excludes camera/sonar files in those other directories. Disk reserve checks use the session filesystem's 2 GiB floor; separate storage for external services still needs its own monitoring.

New unified-page pipeline routes explicitly use operator-controlled recording. Historical plans can still explicitly use the older automatic recording adapter; the new page does not generate that setting. Recording is never a prerequisite for executing the plotted survey route.

The vehicle needs the ROS `sensor_msgs`, `ros2bag`, `rosbag2_transport` and `rosbag2_storage_default_plugins` dependencies declared in package.xml. Hardware camera/sonar recording and network performance remain to be commissioned when GRAEY is available.

## Local validation

The full GRAEY regression suite passes, including mission-library, recording ownership/disk guards and OCS proxy checks. Browser tests use the real laptop/vehicle TCP bridge with actual page assets and explicitly synthetic OCS/vehicle/device data. They exercise plotting, save/open, pipeline/5â€“2â€“5 selection, shared-run controls, tiers, independent recording, stale sensors and mobile layout. A separate test starts the actual ROS GUI node and its HTTP handlers. Another records 109 real ROS synthetic sensor messages and verifies a finalized rosbag/metadata; no camera, sonar, Cube or vehicle hardware was opened. Build/installed-file verification and final counts are recorded in `docs/validation/unified-operator-2026-11-09.json`.
