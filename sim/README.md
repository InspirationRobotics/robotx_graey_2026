# Graey software-in-the-loop

Runs Graey's flight controller and Jetson code on a laptop, with no hardware:

| Piece | In the sim |
|---|---|
| Cube Orange | ArduSub **4.5.7** SITL (same version), `vectored_6dof`, Graey's real `params/graey_4.5.7_autonomous_validated.parm` |
| MAVProxy | Graey's port layout (14551-14556), plus 14557-14559 for sim helpers and QGC on UDP 14550 |
| VN-100 + DVL | `sim/sim_sensors.py` publishes the same ROS topics from the simulator's true motion |
| `nav_ekf_bridge`, `gui_node`, `pos_server` | **Graey's real code, unchanged** |
| Ping360 | `sim/sim_sonar.py`: real beam shape (2 deg x 25 deg), pinged from the sonar's true position and tilt |
| Sonar map | **`tools/sonar_map.py` and `tools/pose_relay.py`, unchanged** (`sim/sim_sonar_map.py` swaps in the simulated Ping360) |

So the EKF dead-reckons from "VN-100 + DVL" through Graey's own bridge, exactly as on the sub.

**The simulated world** (`sim_sonar.py`): a flat seafloor at `WATER_DEPTH`, and the Task 2 pipeline
built from the handbook's parts list (3.5.5) - five straight sections joined by 45 deg elbows, not
all in one plane, ~4.9 m of 3" pipe 1.0-1.7 m above the floor, three tee legs to the floor, and
three ~0.15 m light boxes on it. Move it with `PIPE_N`, `PIPE_E` (m from home) and `PIPE_HDG`.
Graey has no GPS; `sim/set_origin.py` gives the EKF a latitude/longitude for its (0,0), which is
what lets QGroundControl draw the sub on the satellite map, submerged or not.

## Use

```bash
docker build -t graey-sitl sim/                     # once, ~10 min
HOME_LAT=<pool lat> HOME_LON=<pool lon> sim/run.sh  # WATER_DEPTH=12 for the harbor
```

- **QGroundControl** connects by itself (UDP 14550).
- **GUI:** http://localhost:8090 (status; Mission planner tab tracks via pos_server on 8081).
- **Waypoint test** (surface -> dive 0.5 m -> one GPS waypoint -> surface):
  `docker exec graey-sitl bash -c 'cd /root/robotx_ws/src/robotx_graey_2026 && PYTHONPATH=. python3 sim/waypoint_test.py --lat <lat> --lon <lon>'`
- **Sonar map:** http://localhost:8095/map (also the GUI's Sonar tab -> "reload page").
- **Fly-over test** (dives 1 m above the pipe, Starts the map, flies the pipe, grades the crumbs):
  `docker exec -e WATER_DEPTH=12 graey-sitl bash -c 'cd /root/robotx_ws/src/robotx_graey_2026 && PYTHONPATH=. python3 sim/flyover_test.py'`
  First run: 82 % of crumbs within 0.3 m of the pipe, all within 0.5 m (median 0.17 m).
- **Pipeline mission** (dive at the buoy, far scan, close scan, pick the pipe; watch the Map tab and QGC):
  `docker exec graey-sitl bash -c 'source /opt/ros/humble/setup.bash && cd /root/robotx_ws/src/robotx_graey_2026 && PYTHONPATH=.:$PYTHONPATH python3 -c "from robotx_graey_2026.api.navigation.pipeline_mission import main; main()" --ros-args -p dry_run:=false'`
- **Navigation check** (simulator truth vs Graey's EKF, side by side):
  `docker exec graey-sitl bash -c 'cd /root/robotx_ws/src/robotx_graey_2026 && PYTHONPATH=. python3 sim/truth_vs_ekf.py 60'`
- Logs: `docker exec graey-sitl ls /tmp/sim` · Stop: `docker rm -f graey-sitl`

## Sim-only parameter changes

Listed with reasons in `sim/sim_overrides.parm`. Everything else is Graey's real configuration.

## Found by the sim (Oct 2026)

- Re-sending an unchanged GUIDED position target every 3 s (as `mission_base.py` does) restarts
  ArduSub's position controller, integrator included; the buoyant simulated sub never left the
  surface. Sending once fixed it. Check on the real sub before changing `mission_base.py` - the
  sim's buoyancy may not match Graey's.
