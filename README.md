# Graey UUV — RobotX 2026

Autonomy software for **Graey**, Team Inspiration's uncrewed underwater vehicle
(UUV) for the 2026 Maritime RobotX Challenge. Built from scratch on ROS 2 Humble.

Graey works alongside **Crusader** (USV) and **BabyDragon** (UAV). Its main
competition job is Mission Task 2 (Infrastructure Survey & Repair); it must first
pass a Proof of Readiness (prequalification) run.

> **Full documentation:** in the team Drive — the complete
> operating manual: hardware, wiring, procedures, troubleshooting, and design
> decisions. This README is just the quick reference.

## Architecture

- **Computer:** NVIDIA Jetson Orin Nano (JetPack 6.2). Code lives on the Jetson at
  `~/robotx_ws` and runs inside a Docker container named `graey` (ROS 2 Humble +
  CUDA/TensorRT). Edit outside the container, build and run inside it.
- **Flight controller:** CubePilot Cube Orange running ArduSub, frame
  VECTORED_6DOF (8 thrusters). Owned by exactly one program — **MAVProxy** — which
  rebroadcasts MAVLink over UDP to the ROS 2 nodes and to QGroundControl.
- **Navigation (no GPS underwater):** Water Linked DVL A50 velocity + VectorNav
  VN-100 heading fused into the Cube's EKF; Bar30 for depth.
- **Perception:** Luxonis OAK-D W (forward). Status shown on a NeoPixel strip
  driven by an Arduino.

## Quick start

```bash
# 1a. connect over WiFi
ssh graey@graey.local

# 1b. or over tether: set laptop Ethernet to static 192.168.2.1, mask 255.255.255.0
#     (Fathom-X powered, Ethernet -> adapter -> laptop, tether mated to penetrator)
ssh graey@192.168.2.2

If SSH warns "REMOTE HOST IDENTIFICATION HAS CHANGED" after a Jetson swap, clear the old key:
ssh-keygen -R 192.168.2.2

# 2. enter the container
docker exec -it graey bash

# 3. terminal 1 — MAVProxy (owns the Pixhawk; start it first)
cd /root/robotx_ws
./src/robotx_graey_2026/scripts/start_mavproxy.sh <YOUR_LAPTOP_IP>

# 4. terminal 2 — the nav + status stack
cd /root/robotx_ws && source install/setup.bash
ros2 launch robotx_graey_2026 core.launch.py

# 5. open QGroundControl on the laptop — it connects on UDP 14550
```

## Repository layout

```
docker/Dockerfile          container recipe (the environment)
docs/host-setup.md         Jetson host setup not covered by the Dockerfile
firmware/LED/LED.ino       Arduino sketch for the status LED strip
launch/core.launch.py      starts the LED + navigation nodes
params/                    saved Pixhawk parameter backups
scripts/                   start_mavproxy.sh, run_container.sh
tools/                     oak_view.py (camera), serial_probe.py
robotx_graey_2026/api/
  led/                     led_node, pixhawk_led_node
  navigation/              dvl_node, vn100_node, nav_ekf_bridge, prequal_mission
```

## Nodes

| Node | Role |
|------|------|
| `dvl_node` | DVL A50 → velocity / altitude topics |
| `vn100_node` | VN-100 IMU → attitude / heading topics |
| `nav_ekf_bridge` | DVL velocity + VN-100 attitude → Cube EKF (MAVLink ODOMETRY) |
| `led_node` | status colour → Arduino strip |
| `pixhawk_led_node` | derives status (disarmed=red, manual=yellow, auto=green) |
| `prequal_mission` | prequalification state machine (GUIDED waypoints) |

The mission node defaults to **`dry_run:=true`** (prints the plan, never arms or
moves). Set `dry_run:=false` only in the water.

## Status

- Sensors, status LED, camera, MAVProxy/QGC link — working
- VN-100 heading fused into the EKF — bench-validated
- Prequal mission state machine — dry-run validated
- In-water (pending): DVL position validation, GUIDED tuning, prequal run

## Environment

Rebuild the container from `docker/Dockerfile`; host-level setup is in
`docs/host-setup.md`. See the full documentation for everything else.
