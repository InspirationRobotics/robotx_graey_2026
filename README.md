# Graey UUV — RobotX 2026

Autonomy software for **Graey**, Team Inspiration's uncrewed underwater vehicle
(UUV) for the 2026 Maritime RobotX Challenge. Built from scratch on ROS 2 Humble.

Graey works alongside **Crusader** (USV) and **BabyDragon** (UAV). Its competition
job is Mission Task 2 (Infrastructure Survey & Repair). Its Proof of Readiness
(prequalification) run was submitted in August 2026.

> **Full documentation:** in the team Drive — the complete operating manual:
> hardware, wiring, procedures, troubleshooting, and design decisions. This README
> is just the quick reference.

## Architecture

- **Computer:** NVIDIA Jetson Orin Nano (JetPack 6.2). Code lives on the Jetson at
  `~/robotx_ws` and is bind-mounted into a Docker container named `graey` (ROS 2
  Humble + CUDA/TensorRT), which holds only the environment. Edit outside the
  container, build and run inside it.
- **Flight controller:** CubePilot Cube Orange running ArduSub, frame
  VECTORED_6DOF (8 thrusters). Owned by exactly one program — **MAVProxy** — which
  rebroadcasts MAVLink over UDP to the ROS 2 nodes and to QGroundControl. Nothing
  else may open the Cube's serial port.
- **Navigation (no GPS underwater):** Water Linked DVL A50 velocity + VectorNav
  VN-100 heading fused into the Cube's EKF; Bar30 for depth.
- **Perception:** two Luxonis OAK-D cameras, forward and downward. Status is shown
  on a NeoPixel strip driven by an Arduino.
- **Autostart:** two systemd units on the Jetson *host* bring everything up on
  boot — `graey-mavproxy.service`, then `graey-ros.service`. Normal operation
  needs no terminal at all.

## Quick start

Power the vehicle on and wait about a minute. Everything starts by itself; open
the control GUI in a browser:

```
http://graey.local:8090
```

QGroundControl needs no configuration — MAVProxy broadcasts to every subnet the
Jetson is on, so it appears on the router *and* over the tether.

To get a shell:

```bash
# over WiFi
ssh graey@graey.local

# or over tether: set laptop Ethernet to static 192.168.2.1, mask 255.255.255.0
# (Fathom-X powered, Ethernet -> adapter -> laptop, tether mated to penetrator)
ssh graey@192.168.2.2
```

If SSH warns `REMOTE HOST IDENTIFICATION HAS CHANGED` after a Jetson swap, clear
the old key with `ssh-keygen -R 192.168.2.2`.

## Running anything by hand

**Stop the services first.** They own the Cube's serial port and the LED serial
port, and a hand-started node will fail in confusing ways while they hold those.

```bash
# 1. host: release the hardware
sudo systemctl stop graey-ros graey-mavproxy

# 2. enter the container (colcon lives here, NOT on the host)
docker exec -it graey bash

# 3. terminal 1 — MAVProxy, first, because it owns the Pixhawk
cd /root/robotx_ws
./src/robotx_graey_2026/scripts/start_mavproxy.sh          # broadcast to all subnets
./src/robotx_graey_2026/scripts/start_mavproxy.sh <IP>     # or one address

# 4. terminal 2 — the always-on stack
cd /root/robotx_ws && source install/setup.bash
ros2 launch robotx_graey_2026 core.launch.py
```

Put the services back with `sudo systemctl start graey-mavproxy graey-ros`.

Because systemd normally owns the nodes, **their output goes to the journal, not
to a terminal**: `journalctl -u graey-ros`, or the GUI's Logs tab.

## Repository layout

```
deploy/                    systemd units, udev rule, logrotate, host scripts
docker/Dockerfile          container recipe (the environment)
docs/host-setup.md         Jetson host setup not covered by the Dockerfile
firmware/LED/LED.ino       Arduino sketch for the status LED strip
launch/core.launch.py      always-on stack: LEDs, kill switch, navigation, GUI
launch/prequal.launch.py   core stack plus the vision and position-server nodes
models/                    YOLO weights
params/                    saved Pixhawk parameter backups
scripts/                   start_mavproxy.sh, run_container.sh
tools/                     bench and diagnostic scripts; GUI and planner pages
robotx_graey_2026/api/
  gui/                     gui_node
  led/                     led_node, pixhawk_led_node
  navigation/              dvl_node, vn100_node, nav_ekf_bridge, pos_server,
                           mission_base, frames, missions
  pixhawk/                 mavlink.py (the Link class), kill_switch
  vision/                  camera.py, pole_tracker
```

## Nodes

| Node | Role |
|------|------|
| `dvl_node` | DVL A50 → velocity / altitude topics |
| `vn100_node` | VN-100 IMU → attitude / heading topics |
| `nav_ekf_bridge` | DVL velocity + VN-100 attitude → Cube EKF (MAVLink ODOMETRY) |
| `kill_switch` | RC kill / arm / mode switches; commands the hardware kill relay |
| `led_node` | status colour → Arduino strip |
| `pixhawk_led_node` | derives status (disarmed=red, manual=yellow, auto=green) |
| `gui_node` | serves the control GUI on port 8090 |
| `pos_server` | live position feed for the map planner |
| `pole_tracker` | camera → detected target bearing and range |
| `prequal_mission`, `prequal_mission_cv` | prequalification state machines (kept as reference) |
| `demo_mission` | instrumented GUIDED run for depth-hold diagnosis |

Every node in `core.launch.py` respawns on crash, because `ros2 launch` does not
restart a dead node on its own.

Mission nodes default to **`dry_run:=true`** (print the plan, never arm or move).
Set `dry_run:=false` only in the water.

## Safety

The kill is hardware, not software. A switch on the RC transmitter passes through
the Cube to a Pololu RC switch, a solid-state relay, and finally the coil of a
120 A contactor that feeds the thruster ESCs. It cuts power both when the switch
is thrown and when the transmitter loses power. A reed switch sits in series as an
independent, purely mechanical kill.

`kill_switch` additionally disarms the Cube and forces MANUAL, so the props cannot
start spinning again when power is restored.

## Environment

Rebuild the container from `docker/Dockerfile`; host-level setup is in
`docs/host-setup.md` and the units in `deploy/`. See the full documentation for
everything else.
