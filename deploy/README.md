# Host configuration

Everything here lives **outside** the container, on the Jetson host. The container
holds the environment and the repo holds the code, but until these files are
installed the vehicle does not start on boot and the GUI's reboot and shutdown
buttons do nothing.

None of this is installed by `docker/Dockerfile`. It is the piece a fresh Jetson
flash silently loses.

## What each file does

| File | Purpose |
|---|---|
| `systemd/graey-mavproxy.service` | Starts the container, then MAVProxy inside it. MAVProxy owns the Cube's serial port. |
| `systemd/graey-ros.service` | Starts `core.launch.py` in the container, after MAVProxy. |
| `systemd/graey-mavproxy-restart.service` | Restarts MAVProxy when the Cube re-enumerates. Fired by the udev rule. |
| `systemd/graey-reboot.path` / `.service` | Watches for `logs/reboot.request` and reboots. |
| `systemd/graey-shutdown.path` / `.service` | Watches for `logs/shutdown.request` and powers off. |
| `systemd/graey-usb-watchdog.timer` / `.service` | Every 30 s, resets the USB hub if the Cube has vanished. |
| `bin/graey-usb-watchdog.sh` | The reset itself. |
| `udev/99-graey-cube.rules` | Detects the Cube re-appearing on the bus. |
| `logrotate/graey` | Caps the ROS log and the MAVProxy telemetry logs. |

## Install on a fresh Jetson

Run from the repo root, `~/robotx_ws/src/robotx_graey_2026`:

```bash
sudo cp deploy/bin/graey-usb-watchdog.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/graey-usb-watchdog.sh
sudo cp deploy/systemd/graey-* /etc/systemd/system/
sudo cp deploy/udev/99-graey-cube.rules /etc/udev/rules.d/
sudo cp deploy/logrotate/graey /etc/logrotate.d/
mkdir -p /home/graey/robotx_ws/logs

sudo udevadm control --reload-rules
sudo systemctl daemon-reload
sudo systemctl enable --now graey-mavproxy.service graey-ros.service \
                            graey-reboot.path graey-shutdown.path \
                            graey-usb-watchdog.timer
```

**Enable exactly those five and no others.** The remaining units are deliberately
triggered rather than enabled, and none of them has an `[Install]` section:

- `graey-mavproxy-restart.service` — fired by the udev rule's `SYSTEMD_WANTS`
- `graey-reboot.service`, `graey-shutdown.service` — fired by their `.path` units
- `graey-usb-watchdog.service` — fired by its `.timer`

Enabling a triggered unit directly would run it once at every boot, which for the
shutdown unit means powering the Jetson straight back off.

## Verify

```bash
systemctl is-enabled graey-mavproxy graey-ros graey-reboot.path graey-shutdown.path
systemctl list-timers graey-usb-watchdog.timer --no-pager   # NEXT must be < 30 s away
journalctl -u graey-ros -n 30
```

## How the GUI reboots the host from inside a container

`run_container.sh` mounts `/home/graey/robotx_ws` as `/root/robotx_ws`, so the two
paths are the same directory. The GUI, running in the container, writes
`/root/robotx_ws/logs/shutdown.request`; the host's `.path` unit is watching
`/home/graey/robotx_ws/logs/shutdown.request` and sees it appear. A file is the
whole bridge — the container is never given a way to command the host directly.

`graey-shutdown.service` deletes the request file *before* powering off. If the
file survived, the path unit would fire again on the next boot and shut the Jetson
down immediately.

## If the Cube is ever replaced

Its serial number is baked into **three** places, and all three must be updated
together or the vehicle will boot with no MAVLink at all:

- `deploy/udev/99-graey-cube.rules`
- `deploy/bin/graey-usb-watchdog.sh`
- `scripts/start_mavproxy.sh`

Read the new value from `ls -l /dev/serial/by-id/`. Never substitute a
`/dev/ttyUSB*` or `/dev/ttyACM*` number — those are reassigned across reboots.

The watchdog also hard-codes the hub's bus path (`1-2.2`). Confirm it in
`sudo dmesg` if the hub is rewired.
