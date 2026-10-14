# Sonar code updates — Oct 4, 2026

Two new features for the Ping360 on Graey: an **adjustable range on the radar
viewer**, and a **breadcrumb map** (manual mode, version 1). Both are deployed on
Graey and were tested in the pool. Neither is committed to git yet.

---

## 1. Radar viewer: range box

**Files:** `robotx_graey_2026/api/sonar/webview.py`, `robotx_graey_2026/api/sonar/viewer.py`, `tools/sonar_pole_test.py`

- The radar page has a **range box** (1–50 m). Type a value and press Enter or click **set**.
- A new range **restarts the scan within one ping**. It doesn't wait for the current sweep to finish.
- The old picture **stays on screen in grey**, moved to the new scale so every echo is still at its true distance. New pings paint over it in colour.
- Detections found at the old range are cleared, and the status line shows the range in use (`range=6m`).

```bash
cd ~/robotx_ws/src/robotx_graey_2026 && PYTHONPATH=. python3 tools/sonar_pole_test.py --device /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D2010VWF-if00-port0 --web --port 8095 --range 4
```

---

## 2. Breadcrumb map (manual mode v1)

A live top-down map of **every sonar echo** ("crumbs"). Each one is placed using where the sub was at the moment of its ping.

### How to run (one command, on Graey)

```bash
cd ~/robotx_ws/src/robotx_graey_2026 && PYTHONPATH=. python3 tools/sonar_map.py --device /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D2010VWF-if00-port0
```

Open **http://graey.local:8095/map**. The normal radar is on the **Radar** tab.
The tool only watches: it sends nothing to the sub. Hold depth in QGC and drive yourself.

### The Map tab

| Control | What it does |
|---|---|
| Start / Pause / Resume | Start makes where the sub is now (0,0). Pause freezes the crumbs but keeps tracking the sub. |
| Reset | Clears the map and makes a new (0,0) |
| range | 2–20 m. Restarts the sweep immediately. |
| sector start → end | Swept counterclockwise. 0 = right, 90 = up, 180 = left, 270 = down. Same value = full circle. Example: 270 → 0 is down-to-right; 340 → 20 is straight out to the right. |
| threshold | Echo strength (1–255) that makes a crumb. Applies from the next ping. |
| map area | How many metres across the map shows (zoom). Only changes the page, not the sonar. |
| follow sub | Keep the sub in the centre of the view |

- **Map frame:** y = forward along the sub's heading at Start, x = right.
- **Drawn on the map:** the sub (white arrow), its track (blue), the sonar slice (green), and crumbs coloured weak → strong.
- **Banners:** "press Start", "DVL lost", "turning fast", "paused" and "no VectorNav / DVL data".

### Rules the map follows

- **What counts as a crumb:** each ping is smoothed over 5 bins. Every stretch at or above the threshold, past the 0.75 m blind zone, gives one crumb at its strongest point (the middle of a flat top). Up to 5 per ping, strongest first.
- **Where a crumb goes:** the sonar sits **0.42 m ahead of the DVL** (measured). The beam is 25° wide front-to-back, so each crumb goes in the **centre** of the beam.
- **Fading:** when a newer sweep's beam actually looks at a crumb's spot (pointing at it, abeam, in range), the crumb fades. After 1 newer sweep it's half-faded; after 2 it's gone. A crumb's own sweep never fades it, and one sweep fades a crumb at most once.
- **Turning faster than 10°/s:** the ping is ignored, with no crumbs and no fading, because turns smear crumbs sideways through the 25° beam. Change the limit with `--max-turn`.
- **DVL lost (or no data):** no crumbs and no fading until the DVL is back.

### Where the position comes from

- **Heading:** VN-100 attitude. **Movement:** DVL velocity added up over time (dead reckoning). It does **not** use the Cube or its EKF.
- These readings only exist inside the `graey` ROS container. `sonar_map.py` starts a small relay, `tools/pose_relay.py`, inside the container automatically, the same way `graey-ros.service` starts the core launch.
- The relay forwards `/graey/vn100/imu`, `/graey/dvl/velocity` and `/graey/dvl/valid` as UDP packets to `127.0.0.1:14660` (the container uses host networking).
- The relay quits by itself when the map tool stops, even if the map tool is killed. Use `--no-relay` to run it by hand instead.

### Saved data

Every sweep, finished or cut short, is saved to `sonar_data/map_<date_time>/sweep_NNNNN.npz` with:
- the image, angles, ping times and metres per bin;
- the sub's pose at every ping (x, y, heading, valid);
- the range, sector, threshold, map state and sonar offset;
- whether the sweep was cut short.

This allows replaying a run on the Mac later.

### Options

`--port 8095` · `--range 4` · `--start 180 --end 0` (bottom half) · `--step 2` · `--threshold 100` · `--max-turn 10` · `--pose-port 14660` · `--no-relay` · `--save-dir DIR` · `--no-save` · `--down-gradian` · `--udp host:port` (pingproxy)

---

## 3. All files changed

| File | Change |
|---|---|
| `api/sonar/webview.py` | Range box and its limits. Map tab page, data feed and its settings/buttons. Radar ↔ Map tab links. |
| `api/sonar/viewer.py` | Old picture is kept in grey on a range change |
| `api/sonar/sweep.py` | Each ping's time is recorded (`Sweep.ping_times`). Step size fixed at the 0/360 seam (a 2° step there read as 358°). |
| `api/sonar/settings.py` | `SONAR_FWD_M = 0.42`, `SONAR_RIGHT_M = 0.0` |
| `api/sonar/pose.py` | **New.** VN-100 + DVL dead reckoning in the map frame; pose at any ping time; turn rate; UDP listener |
| `api/sonar/crumbs.py` | **New.** Echo → crumb maths, fading, turn and DVL rules |
| `tools/sonar_pole_test.py` | Range box on, instant restart on a new range |
| `tools/sonar_map.py` | **New.** The map tool |
| `tools/pose_relay.py` | **New.** Relay inside the ROS container |

(`api/sonar/` = `robotx_graey_2026/api/sonar/`.)

---

## 4. Testing

**On the Mac, with a fake Ping360 (real beam shape) and a fake sub feeding the real tracker.** All checks pass:
- **Position tracker (16):** forward, turns, sliding sideways, tilt, crossing north, DVL lost, turn rate.
- **Crumb maths (32):** geometry at every heading, blind zone, threshold, sonar offset, fading rules, turn rule.
- **Map tab end-to-end (27):** every button and setting, rejection of bad values, saving.
- **Relay (9):** topics, data reaching the map, auto-start and auto-stop (including when the map tool is killed), failure warnings.
- **Radar range box (25) and instant restart (10).**
- **Accuracy on fake data:** crumbs from a straight pipe land within ~2 cm of it. At a 45° bend they're up to ~0.37 m off. That's the 25° beam with the "centre of the beam" rule.

**On Graey (Oct 4, pool).** Before the copy, I checked Graey's files to make sure nobody else had edited them. After the copy, the files matched byte for byte and imports worked on the Jetson and inside the container.
- **Pool test:** the pool walls were scanned with sector 340 → 20, range 8 m and threshold 120. They show clearly as lines of crumbs. Slower driving and more passes should sharpen them.

---

## 5. Known issues and limits

- **dvl_node stalled.** It stayed connected to the DVL but stopped publishing, while the DVL itself was sending valid data at 1.33 m altitude. Restarting the nav group in the GUI fixed it. The cause is unknown.
- **Beam smear.** Crumbs smear where something crosses the 25° beam at a slant: pipe bends, slanted walls, corners. Expect rounded corners.
- **Tilt not corrected.** Crumbs use heading only. If the sub sits tilted, crumbs can be off by up to ~10 cm. The VN-100 tilt could be used to correct this.
- **Crumb spacing.** Spacing along the track = speed × sweep time. Use about 0.1 m/s and a narrow sector for dense crumbs.
- **Before Start.** The track is drawn before Start is pressed, which can look like mapping when it isn't.

## 6. Not done yet

- Commit to git (`pipeline-sonar` branch).
- Map the mock pipe; replay runs on the Mac to tune the threshold.
- Version 2: highlight crumbs that several sweeps agree on, and draw a bending path through them. Then auto mode.

## Undo on Graey

Backups from Oct 4 are in `/tmp/sonar_bak_oct04b/` (`sweep.py`, `settings.py` and `webview.py` from before the map) and `/tmp/*.bak_oct04` (from before the range box). `/tmp` is cleared on reboot; the originals are also in git.
