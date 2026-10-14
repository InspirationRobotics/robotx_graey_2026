# Graey sonar session — 2026-10-10 (Ruth)

Pool session with the Ping360 pipeline sonar. This covers what I changed on the sub, how to use it, and what I measured.

## Summary

- **The sonar map changed (sonar files only).** It now has a draggable height band, a fade switch, a sub's-track switch, and zoom/drag. "Combine looks" and the floor cut box are gone.
- **The changes are on Graey (`5bada65`) but not on GitHub yet.**
- **The Ping360 misses pings only while ESC power is on.** With ESC power off it missed 0 of 16,297 pings; with it on, 1.8%. Suspect electrical noise or a power dip from the ESCs/thrusters. Each miss freezes the sonar for 4 s.
- **The sonar does see the pipe** at about 1–2 m. In the 0.9–1.2 m deep pool, surface and floor reflections make the map messy, so the settings matter (below).

## What I changed on the sub

| Merge on Graey | What |
|---|---|
| `8dc523e` | Classic map back as the default; "combine looks" behind a checkbox (morning) |
| `3364bd6` | **Fade old crumbs** checkbox. Off keeps every crumb until Reset |
| `393d381` | **Height band**: only echoes between two heights (from the sonar) become map crumbs. Drag its two lines on the Radar tab, or type them on the Map tab |
| `8a23c7e` | **Combine looks removed** (confusing, no benefit on real data). Added a **sub's track** switch, and the map now **zooms** (scroll or pinch at the mouse) and **drags** |
| `5bada65` | **Floor cut box removed** from the Map tab, and the floor cut is off by default. `pipeline_mission` still sets `/floorcut` itself for its harbor scans |

Files touched, all sonar-only: `api/sonar/crumbs.py`, `api/sonar/webview.py`, `api/sonar/outlines.py`, `tools/sonar_map.py`, `tools/sonar_map_replay.py`, `api/navigation/pipeline_mission.py` (reads plain crumbs), `sim/*`. Deleted: `api/sonar/looks.py`.

Sonar code reaches Graey by pushing a side branch and fast-forwarding:
1. From a laptop: `git push graey@graey.local:robotx_ws/src/robotx_graey_2026 <branch>:refs/heads/sonar-incoming`
2. On Graey: `git merge --ff-only sonar-incoming`

The sonar tools run from `src/`, so restarting the map picks up changes. No colcon build is needed.

## Using the sonar map

GUI → **Sonar** tab → **Start map**, then **reload page**. The Radar tab shows the raw sweep; the Map tab shows the top-down crumbs. **Record** and **Mark** save sweeps, pose and notes under `sonar_data/rec_<time>_<note>/`, which can be replayed on a laptop with `tools/sonar_map_replay.py`.

**Pool settings that worked** (pipe hanging about 0.3 m below the sub):

| Setting | Value | Why |
|---|---|---|
| range | 6 (or 8) | Longer ping. Range 4 → 6 made echoes about 15–20 brighter; above 8 only the background rose |
| sector | 160 → 230 | The pipe sits below sonar level |
| height band | about −0.65 to −0.35 m | Brackets the pipe. Drag the lines so the pipe's dot sits between them, and keep the lower line above the floor |
| threshold | about 60 | The water inside a tight band is dark (about 20), so a faint pipe still clears it |
| fade old crumbs | off | A pipe heard in 2 of 3 sweeps builds up |

**Hold one depth (ALT_HOLD).** The band is measured from the sonar, so it moves with the sub.

## What I measured

1. **Ping360 misses follow ESC power.** Across every map run on Graey today: ESC power off, 0 missed out of 16,297 pings. ESC power allowed, 377 missed out of 20,505 (1.8%). The pattern holds inside single runs.
   - A missed ping leaves the sweep waiting 4 s (brping's `transmitAngle` timeout).
   - Not caused by the recorder, sub motion, or CPU load (`sonar_map.py` uses about half a core).
   - Not yet confirmed with a deliberate on/off test.
   - **Hardware to check:** does the Ping360 share power with the ESCs? Do its power and USB cables run next to thruster or ESC wiring? Would a ferrite on the USB cable help?
2. **Tether disconnects during testing.** `TETHER_LOST` latched four times (19:05, 19:13, 19:14, 19:20), each time disarming the sub and cutting the ESCs. The cause was a loose physical connector.
3. **Pipe detection:**
   - **Oct 4 range test:** about 173 at 1.0 m, 112–148 at 1.25 m, 128–141 at 1.75 m, 116–133 at 2.25 m.
   - **Oct 4 heading test (±30°):** the pipe never vanished; its peak went from about 130 to about 80.
   - **Today:** strong echoes of 130–217 about 0.7–1.3 m to the side and 0.4–0.5 m below the sonar (18:17–18:19, 19:18). At about 25° off parallel, around 94 (5× the background).
4. **Pool clutter.**
   - In one run aimed upward (120 → 200), 77% of crumbs were drawn above the water surface. Those are surface reflections.
   - A strong echo is often heard on many angles at the same range (a median of 10 of 31 angles), so one object draws an arc.
   - The pool floor rises into the pipe's height band beyond about 2.5 m.
   - Walls are always the brightest thing, so pick by "nearest steady cluster at pipe height", not "brightest".
5. **Acoustic shadow** (suggested by the USV lead): no shadow behind the pipe was visible in the pool. The beam is 25° wide fore-aft and shallow water fills the shadow. It's worth checking at the lake, where the pipe stands 1–2 m above the bottom.

## Next steps

- **Gate 0:** sub still with the sonar running, ESC power on 2 min, then off 2 min. This confirms or clears the ESC noise.
- **Software:** shorten the 4 s missed-ping wait to about 1 s, with a retry (not built yet). Make the pool settings the map's startup defaults.
- **Map accuracy:** tape-measure the pool, drive a straight pass along a wall and back, and compare against the map.
- **Gate 2:** a slow straight pass along a straight pipe section at about 1 m, then back, with the pipe position taped. Pass if the line is within ±0.2 m and the two passes are within 0.3 m of each other.
- **Lake:** treat it as a test and data trip. Run a 10-minute go/no-go first (sub still beside the pipe). Drive manually and check for the pipe's shadow on the bottom.
- Push the sonar changes to GitHub.

## Data

All of Graey's `sonar_data/rec_20261010_*` and `map_20261010_*` folders. All of today's recordings (14:37 → 19:23) are also copied to Ruth's laptop. At 19:36–19:42 there is a range test (4 / 6 / 8 / 12 / 8, sub still, ESC off) with Marks, inside `rec_20261010_192303`.
