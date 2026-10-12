# Graey sonar status — 2026-10-11 (Ruth)

Where the Ping360 pipeline sonar stands after the Oct 10–11 pool sessions: what I changed on the sub, what the recordings show, and what is left before the lake test. This replaces the summary in `sonar-session-2026-10-10.md`. Some of that doc's conclusions were wrong, as noted below.

## Summary

- **The sonar does hear the pipe, but the pool hides it.**
  - In the 22:20 recording the pipe shows as a strong echo (117–205) in 13 of 71 sweeps.
  - A no-pipe recording at the same settings never shows it.
  - Most map dots in the pool are not the pipe. They are the sonar's own ringing just past its blind zone, the floor, the surface and the walls.
- **The pipe was usually too close to be heard.** Half of its echoes sit right at the 0.75 m blind-zone edge. Following at 1.0–1.3 m should give a hit in most sweeps, about one every 20–30 cm along the pipe.
- **Graey runs my latest sonar code (`2b30056`); GitHub does not have it yet.**
- **The missed-ping freezes are fixed in software.** A missed ping now costs about 1 s instead of 4 s. Gate 0 had 0 misses.
- **Lake test: Sat Oct 17 or Sun Oct 18.** The checklist is at the end.

## What I changed on the sub (Oct 10–11)

All sonar-only files. Graey is at `2b30056` on `tether_toggle_beichen`.

| Merge on Graey | What |
|---|---|
| `3364bd6` | **Fade old crumbs** switch. Off keeps every crumb until Reset |
| `393d381` | **Height band**: only echoes between two heights (measured from the sonar) become crumbs. Drag its lines on the Radar tab |
| `8a23c7e` | "Combine looks" removed. **Sub's track** switch. The map **zooms** (scroll or pinch) and **drags** |
| `5bada65` | Floor cut box removed, and the floor cut is off by default |
| `818659c` | **Missed pings cost about 1 s, not 4 s.** The sonar waits at most 1 s for an answer (2 s for a sweep's first ping), and only accepts the answer for the angle it asked for |
| `afcd698` | **Little dots only** switch: tall, wall-like echoes don't become crumbs. The radar side panel says so when it's on |
| `2b30056` | **Outlines** switch removed. Messages on the Map tab stay 10 s, then fade. **Mark** now says `mark saved at <time>: <note>`, or `mark NOT saved: not recording` |

Deploying sonar code: push a branch to Graey as `sonar-incoming`, then run `git merge --ff-only sonar-incoming` on Graey. Then in the Sonar tab: Stop, wait about 5 s, Start map, and reload the page. No colcon build is needed.

## What the recordings show

### 1. Ringing past the blind zone looks like a pipe on the map
- **The cause:** the sonar can't hear anything closer than 0.75 m, and its ringing carries on to about 1.05 m, on every angle.
- **How strong:** usually 55–70, which is above a threshold of 60 in most pings.
- **Why it looks like a pipe:** it sits at the same distance on every angle, so it isn't tall enough for "little dots only" to remove. As the sub moves, it draws a line about 0.85 m beside the sub's own path, which looks like a pipe being followed.
- **The check:** a no-pipe recording (`rec_20261010_235617`) made the same dots at the same rate (85 vs 80 per sweep).
- **Correction:** I said earlier that the 22:20 dot band "traced the pipe". That band follows the sub's path.

### 2. The pipe is in the 22:20 recording (`rec_20261010_222035`)
- **Where:** about 0.65–0.9 m from the sonar, 0.2–0.4 m **above** it, to the left. The pipe was set slightly above the sub.
- **How often:** 36 strong echoes in 13 of 71 sweeps (1, 2, 36, 51, 53–55, 58–61, 63, 64), mostly while following closely.
- **In the no-pipe recording:** 0.
- **Why so few:** 53% of them sit at the 0.75 m blind-zone edge, so most of the time the pipe was closer than that and invisible.
- **Map settings can't pull it out in the pool.** I tried 12 combinations (threshold 60/100/115, two bands, little dots on/off). The no-pipe run always made as many dots, because the wall and surface echoes 1.0–1.4 m away sit at the same heights. Only distance separated the pipe: it was closer than the wall.

### 3. Other measurements
- **The floor straight below** shows as a bright spot (165–200) about 0.3 m past the DVL's floor reading, because the sonar sits about 0.3 m above the DVL. This was the same in every session.
- **An unexplained echo** at about 1.2–1.3 m and 0.37 m below the sonar shows in most recordings, with or without the pipe (0–84% of pings, depending on place). Candidates: the floor rising toward a wall, or the tether. Not resolved.
- **Box test:** one sweep (00:36:45 on Graey's clock) had an echo of 192 about 15 cm above where the floor should be. That's a hint, not proof. Hover with the DVL reading 0.9 m or more, so the box's top clears the blind zone.
- **Missed pings:**
  - **Gate 0:** 0 misses in 871 pings with ESCs off, 806 armed and still, and 259 armed and moving. So the earlier "ESC noise" theory is weakened.
  - A burst came back on Oct 11 (6 misses in a minute), cause unknown. Each one now costs about 1 s.
- **Hardware fixed during the sessions:**
  - The topside Fathom-X was replaced; it caused the tether drops.
  - The Ping360's USB-serial chip locked up; a full power cycle cured it.
  - The kill switch connector was loose.
- **Cleanest proof that the sonar sees the pipe:** the Oct 4 test with the sonar held still. Without the pipe, nothing; with the pipe, a solid echo at 1.31 m, strength 132–146.

## Tools

- **Replay page** (works in any browser): https://claude.ai/artifact/WxgyYP6jigWdiD2nuhsvGu
  - Map and radar for the pipe and no-pipe recordings, with marks.
  - Circles on the radar show which echoes became map dots.
  - The link is private until shared. A standalone copy is `graey_pipe_replay_2026-10-10.html`.
- **Data sheet for the 22:20 recording:** `pool_2026-10-11_inside_echoes.png`, `_inside_echoes.csv` (every strong echo) and `_inside_by_sweep.csv` (one row per sweep).
- **Bottom camera footage** (Oct 11, about 5 min): `camera_rec/bottom_cam_2026-10-11.mp4` plus a brightened copy. The pipe is visible about 25 s in, then the picture goes dark. It needs lights.
- **Viewing both cameras:** stop **Camera tracker** in the GUI's Nodes tab, then run `tools/cam_pair.py` in the container. It serves on port 8080, with DOWN on the left. Don't stop `graey-ros` for this; that stops the kill switch too.

## How to test in the pool now

1. Put the pipe away from the walls, raised off the floor if possible.
2. Follow it at **1.0–1.3 m from the sonar head** (the front of the sub), not 0.7 m.
3. Record. Add a Mark with a note at each change.
4. Record a **no-pipe pass along the same route** for comparison.
5. Replay it, point out the pipe on the radar, and make the data image. In the pool, a human label is the ground truth. The lake won't have this much clutter.

## Before the lake (Sat Oct 17 or Sun Oct 18)

- [ ] Pool sweeps at 1.0–1.3 m, labelled, each with a no-pipe pass
- [ ] `pipeline_mission` run in the simulator (`sim/`)
- [ ] Shadow run in the pool: the mission in dry run (its default; it never moves the sub) while I drive, then compare its picks with my labels
- [ ] Follow-by-distance behaviour for a short real follow in the pool (**not built yet**)
- [ ] Lake plan: a 10-minute go/no-go with the sub still beside the pipe, record everything, drive manually, look for the pipe's shadow on the bottom
- [ ] Push the sonar changes to GitHub
- [ ] Lights for the down camera, if we want camera footage

## Data

On Graey: `~/robotx_ws/src/robotx_graey_2026/sonar_data/` (`rec_*` recordings and `map_*` sweep logs), plus the camera stream in `~/camera_rec/`. Copied to my laptop: all Oct 10 recordings, `rec_20261010_235617` (no pipe), `map_20261010_221321` and `map_20261011_002553`. Graey's clock is wrong (it shows Oct 10 late evening), so its times are only good relative to each other.
