# Task 2 mission, step by step (Oct 24, updated Oct 25)

What `tools/task2.sh <part>` runs, in order. For each step:
- **Does**: what the sub does.
- **Uses**: the settings, with the value `task2` uses. A name like `pool_alt_m` is in `SETTINGS` at the top of `pipeline_mission.py`. An ALL_CAPS name is a fixed constant in the code.
- **Next when**: the exact condition to move on.
- **Otherwise**: what happens when that condition isn't met.

Line numbers are in `robotx_graey_2026/api/navigation/pipeline_mission.py`, unless marked `mission_base.py` or `led_boxes.py` (`robotx_graey_2026/api/vision/`).

**Which parts run which steps:**

| Part | Steps |
|---|---|
| `search` | 0–9, then stops after the line-up (`stop_after: align`): holds 5 s, surfaces |
| `follow` | 0–2, then 9–11. No search; the pilot leaves the sub over the pipe |
| `survey` | 0–12, with a 5 s hold over RED (13) |
| `repair` | 0–12, with centring on RED (14) |
| `full` | as `repair`, but surfaces at the end (15) instead of POSHOLD |

---

## 0. Before the mission: `tools/task2.sh`
- **Does:**
  1. If the sonar map isn't up on port 8095, it starts it through the GUI's launcher (port 8096).
  2. If the cameras aren't up on port 8080, it starts them the same way.
  3. For a real run, it asks you to type `y`.
  4. It records the cameras to `~/camera_rec/task2_<part>_<date>_<time>.mjpg` for the whole run, and stops the recording however the mission ends.
- **Next when:** the sonar map answers within 30 s, and you type `y` (not needed with `dry`).
- **Otherwise:**
  - No sonar map: it stops.
  - No cameras: it stops for `survey`, `repair` and `full`. `search` and `follow` carry on without video.
- The mission records the sonar itself, in `sonar_data/rec_...`.

## 1. Start: WAIT_NAV, then ARM (`mission_base.py:308`, `tick`)
- **Does:**
  1. Waits for a position and heading from the Cube.
  2. Takes the current depth as "the surface" (`surface reads X m`).
  3. Sets GUIDED and arms. With `dry`, nothing is sent.
- **Next when:** the Cube has given a position and heading.
- **Always, from here to the end:**
  - If the pilot changes the flight mode, the mission exits at once without disarming (`mission_base.py:337`).
  - Any one step longer than `phase_timeout` **1200 s** makes it surface (`mission_base.py:354`).

## 2. Dive (`reached_depth`, line 388)
- **Does:** straight down over the start at `ALT_DZ_SPEED` **0.1 m/s**, until the DVL reads `pool_alt_m` **4.0 m** to the bottom. It uses the median of the last 3 s of DVL readings. If the bottom is already within 4 m, it doesn't move.
- **Uses:** `pool_alt_m` 4.0. Fixed safety limit `ALT_MAX_DEPTH_M` **6 m** below the start, for the whole mission.
- **Next when:** the target depth is settled (within 0.05 m) and the sub is within 0.10 m of it.
- **Otherwise:**
  - **No DVL reading:** it holds. After `ALT_DIVE_WAIT_S` **20 s** it searches from where it is.
  - **At the 6 m limit:** it searches from there, with a warning.
- Then TO_GATE: there is no gate. It moves on within 0.10 m of the start point.

## 3. Map setup for the search (`search`, line 886)
- **Does:** sets up the sonar's search view, resets the map, and starts the sonar recording.
- **Uses:**

  | Setting | Value |
  |---|---|
  | `pool_range` | 7 m |
  | `pool_sector` | 225–315 (45° either side of straight down) |
  | `pool_far_m` | 7 m |
  | `floor_filter` | 0.75–2.25 m |

  - Fixed: no near cut beyond the floor filter's own **0.45 m** blind zone, and no distance limit on picks.
  - **The floor filter, per sweep:** it draws the bottom line, then keeps only echoes that are 0.75–2.25 m above it, at least 3 times their surroundings, and more than 0.45 m below the sonar.
- **Next when:** the map is running and reads (0, 0) at the sub.

## 4. Search stops (`stop turn` line 930, `search_after_stop` line 1662)
- **Does:**
  - Makes `ceil(scan_turn_deg / search_step_deg)` stops: 180 / 25 gives **8 stops, 22.5° apart**.
  - At each stop: it turns to the heading, waits until still (within 3°, turning slower than 2°/s, for 2 s), then waits for `sweeps_per_stop` + 1 = **3** finished sweeps.
- **Next when** (`stop_blob`, line 1644): some dot has others within `blob_m` **0.15 m**, from at least `blob_sweeps` **2** sweeps.
- **Otherwise:** next stop. After the last stop: `nothing passed - surfacing`.
- **All through the search** (`alt_guard`, line 1395): if the DVL reads less than `SEARCH_MIN_ALT_M` **2.5 m**, it rises at 0.1 m/s until it reads 2.6 m.

## 5. Closer look (`search_after_stop` line 1662, `look_judge` line 1698)
- **Does:** four more stops at −20°, −10°, +10° and +20°, then waits `SETTLE_S` **6 s**.
- **Next when:**
  - a shape whose line passes within `LOOK_NEAR_M` **0.5 m** of the promising dots scores at least `look_min_score` **0.2**, with at least 3 dots along it.
  - **Densest spot** (`dense_spot`, line 1745): the dot with the most others within `DENSE_M` **0.15 m**, plus the shape's direction there (toward its longer part) and the pipe's height.
- **Otherwise:** on with the stops.
- **Then:** the sonar switches to looking straight down (the follow view below) before moving.

## 6. Over the pipe (`go_over`, line 1281)
- **Does:**
  - Moves at 1.5 × `follow_speed` = **0.12 m/s** so the **sonar**, 0.42 m ahead of the sub's centre, is over the densest spot.
  - **Lines up while moving:** turns to face the closer look's direction at that spot, no faster than **5°/s**.
- **Next when:** sonar within **0.15 m** of the spot, heading within **8°**, the target has stopped moving, and speed under **0.05 m/s**.

## 7. Look-down check (`over_check_step`, line 1346)
- **Does:** holds still and looks straight down for `OVER_CHECK_SWEEPS` **2** sweeps, for pipe dots at its height (±0.3 m), within `OVER_CHECK_M` **1.0 m** either side of the sonar.
- **Next when:**
  - **The pipe is at least `OVER_SLIDE_M` 0.15 m to one side:** it slides across over it (step 6 again), then goes on.
  - **Under the sonar:** it goes straight on.
  - Either way, the pipe's height measured straight down replaces the search's.
- **Otherwise:** no pipe seen: it goes on anyway, and the walk decides.

## 8. Down to the pipe (`walk_down_begin`, line 1416)
- **Does:** if the pipe is more than **1.2 m** under the sonar, it goes straight down at 0.1 m/s until the pipe is `WALK_PIPE_BELOW_M` **1.0 m** under it.
- **Next when:** within 0.1 m of that depth.
- **Otherwise:** it stops early if going lower would bring the DVL within **0.5 m** of the bottom.

## 9. Line-up walk (`align_walk_step` line 1918, `align_end` line 1992, `align_turn_now` line 2017)
- **Does:**
  1. Looking straight down, with `follow_range` **6 m** and `follow_sector` 240–300, it walks **straight** along the pipe's direction (no sliding sideways), at most `align_walk_m` **0.6 m**.
  2. It fits a line to the pipe dots as they come in.
  - It never goes further than (finished sweeps + 1) / 6 of the walk, so it always gets **5 sweeps**.
  - **Which dots count:** within 0.6 m of the walk's line (0.3 m of the fitted line once there is one), and within 0.12 m of the pipe's height.
- **Next when, whichever comes first:**
  - **The pipe is `align_turn_m` 0.4 m off the sonar's middle**, with a line through at least 3 dots from 3 sweeps over 0.25 m: it turns by the line's angle, the sonar over the pipe where it is now. If the walk has gone past the last place the pipe was seen, it goes back only to that place. Then it follows (step 11).
  - **The walk has gone 0.6 m:** the same small turn, then it follows.
  - **The pipe is 0.55 m off with fewer dots:** it turns by the best line it has.
- **Otherwise:**
  - **Pipe seen only near the start:** it walks the other way, once.
  - **No line:** `not the pipe after all`. It goes back up to the search depth and on with the next stop.
- **With a `pipe_end` model** (`end_model`, or `light_detector: yolo`): if the camera sees the pipe's end ahead, it walks the other way at once.

## 10. Start following (`follow_map` line 2106, `follow_anchor` line 2183)
- **Does:**
  - `survey`/`repair`/`full`: works out which way along the pipe gets closer to the buoy (where the mission started). If that's behind, it turns 180° and backs up so the sonar is over the same point.
  - Resets the map in the follow view.
  - The walk has just seen the pipe here, so if it's missing from the start, that counts as lost (step 11), not as "never seen".
- **Next when:** the map runs from (0, 0).

## 11. Following (`follow_step_once`, line 2228)
- **Pipe dots:** within `follow_gate_m` **0.3 m** sideways of the pipe and `follow_gate_up_m` **0.2 m** of its height, from 1.2 m behind to 0.4 m ahead of the sonar.
- **Direction:** at least 4 dots over 0.3 m → fit a line; if within **35°**, move the direction 40% toward it.
- **Moving:** along at the smaller of `follow_speed` **0.08 m/s** and 0.3 m per sweep, sideways toward the pipe at most 0.08 m/s, turning at most 5°/s.
- **Depth** (`follow_depth`, line 2463): keeps the pipe `follow_pipe_below_m` **0.7 m** under the sonar, give or take `follow_pipe_tol_m` **0.15 m**. Moves at 0.05 m/s, never more than 1.5 m from where following began, never within 0.5 m of the bottom.
- **Pipe missing for more than one sweep:** back over where it was last seen, and waits.
- **The end, seen by the camera** (`end_model` **models/pipe_end.pt**, or `light_detector: yolo`; `pipe_end_seen` line 2914):
  - The pipe has gone from under the sonar, and the camera has seen a pipe end scoring at least `end_conf` **0.25** in the last `END_SEEN_S` **3 s**, near the middle across (`END_MID_X` 0.6) and not behind (`END_BEHIND_Y` 0.3). That's the end: no 30 s wait, no search.
  - `pipe_end.pt` (Oct 25, trained on Ruth's 333 labelled Oct 24 frames): on held-back frames, 0.25 found all 28 ends with 1 false box in 139 frames without one. Real ends score 0.28–0.55, so 0.4 (the lights' bar) found only 13.
  - No model file, or it won't load: ends come from the sonar only (the lost search below), as before.
- **Lost** (missing **30 s**) (`lost_search_begin`, line 1809):
  1. Back over where it was last seen.
  2. Back up to 4 m off the bottom.
  3. A full 360° search: 15 stops, leaving out the part already followed.
  4. Found: steps 6–9 again, then on.
  - Up to `follow_searches` **2** searches at one spot. After that, it's the end.
- **`survey`, on the way to the buoy:** an end (seen, or lost within `BUOY_END_M` **3 m** of the buoy) is the buoy end. It turns round (step 12).
- **`follow` part, at an end:** back to where following began, turn 180°, back up 0.85 m, and follow the other part.
- **Ends when:**
  - at the end (the camera saw it, or nothing was found),
  - lost twice at one spot,
  - it has followed `follow_max_m` **40 m**,
  - or `follow_time_s` **1200 s** has passed.
- **Then:** POSHOLD (the pilot takes over), or for `full`, hold 5 s and surface.

## 12. Survey: the buoy end and the lights (`survey_turn` line 2718)
- **Does:**
  1. At the buoy end: turns 180°, backs up 0.6 m, and starts counting lights.
  2. Follows to the far end, logging each light the down camera sees, in order.
  3. The report goes to `sonar_data/survey_<date>_<time>.json`.
- **How lights are found, by `light_detector`** (`led_boxes.py`):
  - **`contrast` (the default):**
    - A round patch at least **50** brighter than the water around it, with a burnt-out white middle.
    - Its colour is the hue of its glow: red **0–25 / 165–180**, green **60–100**.
    - Green also has to add more green than red to the water around it.
    - It ignores the `DOWN` label and the magnet.
  - **`hsv`:** the old fixed colour test.
  - **`yolo`:** the trained model in `light_model`, with classes `red_light`, `green_light` and `pipe_end`. If the model won't load, it falls back to `contrast`.

## 13. RED light, `survey` part (`repair_begin`, line 2735)
- **Does:** goes back until the camera is over the RED light, and holds `repair_hold_s` **5 s**.
- **Next when:** after the hold, on along the pipe.

## 14. Repair centring, `repair` and `full` (`centre_step` line 2804, `centre_lower` line 2919)
- **Does:** lines up the light on the magnet's pixel, within `centre_tol_px` **20 px** for 1.5 s.
- **Each time it's lined up, it steps down:**
  - **0.4 m** while the magnet is more than 0.5 m above the light,
  - otherwise **0.1 m**,
  - never more than `centre_max_down_m` **0.65 m** below where centring began.
- **Next when:**
  - **`contrast`/`yolo`:** the biggest light reads GREEN in `C_GREEN_FRAMES` **3** pictures running (`light_fractions`, line 2871).
  - **`hsv`:** green fills at least 5% of the picture.
  - Then it rises 5 cm, logs the colours for 40 s, and goes on along the pipe.
- **Otherwise:** no GREEN in **300 s**: it goes on along the pipe.

## 15. Surfacing and the end (`mission_base.py:405`)
- **Does:** SURFACE goes up to the start depth where it is.
- **Next when:** within 0.10 m. Then DONE: disarm, MANUAL, exit.
