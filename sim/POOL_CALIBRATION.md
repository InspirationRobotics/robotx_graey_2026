# Calibrating the simulated sonar from the pool

The sim's echo strengths (`sim/sonar_physics.py`: how loud floor, walls, PVC and the
light box are, and how that changes with angle and distance) are placeholders. These
recordings set them.

Use the Map tab's **Record** button for each one: type what it is in the note box
first (e.g. "pipe side-on 2 m right"), press Record, Stop when done. Each becomes
`sonar_data/rec_<time>_<note>/` with every sweep, every VN-100/DVL reading, all
settings and the Ping360's own settings (`api/sonar/recorder.py`). Press **Mark**
(with a note) at moments worth finding later, e.g. "turned 90 deg now". The sim
can then replay the exact same path and the two maps can be compared ping by ping.

**Floor cut 0** for all of them (Map page, or `--floor-cut 0`): the test pipe may be
lying on the pool floor, and the cut would hide it.

Put in the note, with a tape measure, where the pipe is from the sub (distance, which
side, which way it points).

| # | What | Why |
|---|---|---|
| 0 | Copy the Oct 4 wall run off Graey (`sonar_data/map_20261004_*`) | The wall map you already know looks right |
| 1 | Pool size: length, width, depth; walls and floor material (concrete? liner?) | To build the pool in the sim |
| 2 | Hover still facing a wall at ~1, 2 and 3 m, 1 min each | Wall echo vs distance |
| 3 | Hover beside or above the pipe with the pipe pointing the **same way as the sub** (the slice cuts straight across it: side-on), ~2 m away, 2 min | Pipe echo, best case |
| 4 | Same spot, turned 90 deg so the pipe points **left-right across the sub** (the slice runs along it: end-on), 2 min | Does a real pipe go quiet end-on? (the sim says yes, Ruth doubts it) |
| 5 | One slow turn on the spot (~4 deg/s) next to the pipe | The far-scan pattern, real |
| 6 | Slow pass along the pipe, then across it | The follow pattern, real |
| 7 | If there's time: 3 and 4 again from twice as far | How the pipe's echo fades with distance |

Settings to keep the same in all runs: range, threshold, sector (bottom half), and
the Ping360's gain (whatever the tool uses now).
