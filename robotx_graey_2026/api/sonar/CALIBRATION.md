# Recalibrating the sonar for a new location

What has to be measured again when you move the sub to different water, what
carries over, and the traps that have already cost a day.

## What to redo, and when

| Setting | Redo when | Why |
|---|---|---|
| `DOWN_GRADIAN` | the sonar is unbolted or remounted | it describes how the sonar sits on the sub, nothing else |
| `SPEED_OF_SOUND` | the water changes type | fresh is about 1480 m/s, salt about 1520. Every range scales with it |
| `DETECT["threshold"]` | **every new location** | it is the local background level and travels worst of all |
| every object's `brightness` | **every new location** | same reason. Re-record the object with `tools/sonar_record.py`; the profile rebuilds itself from the new samples |
| `ALIGNED_SPAN_FRACTION` | rarely | span is geometry, so it barely cares about the water |

Ranked by how well they transfer: span ratio best, then speed of sound, then
brightness and threshold worst.

Marina Bay sits behind a barrage and is **fresh**, so 1480 should hold there.

## How to measure each one

### DOWN_GRADIAN

Stand a **vertical** pole a couple of metres off the sub's beam, sub upright. A
vertical pole cuts the scan plane at whatever depth the sonar sits at, so nothing
has to be measured or matched. Off starboard it must read bearing 0, off port
180.

    DOWN_GRADIAN = (reported bearing - expected) / 0.9,  mod 400

Do **both** sides and average. The pole spans the water column, so the detection's
centre lands wherever the pole returns strongest rather than exactly on the
horizontal. That bias tilts the answer one way to starboard and the other to
port, so one side alone is worth about ten degrees of error.

### SPEED_OF_SOUND

Tape-measure from the **sonar face**, not the hull, to a target. Compare against
the reported range and scale by `true / reported`. Measuring from the hull side
leaves the mounting offset in your answer, which looked like a 12% error once.

### Threshold and brightness

Do not try to measure a "noise floor" in a pool. A pool is not empty water; every
sweep is full of wall returns. Run a **paired before-and-after** instead:

1. Park the sub and do not move it.
2. Sweep with nothing added.
3. Put the target in and sweep again, same range, arc and threshold.
4. Anything in the second that is not in the first is the target.

The useful question is not "how noisy is the water" but "is the target brighter
than the clutter". If they overlap, no threshold separates them and the code has
to lean on height above the floor and span instead.

### Span ratio

Needs a **horizontal** pipe. A vertical pole is round about its own axis, so
turning it changes nothing. Record the pipe held one way, then rotated 90
degrees, at the same range and threshold. The ratio of narrow span to wide span
is what `ALIGNED_SPAN_FRACTION` is guessing at.

## Measured, 16 Sep 2026, pool

Graey upright, sonar about 0.3 m down, pool under 1 m deep, fresh water.

| Quantity | Value |
|---|---|
| `DOWN_GRADIAN` | 111, starboard side only, provisional to about 10 deg |
| PVC brightness | 132 to 146, so call it 140 |
| Pool background at threshold 120 | nothing detected at all |
| Range repeatability | plus or minus 1.5 cm at 1.32 m |
| Sweep time | roughly 120 ms per ping; the motor dominates, not the range |

`DETECT["threshold"]` still ships at 60, which was a guess and would be swamped
in this pool. 120 cleanly separated PVC from the clutter here.

Still outstanding: the port-side reading for `DOWN_GRADIAN`, the span ratio, and
anything at all involving height above the floor.

## Traps

**Do not calibrate off the floor.** The floor detector only finds a flat surface,
and in a pool a wall is equally flat. It once reported a confident floor 1.1 m
down that was a wall, and agreed with itself at the wrong rotation. Only a
reference you placed yourself is honest.

**A tilted sub tilts the horizon, and that is correct.** If the picture looks
rotated, check the sub's attitude before touching `DOWN_GRADIAN`. The sub sat 9
degrees off level once and the display faithfully showed it. Folding that into the
mounting constant would break the display whenever the sub is level. The real
fixes are trimming the sub, or rotating the scan angles by the IMU's roll before
the floor detector sees them.

**Nothing inside 0.75 m is ever detected.** `find_blobs` blanks the near field
before it looks, because that region is the sonar hearing its own transmit pulse.
The radar still draws it, so you can see something the detector will never report.
In water shallower than about a metre this means the floor and the surface are
both invisible, and only sideways directions are usable.

**Use `python3 -u`** when piping into `tee`, or Python buffers the output and the
terminal stays blank for dozens of sweeps.

**To sweep an arc across 0, count past 360.** `--start 352 --end 8` produces zero
pings. `--start 340 --end 380` gives the 40 degrees you wanted. A narrow arc is
worth having: 21 pings instead of 180 turns a 20 second sweep into about 3.

**Only one client at a time.** Ping Viewer and these tools both drive the same
motor through the proxy, and running both gives you garbage in both.
