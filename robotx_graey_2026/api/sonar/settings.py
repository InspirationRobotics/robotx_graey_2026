"""Every number you might want to change, in one place.

Two kinds of setting live here and they are kept apart deliberately:

  SONAR AND WATER  - properties of the hardware and the environment. Retune
                     these when the water changes. They will need retuning
                     between a pool and Marina Bay.

  TARGET           - what the thing you are hunting looks like. This is NOT
                     here, and must not end up here. It is passed in by the
                     mission, because which object you want is a mission
                     decision, not a sonar one. See sonar/library.py, and the
                     note at the bottom of this file.

Angle convention throughout: degrees in the vertical scan plane.
    0 = right, 90 = up, 180 = left, 270 = down.
Direction vector is (cos a, sin a) in (right, up).
"""

# ---------------------------------------------------------------- physics
SPEED_OF_SOUND = 1480.0     # m/s. FRESH water - Marina Bay sits behind the
                            # barrage and is a reservoir, not seawater. Salt
                            # would be ~1520. Getting this wrong scales every
                            # range you measure.

TICK = 25e-9                # sonar's sample-period clock tick, seconds
MAX_SAMPLES = 1000          # hardware cap on range bins per ping
MIN_SAMPLE_PERIOD = 400     # hardware floor on sampling rate
GRADIANS = 400              # a full turn, in the sonar's own units
DEG_PER_GRADIAN = 360.0 / GRADIANS

# Beam shape, fixed by the transducer. With the unit mounted so the scan plane
# is vertical, the narrow 2 degrees is the vertical resolution and the wide
# 25 degrees is how thick the "curtain" is fore and aft.
BEAM_IN_PLANE_DEG = 2.0
BEAM_FORE_AFT_DEG = 25.0

MIN_RANGE_M = 0.75          # near-field blanking. Inside this the sonar is
                            # still hearing its own transmit pulse.

# THE calibration constant. Which hardware gradian points straight down.
# Every angle this package reports is measured from here, so if the whole
# picture looks rotated, this is why.
#
# HOW TO MEASURE IT. Stand a vertical pole in the water a couple of metres off
# the sub's beam, with the sub upright. A vertical pole cuts the scan plane
# whatever depth the sonar sits at, so nothing has to be measured or matched.
# Off the starboard side it must read bearing 0, off port 180. Then
#
#     DOWN_GRADIAN = (reported bearing - expected) / 0.9,  mod 400
#
# Do BOTH sides and average. The pole spans the water column, so the detection's
# centre sits wherever the pole returns strongest rather than exactly on the
# horizontal, and that bias tilts the answer one way to starboard and the other
# way to port. One side alone is worth about ten degrees of error.
#
# DO NOT calibrate off the floor, which is what this comment used to advise. The
# floor detector only finds a flat surface, and in a pool a wall is just as flat.
# It reported a confident "floor" that was a wall, and agreed with itself at the
# wrong rotation. A reference you placed yourself is the only honest one.
#
# 111 measured 16 Sep 2026 in the pool, starboard side only, so treat it as
# provisional to about ten degrees until the port-side reading is averaged in.
DOWN_GRADIAN = 111

# --------------------------------------------------------- sweep per state
# Each state gets its own arc, step and range, because they want different
# things. SEARCH is thorough and slow; FOLLOW is narrow and fast.
#
# Time per ping is roughly (2 * range / 1480) + 9 ms of motor and serial
# overhead. The overhead does not shrink with range, so short-range sweeps are
# faster but not proportionally so - the big lever is arc width, not range.
SWEEP = {
    # downward half of the plane: left, through down, to right
    "SEARCH": dict(start_deg=180, end_deg=360, step_deg=2, max_range_m=8.0),
    # APPROACH must reach at least as far as SEARCH. Shortening it here means
    # a target found near the search limit disappears the moment the state
    # changes, which looks exactly like a dropout and is not one.
    "APPROACH": dict(start_deg=180, end_deg=360, step_deg=2, max_range_m=8.0),
    "ORIENT": dict(start_deg=180, end_deg=360, step_deg=2, max_range_m=6.0),
    # narrow arc either side of straight down - this is the one that runs often
    "FOLLOW": dict(start_deg=240, end_deg=300, step_deg=2, max_range_m=4.0),
    # widen back out gradually rather than jumping to a full search
    "LOST":   dict(start_deg=200, end_deg=340, step_deg=2, max_range_m=5.0),
}

# ------------------------------------------------------- detection tuning
DETECT = dict(
    threshold=60,           # 0-255. Echo strength below this is not a blob.
    blur=5,                 # smoothing; kills speckle, keeps solid patches
    close=11,               # gap filling; merges a patchy object into one blob
    min_blob_px=12,         # smaller contours are not worth measuring
    floor_margin_m=0.30,    # ignore anything within this of the detected floor
    height_band_m=(0.4, 2.8),   # how far above the floor to look. The pipeline
                                # is specified at 1-2 m; the band is wider
                                # because the build guide says segments are not
                                # all in the same plane.
)

# --------------------------------------------------------- search pattern
# The scan plane reaches out both sides of the sub, so yawing 180 degrees
# covers every horizontal bearing - not 360. Steps must be no wider than the
# beam's fore-aft thickness or you leave unswept wedges between headings.
SEARCH_YAW_SPAN_DEG = 180.0
SEARCH_YAW_STEP_DEG = 22.5      # 8 headings, slight overlap on 25 deg beam

# ------------------------------------------------------------- behaviour
FOLLOW_STEP_M = 0.5             # how far to move between FOLLOW sweeps
APPROACH_STEP_M = 0.4           # how far to strafe per APPROACH sweep
APPROACH_CENTRED_M = 0.4        # close enough overhead to stop strafing
OFFSET_CENTRED_M = 0.25         # offset smaller than this counts as centred
BEND_SLOPE_M_PER_SWEEP = 0.12   # offset drifting faster than this means a bend
BEND_TURN_DEG = 45.0            # elbows are 45 degrees; after a bend the new
                                # heading is one of exactly two options
# Measured spans come out wider than the raw beam: painting, blur and the
# closing kernel all add a few degrees. An aligned pipe reads around 10, not 2,
# so this sits above that. Confirm against real returns before trusting it.
SPAN_ALONG_DEG = 14.0           # span below this: pipe runs fore-aft (aligned)
ALIGNED_SPAN_FRACTION = 0.6     # or: span has fallen to this share of its
                                # broadside value. Relative, because span has no
                                # absolute meaning and a zigzag has no single
                                # direction to align to.
SPAN_ACROSS_DEG = 30.0          # span above this: pipe lies across the view
# Below this, a candidate is not the thing you are looking for. Scores are the
# geometric mean of the per-feature scores, so this reads as "every feature has
# to be at least about half right, on average". 0.50 replaces an old 0.25 that
# was against a plain PRODUCT of the feature scores: for the two-feature
# profiles used so far, a product of 0.25 IS a geometric mean of 0.50, so this
# keeps the bar where it was rather than moving it quietly.
#
# It is a knob, not a measurement. Watch what real objects and real clutter
# score in the viewer and move it to sit between them.
MIN_SCORE = 0.50
STALE_SWEEPS = 4                # no detection for this many sweeps means lost

# --------------------------------------------------- what NOT to put here
# There used to be a PIPELINE_PROFILE at the bottom of this file: the numbers
# describing the thing to hunt for. It is gone, and deliberately.
#
# A profile describes an OBJECT. This file describes the SONAR AND THE WATER.
# Keeping the object here meant every tool imported "a pipeline" whether it
# wanted one or not, the debug viewer drew rings for a target nobody had asked
# for, and measuring a new object was a code edit instead of a data entry.
#
# Objects now live in sonar/library.py, in a JSON file you fill by measuring
# real things with tools/sonar_record.py. Every entry point takes a target
# argument and passes it to library.resolve(), which accepts a name, a
# "height_m=1.5,brightness=140" string, a profile dict, or None for "judge
# nothing". The reasoning about WHICH features belong in a profile - the
# pose-invariance rule that keeps span and width out of one - moved to
# library.IDENTITY, where it applies to every object rather than to one.
