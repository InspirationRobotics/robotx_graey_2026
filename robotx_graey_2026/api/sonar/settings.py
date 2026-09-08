"""Every number you might want to change, in one place.

Two kinds of setting live here and they are kept apart deliberately:

  SONAR AND WATER  - properties of the hardware and the environment. Retune
                     these when the water changes. They will need retuning
                     between a pool and Marina Bay.

  TARGET           - what the thing you are hunting looks like. This is NOT
                     here. It is passed in by the mission, because which
                     object you want is a mission decision, not a sonar one.
                     See PIPELINE_PROFILE at the bottom for the shape it takes.

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
# Calibrate by aiming at the floor and seeing which gradian lights up, or by
# putting a target directly below. Every angle this package reports is
# measured from here, so if the whole picture looks rotated, change this.
DOWN_GRADIAN = 0

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
MIN_SCORE = 0.25                # below this, treat a candidate as not found
STALE_SWEEPS = 4                # no detection for this many sweeps means lost

# ------------------------------------------------- example target profile
# NOT settings - this is what the mission passes in, and it belongs in mission
# code. Reproduced here only as a worked example of the shape.
#
# Every number is a guess until you measure the real structure with the debug
# viewer. Do not trust these.
# Only two features, and the omissions are the point.
#
# span_deg and solidity are both measured, both shown in the viewer, and both
# deliberately absent from the scoring. Each varies with the pipe's ORIENTATION
# rather than with what the pipe IS:
#
#   span      a couple of degrees end-on, over a hundred broadside
#   solidity  high for a compact end-on blob, low broadside, because a pipe
#             seen side-on traces an arc and an arc's convex hull is mostly
#             empty space
#
# Scoring against either one rejected the pipeline hardest exactly when it was
# most visible. The rule this leaves behind is worth keeping: a feature that
# changes with pose is an OUTPUT, not an identity constraint. Only put things
# in a profile that describe what the object is regardless of how you are
# looking at it.
#
# That leaves height above the floor, which is the strongest discriminator you
# have - it is what separates a suspended pipeline from the bottom - and
# brightness, which is a material property.
PIPELINE_PROFILE = {
    "height_m":     {"min": 0.4, "ideal": 1.5, "max": 2.8},   # above the floor
    "brightness":   {"min": 70,  "ideal": 170, "max": 255},
}
