"""A library of measured objects, and matching detections against it.

One JSON file holds every object you have ever measured, each under its own
name:

    {
      "pvc_pipe": {
        "notes": "1.3 m off starboard, pool, 16 Sep",
        "samples": [{"brightness": 146, "span_deg": 16, ...}, ...],
        "profile": {"brightness": {"min": ..., "ideal": ..., "max": ...}, ...}
      },
      "pool_wall": { ... }
    }

samples are raw measurements, one per sweep. profile is derived from them and
is what the scorer actually reads. Keeping both means a profile can always be
rebuilt with different rules, and you can see what it was built from.

Why a file rather than numbers in settings.py: a profile is a description of a
THING, and which thing you are hunting is a mission decision. settings.py is for
the sonar and the water. It also means measuring a new object is a data task,
not a code change. settings.py used to carry a PIPELINE_PROFILE, which quietly
made "a pipeline" the one thing this package could look for; it is gone.

resolve() IS THE DOOR. Every entry point - perceive(), Driver(), the viewer,
every tool - takes a target argument and passes it through resolve(). So the
same thing can always be written any of these ways:

    "pvc_pipe"                      a name you measured and stored
    "height_m=1.5,brightness=140"   ideals typed straight in
    None                            no target: measure everything, judge
                                    nothing, draw no rings

and nothing downstream has to care which you used.

No ROS, no hardware, no OpenCV.
"""
import json
import os

# Features worth recording. Not all belong in a profile - see build_profile.
RECORDED = ("height_m", "brightness", "span_deg", "solidity", "width_m",
            "thickness_m", "range_m")

# Features that describe what a thing IS, rather than where you happen to be
# standing. Only these go into a profile by default.
#
# The rule behind the list, which is worth more than the list: A FEATURE THAT
# CHANGES WITH POSE IS AN OUTPUT, NOT AN IDENTITY CONSTRAINT. Only put things in
# a profile that describe the object regardless of how you are looking at it.
#
# span_deg and width_m are excluded by that rule. Both collapse when you view a
# pipe end-on and open right up broadside - a couple of degrees one way, over a
# hundred the other - so scoring against them rejects the pipeline hardest
# exactly when it is most visible. They are still RECORDED, because they are
# what ORIENT reads to work out which way the pipe runs.
#
# solidity is a borderline case and is kept, but know why it wobbles: it is high
# for a compact end-on blob and low broadside, because a pipe seen side-on
# traces an arc and an arc's convex hull is mostly empty space. Drop it from a
# profile if it turns out to be costing you detections.
#
# range_m is excluded because it is a fact about the sub, not the object.
IDENTITY = ("height_m", "brightness", "thickness_m", "solidity")

DEFAULT_PATH = os.path.expanduser("~/sonar_data/objects.json")

# What each feature can physically be. profile_from_ideals clamps to these, so a
# window either side of an ideal cannot run off the end of the scale and promise
# a brightness of 300 or a solidity of 1.4.
LIMITS = {
    "brightness":  (0.0, 255.0),
    "solidity":    (0.0, 1.0),
    "span_deg":    (0.0, 360.0),
    "height_m":    (0.0, None),
    "thickness_m": (0.0, None),
    "width_m":     (0.0, None),
    "range_m":     (0.0, None),
}

# Default half-width of an ideal-only profile, as a fraction of the ideal.
TOLERANCE = 0.5


def load(path=DEFAULT_PATH):
    """Whole library, or an empty one if the file does not exist yet."""
    if not os.path.exists(path):
        return {}
    with open(path) as fh:
        return json.load(fh)


def save(library, path=DEFAULT_PATH):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as fh:
        json.dump(library, fh, indent=2, sort_keys=True)


def sample_from(detection):
    """A Detection -> the plain dict that gets stored."""
    out = {}
    for name in RECORDED:
        value = getattr(detection, name, None)
        if value is not None:
            out[name] = round(float(value), 4)
    return out


def build_profile(samples, features=IDENTITY, margin=0.25):
    """Samples -> a profile the scorer can use.

    ideal is the MEDIAN, not the mean, so one bad sweep cannot drag it. The
    edges are the observed range widened by `margin` of that spread, because a
    handful of sweeps never sees the full spread of a thing and a profile whose
    edges sit exactly on your samples will reject the next honest measurement.

    A feature every sample agrees on gets a floor of 10% of its value as the
    half-width, so a constant does not produce a zero-width profile that only
    ever scores 0 or 1.
    """
    profile = {}
    for name in features:
        values = sorted(s[name] for s in samples if name in s)
        if not values:
            continue
        mid = values[len(values) // 2]
        lo, hi = values[0], values[-1]
        pad = max((hi - lo) * margin, abs(mid) * 0.1, 1e-6)
        lo, hi = _clamp(name, lo - pad, hi + pad)
        profile[name] = {"min": round(lo, 4),
                         "ideal": round(mid, 4),
                         "max": round(hi, 4)}
    return profile


def _clamp(name, lo, hi):
    """Hold an edge inside what the feature can physically be.

    Every profile goes through this, however it was built. Padding a measured
    spread can otherwise produce a minimum thickness of -0.02 m, which is not a
    tolerant profile, it is a meaningless one - and a bound that can never be
    crossed silently stops being a constraint at all.
    """
    low_limit, high_limit = LIMITS.get(name, (None, None))
    if low_limit is not None:
        lo = max(lo, low_limit)
    if high_limit is not None:
        hi = min(hi, high_limit)
    return lo, hi


def profile_from_ideals(ideals, tolerance=TOLERANCE):
    """{"height_m": 1.5, "brightness": 140} -> a full min/ideal/max profile.

    You type the number you expect to see and nothing else; the edges go a
    fraction either side, clamped to what the feature can physically be.

    THE FRACTION IS A KNOB, NOT A MEASUREMENT. It sets how fussy the score is
    and says nothing about the object. Widen it and everything scores higher,
    including the things you do not want. The honest way to place edges is to
    measure the object with tools/sonar_record.py and let the spread of real
    samples do it. This exists for trying a number quickly, and for the case
    where you know roughly what a thing should look like but have not met it
    yet.

    An ideal of zero has nothing to scale, so it falls back to `tolerance` in
    the feature's own units - which is a guess about units and probably wrong.
    Give min/max yourself if a feature's ideal is zero.
    """
    profile = {}
    for name, ideal in ideals.items():
        ideal = float(ideal)
        pad = abs(ideal) * tolerance or float(tolerance)
        lo, hi = _clamp(name, ideal - pad, ideal + pad)
        profile[name] = {"min": round(lo, 4), "ideal": round(ideal, 4),
                         "max": round(hi, 4)}
    return profile


def parse_ideals(text):
    """"height_m=1.5,brightness=140" -> {"height_m": 1.5, "brightness": 140.0}

    For typing a target straight into a command line. Unknown feature names are
    rejected rather than ignored, because a silently dropped constraint is a
    profile that quietly scores everything.
    """
    ideals = {}
    for piece in text.replace(";", ",").split(","):
        piece = piece.strip()
        if not piece:
            continue
        if "=" not in piece:
            raise ValueError(f"expected feature=value, got {piece!r}")
        key, value = piece.split("=", 1)
        key = key.strip()
        if key not in RECORDED:
            raise ValueError(f"no feature called {key!r}. Have: "
                             f"{', '.join(RECORDED)}")
        try:
            ideals[key] = float(value)
        except ValueError:
            raise ValueError(f"{key}={value.strip()!r} is not a number") from None
    if not ideals:
        raise ValueError("no features given")
    return ideals


def resolve(target, path=DEFAULT_PATH, tolerance=TOLERANCE):
    """Whatever a caller was handed -> a profile, or None for "judge nothing".

    This is the ONE place the meaning of a target argument is decided, so the
    viewer, the state machine and the tools cannot disagree about it. Five
    things are accepted:

        None, or ""                     no profile. Nothing is scored, and the
                                        viewer draws no rings - it just shows
                                        what is out there.
        "pvc_pipe"                      a name in the object library, as
                                        recorded by tools/sonar_record.py.
        "height_m=1.5,brightness=140"   ideal values typed straight in.
        {"height_m": {"min": ...}}      a profile, used as it stands.
        {"height_m": 1.5}               ideals as a dict.

    A bare name is the one worth reaching for. It is the only form where the
    numbers came from the real object in real water, and changing what you hunt
    is then a matter of typing a different name rather than editing code.
    """
    if target is None or target == "":
        return None
    if isinstance(target, dict):
        if not target:
            return None
        shaped = [isinstance(v, dict) for v in target.values()]
        if all(shaped):
            return target
        if not any(shaped):
            return profile_from_ideals(target, tolerance)
        raise ValueError("a target dict must be all ideals or all min/ideal/max "
                         "specs, not a mixture")
    if "=" in target:
        return profile_from_ideals(parse_ideals(target), tolerance)
    return profile_for(load(path), target)


def add_samples(library, name, samples, notes=""):
    """Append samples under `name` and rebuild that object's profile."""
    entry = library.setdefault(name, {"notes": notes, "samples": []})
    if notes:
        entry["notes"] = notes
    entry["samples"].extend(samples)
    entry["profile"] = build_profile(entry["samples"])
    return entry


def clear(library, name):
    """Forget everything measured under `name`. Returns how many samples went.

    add_samples only ever appends, and a profile is rebuilt from EVERY sample
    an object has ever had. So one run that measured the wrong blob poisons the
    profile for good unless there is a way to start again - and the poisoning is
    quiet, because the numbers still look like numbers.
    """
    entry = library.pop(name, None)
    return len(entry.get("samples", [])) if entry else 0


def disagreement(entry, samples, features=IDENTITY):
    """Which features of `samples` fall outside what `entry` has seen before.

    Appending to an object is right when you are measuring the same thing again
    and wrong when you have drifted onto something else, and the two look
    identical from the command line. If the new median sits outside the whole
    range of the old samples, that is not more evidence about one object - it is
    evidence about two.
    """
    out = []
    for name in features:
        old = sorted(s[name] for s in entry.get("samples", []) if name in s)
        new = sorted(s[name] for s in samples if name in s)
        if not old or not new:
            continue
        mid = new[len(new) // 2]
        if mid < old[0] or mid > old[-1]:
            out.append((name, mid, old[0], old[-1]))
    return out


def profile_for(library, name):
    """The profile dict for one object, ready to hand to detect.score()."""
    entry = library.get(name)
    if entry is None:
        raise KeyError(f"no object called {name!r}. Have: "
                       f"{', '.join(sorted(library)) or 'nothing'}")
    return entry["profile"]


def identify(detection, library, min_score=0.0):
    """Which object in the library does this detection look most like?

    Returns (name, score, per_feature) for the best fit, or (None, 0.0, {}) if
    nothing clears min_score. Scoring is imported here rather than at module
    level so this file stays importable without OpenCV.

    Read the runner-up as well as the winner. Two objects scoring 0.6 and 0.58
    means the library cannot really tell them apart, which is a fact about your
    features, not about this detection.
    """
    from .detect import score

    best = (None, 0.0, {})
    for name, entry in library.items():
        if not entry.get("profile"):
            continue
        per, total = score(detection, entry["profile"])
        if total > best[1]:
            best = (name, total, per)
    return best if best[1] >= min_score else (None, 0.0, {})


def rank(detection, library):
    """Every object scored, worst to best last. For seeing near-misses."""
    from .detect import score

    out = []
    for name, entry in library.items():
        if entry.get("profile"):
            out.append((name, score(detection, entry["profile"])[1]))
    return sorted(out, key=lambda pair: pair[1])
