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
not a code change.

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
# span_deg and width_m are excluded on purpose: both collapse when you view a
# pipe end-on and open right up broadside, so scoring against them rejects the
# pipeline hardest exactly when it is most visible. They are still RECORDED,
# because they are what ORIENT reads to work out which way the pipe runs.
# range_m is excluded because it is a fact about the sub, not the object.
IDENTITY = ("height_m", "brightness", "thickness_m", "solidity")

DEFAULT_PATH = os.path.expanduser("~/sonar_data/objects.json")


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
        profile[name] = {"min": round(lo - pad, 4),
                         "ideal": round(mid, 4),
                         "max": round(hi + pad, 4)}
    return profile


def add_samples(library, name, samples, notes=""):
    """Append samples under `name` and rebuild that object's profile."""
    entry = library.setdefault(name, {"notes": notes, "samples": []})
    if notes:
        entry["notes"] = notes
    entry["samples"].extend(samples)
    entry["profile"] = build_profile(entry["samples"])
    return entry


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
