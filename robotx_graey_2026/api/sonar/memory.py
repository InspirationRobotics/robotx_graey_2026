"""Piece 6: the only thing that remembers previous sweeps.

Everything else recomputes from scratch each time. State is where bugs hide, so
all of it lives here, and there is deliberately very little of it - if
something behaves strangely in the water, this short list is the entire set of
things that could be wrong.

Two questions a single sweep cannot answer, which is why this exists:

  Is the pipe drifting off to one side?  -> the pipeline is bending
  Which heading gave the widest return?  -> that heading is broadside to it

No ROS, no hardware.
"""
from . import settings as S

HISTORY = 5      # how many recent sweeps to keep


class Memory:
    def __init__(self, history=HISTORY):
        self.history = history
        self.offsets = []          # metres, positive to starboard
        self.heights = []          # metres above the floor
        self.spans = []            # degrees of angular extent
        self.headings = []         # compass heading at each detection
        self.misses = 0            # sweeps since the last detection
        self.scan = []             # (heading, span, score) gathered while yawing

    # ------------------------------------------------------------ recording
    def record(self, detection, heading_deg=None):
        """A sweep that found something."""
        self.offsets.append(detection.offset_m)
        self.heights.append(detection.height_m)
        self.spans.append(detection.span_deg)
        self.headings.append(heading_deg)
        for seq in (self.offsets, self.heights, self.spans, self.headings):
            del seq[:-self.history]
        self.misses = 0

    def record_miss(self):
        """A sweep that found nothing. Counted, not forgotten."""
        self.misses += 1

    def record_scan_point(self, heading_deg, detection):
        """One stop of a yaw scan, for finding which heading is broadside."""
        self.scan.append((heading_deg, detection))

    def clear_scan(self):
        self.scan = []

    def forget(self):
        self.__init__(self.history)

    # ------------------------------------------------------------- readings
    @property
    def offset(self):
        return self.offsets[-1] if self.offsets else None

    @property
    def height(self):
        return self.heights[-1] if self.heights else None

    @property
    def span(self):
        return self.spans[-1] if self.spans else None

    @property
    def stale(self):
        return self.misses >= S.STALE_SWEEPS

    @property
    def centred(self):
        return self.offset is not None and abs(self.offset) <= S.OFFSET_CENTRED_M

    # -------------------------------------------------------------- trends
    def offset_slope(self):
        """Metres of sideways drift per sweep, or None without enough history.

        This is the bend detector. Flying along a straight pipe, the offset
        jitters around a constant. Flying toward a bend, it marches steadily
        one way, and the sign says which way.
        """
        return _slope(self.offsets)

    def span_slope(self):
        """Degrees per sweep. A growing span corroborates a bend - the pipe is
        becoming less parallel to you."""
        return _slope(self.spans)

    def bend(self):
        """None, or "port" / "starboard" if the pipe is turning.

        Needs a full history so a single bad sweep cannot trigger it.
        """
        if len(self.offsets) < self.history:
            return None
        slope = self.offset_slope()
        if slope is None or abs(slope) < S.BEND_SLOPE_M_PER_SWEEP:
            return None
        return "starboard" if slope > 0 else "port"

    def best_scan_heading(self):
        """Of the headings tried while yawing, the one whose return was widest.

        The pipe is broadside at that heading, so it runs perpendicular to it -
        which means turning 90 degrees from there points you along the pipe.
        Returns (heading, detection) or None - the whole detection, because
        whoever acts on this needs to know which side the pipe was on, not just
        how wide it looked.
        """
        seen = [(h, d) for h, d in self.scan if h is not None and d is not None]
        if not seen:
            return None
        return max(seen, key=lambda hd: hd[1].span_deg)

    def __repr__(self):
        o = "-" if self.offset is None else f"{self.offset:+.2f}m"
        s = "-" if self.span is None else f"{self.span:.0f}deg"
        return f"Memory(offset {o}, span {s}, misses {self.misses})"


def _slope(values):
    """Least-squares slope per step. Steadier than differencing the ends,
    which lets one noisy sweep dominate."""
    n = len(values)
    if n < 3:
        return None
    xs = list(range(n))
    mx = sum(xs) / n
    my = sum(values) / n
    denom = sum((x - mx) ** 2 for x in xs)
    if denom == 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, values)) / denom
