"""Measured disk growth helpers copied from Crusader main abf7d821."""
import os
import threading
import time

RATE_WINDOW_S = 20.0


def hours_left(free_bytes, rate_bytes_s):
    """How long the disk lasts at the measured rate, or None.

    None while the rate is not measurable: a blank, not a reassuring large number.
    """
    if not rate_bytes_s or rate_bytes_s <= 0 or free_bytes is None:
        return None
    return free_bytes / rate_bytes_s / 3600.0


class GrowthMeter:
    """Bytes-per-second of a directory, MEASURED, with its own lock.

    Shared by every child process that writes into a session, so the answer cannot
    depend on which one is asking.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._samples = []           # [(monotonic, bytes)] over RATE_WINDOW_S
        self._size = 0

    def reset(self):
        with self._lock:
            self._samples = []
            self._size = 0

    def sample(self, path):
        """One reading of `path`. Called from the node's existing timer."""
        now = time.monotonic()
        size = dir_size_bytes(path)
        with self._lock:
            self._size = size
            self._samples.append((now, size))
            cutoff = now - RATE_WINDOW_S
            # Keep one sample older than the window so a rate is available from
            # the first tick after it, rather than only once the window fills.
            while len(self._samples) > 2 and self._samples[1][0] < cutoff:
                self._samples.pop(0)

    @property
    def size_bytes(self):
        with self._lock:
            return self._size

    def rate_bytes_s(self):
        """Measured growth, or None until two samples exist far enough apart."""
        with self._lock:
            if len(self._samples) < 2:
                return None
            (t0, s0), (t1, s1) = self._samples[0], self._samples[-1]
        if t1 - t0 < 1.0:
            return None
        return max(0.0, (s1 - s0) / (t1 - t0))

    def readout(self, free_bytes=None):
        """{size_mb, rate_mb_s, hours_left} — blanks, never zeroes, when
        nothing has been measured yet."""
        rate = self.rate_bytes_s()
        left = None if rate is None else hours_left(free_bytes, rate)
        return {
            "size_mb": round(self.size_bytes / 1048576.0, 1),
            "rate_mb_s": None if rate is None else round(rate / 1048576.0, 2),
            "hours_left": None if left is None else round(left, 1),
        }


def frame_rates(requested, keys, default, lo=None, hi=None):
    """Per-viewer capture rates as {key: hz}, from whatever the operator sent.

    Args:
      requested: None, a single number meaning "the same rate everywhere"
        (which is all the parameter alone could ever mean), or a {key: hz}
        mapping naming only the viewers it wants to change.
      keys: the viewers this session will actually pull from.
      default: the parameter's value, used for every key `requested` omits.
      lo, hi: the parameter's own range, so a rate the page accepted is never
        one this then quietly rewrites.

    Returns:
      (rates, notes). A bad value costs you the OVERRIDE, not the recording, but
      is never silent: whatever was ignored or clamped comes back in `notes` and
      is shown beside the "recording" message.
    """
    notes = []
    if requested is None:
        requested = {}
    elif isinstance(requested, bool) or not isinstance(requested,
                                                       (dict, int, float)):
        notes.append("frame_hz must be a number or a per-viewer object, not "
                     f"{type(requested).__name__} - using {default:g} fps")
        requested = {}
    elif not isinstance(requested, dict):
        requested = {k: requested for k in keys}

    rates = {}
    for key in keys:
        v = requested.get(key, default)
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            notes.append(f"{key} fps {v!r} is not a number - using {default:g}")
            v = default
        elif lo is not None and v < lo:
            notes.append(f"{key} fps {v:g} is below {lo:g} - using {lo:g}")
            v = lo
        elif hi is not None and v > hi:
            notes.append(f"{key} fps {v:g} is above {hi:g} - using {hi:g}")
            v = hi
        rates[key] = float(v)
    return rates, notes


def free_gb(path):
    try:
        st = os.statvfs(path)
    except (OSError, AttributeError):
        return None
    return st.f_bavail * st.f_frsize / 1073741824.0


def dir_size_bytes(path):
    """Total size of everything under `path`, skipping what it cannot stat.

    A file being written mid-walk is the normal case here, and a size readout that
    throws would take the whole snapshot down with it.
    """
    total = 0
    for base, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(base, f))
            except OSError:
                pass
    return total


def dir_size_mb(path):
    return round(dir_size_bytes(path) / 1048576.0, 1)


