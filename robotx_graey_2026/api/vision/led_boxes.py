"""Light boxes in the bottom camera, by the colour of their LEDs (Ruth, Oct 16).

The boxes light themselves, so the test is plain colour, not a trained model: a
pixel counts when it is bright AND strongly coloured, and its colour is the
channel that clearly wins. On the Oct 15 pool footage (dark, pool lights off)
the LED boxes filled up to 36% (red) and 24% (green) of the down camera's
picture; with no box in view, 0-0.6%. A trained model (YOLO) would be the next
step if daylight washes the colours out, or to see a box with its LED off.

A SIGHTING starts when one colour covers min_frac of the picture and ends after
gap_s with none. Each sighting is placed where the camera was when its colour
peaked; sightings within same_m of an earlier box are the same box (the sub
hovers and comes back), so BoxTracker.boxes counts boxes, not sightings.
"""
import math
import threading
import time
import urllib.request

import numpy as np

SMALL = (160, 90)              # frames are judged at this size: plenty for a lit LED box


def colour_fractions(rgb):
    """Share of the picture (0-1) that is bright and red, green, blue or yellow.
    rgb: HxWx3 uint8."""
    d = rgb.astype(np.int16)
    r, g, b = d[..., 0], d[..., 1], d[..., 2]
    mx, mn = d.max(-1), d.min(-1)
    bright = (mx > 140) & (mx - mn > 70)
    n = float(r.size)
    return {
        'red': float((bright & (r >= g + 40) & (r >= b + 40)).sum()) / n,
        'green': float((bright & (g >= r + 30) & (g >= b + 10)).sum()) / n,
        'blue': float((bright & (b >= r + 30) & (b >= g + 10)).sum()) / n,
        'yellow': float((bright & (r >= b + 60) & (g >= b + 60) & (abs(r - g) < 40)).sum()) / n,
    }


class BoxTracker:
    def __init__(self, min_frac=0.02, gap_s=1.0, same_m=0.5):
        self.min_frac, self.gap_s, self.same_m = min_frac, gap_s, same_m
        self.boxes = []            # dicts: n, e, colours {colour: peak share}, sightings, first_t, last_t
        self._cur = None           # the sighting in progress

    def update(self, t, rgb, pos):
        """One frame at time t (s); pos = (north, east) of the camera, or None.
        Returns the box a sighting just ended on (new or seen again), else None."""
        f = colour_fractions(rgb)
        colour = max(f, key=f.get)
        if f[colour] >= self.min_frac:
            if self._cur is None:
                self._cur = {'t0': t, 'peak': {}, 'pos': pos, 'best': 0.0}
            c = self._cur
            c['last'] = t
            c['peak'][colour] = max(c['peak'].get(colour, 0.0), f[colour])
            if f[colour] > c['best'] and pos is not None:
                c['best'], c['pos'] = f[colour], pos
            return None
        if self._cur is not None and t - self._cur['last'] > self.gap_s:
            c, self._cur = self._cur, None
            return self._finish(c)
        return None

    def flush(self):
        """End a sighting still in view (the mission is stopping)."""
        if self._cur is None:
            return None
        c, self._cur = self._cur, None
        return self._finish(c)

    def _finish(self, c):
        pos = c['pos']
        for b in self.boxes:
            if pos is not None and b['n'] is not None and math.hypot(pos[0] - b['n'], pos[1] - b['e']) <= self.same_m:
                b['sightings'] += 1
                b['last_t'] = c['last']
                for k, v in c['peak'].items():
                    b['colours'][k] = max(b['colours'].get(k, 0.0), v)
                b['new'] = False
                return b
        b = {'n': pos[0] if pos else None, 'e': pos[1] if pos else None, 'colours': dict(c['peak']),
             'sightings': 1, 'first_t': c['t0'], 'last_t': c['last'], 'new': True, 'number': len(self.boxes) + 1}
        self.boxes.append(b)
        return b


def describe(b):
    cols = ', '.join(f'{k} {v:.0%}' for k, v in sorted(b['colours'].items(), key=lambda kv: -kv[1]))
    where = f"at NED ({b['n']:.2f}, {b['e']:.2f})" if b['n'] is not None else 'position unknown'
    return f"box {b['number']} ({cols}) {where}, {b['sightings']} sighting(s)"


def frames_from_mjpeg(url, stop, every=2, timeout=5.0):
    """Yield (time, JPEG bytes) from an MJPEG stream (tools/cam_pair.py serves one at
    :8080/stream), every `every`-th frame, until stop is set or the stream ends."""
    with urllib.request.urlopen(url, timeout=timeout) as s:
        buf, k = b'', 0
        while not stop.is_set():
            chunk = s.read(32768)
            if not chunk:
                return
            buf += chunk
            while True:
                a = buf.find(b'\xff\xd8')
                if a < 0:
                    buf = buf[-1:]
                    break
                z = buf.find(b'\xff\xd9', a + 2)
                if z < 0:
                    buf = buf[a:]
                    break
                jpg, buf = buf[a:z + 2], buf[z + 2:]
                k += 1
                if k % every == 0:
                    yield time.monotonic(), jpg


def down_half_rgb(jpg):
    """cam_pair puts DOWN on the left half, FRONT on the right: the left half, small, RGB."""
    import cv2
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    h, w = img.shape[:2]
    half = img[:, : w // 2] if w > 1.5 * h * 16 / 9 else img      # one camera only: the whole frame
    small = cv2.resize(half, SMALL, interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(small, cv2.COLOR_BGR2RGB)


class CameraWatcher:
    """Runs the tracker on the camera stream in a thread. where() gives the camera's
    (north, east) now; report(text) logs; on_box(box) is called as sightings end."""

    def __init__(self, url, where, report, on_box=None, **tracker_args):
        self.url, self.where, self.report, self.on_box = url, where, report, on_box
        self.tracker = BoxTracker(**tracker_args)
        self.stop = threading.Event()
        self.frames = 0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _run(self):
        warned = False
        while not self.stop.is_set():
            try:
                for t, jpg in frames_from_mjpeg(self.url, self.stop):
                    rgb = down_half_rgb(jpg)
                    if rgb is None:
                        continue
                    self.frames += 1
                    if self.frames == 1:
                        self.report(f'camera: watching for light boxes on {self.url}')
                    b = self.tracker.update(t, rgb, self.where())
                    if b is not None and self.on_box:
                        self.on_box(b)
            except Exception as ex:                  # no stream yet, cv2 missing, stream dropped
                if not warned:
                    self.report(f'camera: no picture from {self.url} ({ex}) - is tools/cam_pair.py running? '
                                f'Trying again every 5 s')
                    warned = True
            self.stop.wait(5.0)

    def close(self):
        self.stop.set()
        b = self.tracker.flush()
        if b is not None and self.on_box:
            self.on_box(b)
        return self.tracker.boxes
