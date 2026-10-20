"""Light boxes in the bottom camera, by the colour of their LEDs (Ruth, Oct 16).

The boxes light themselves, so the test is plain colour, not a trained model: HSV
ranges (Ruth, Oct 18) - a pixel counts when it is bright (V) AND strongly coloured
(S), and its colour is the hue range (H) it falls in. On the Oct 15 pool footage (dark, pool lights off)
the LED boxes filled up to 36% (red) and 24% (green) of the down camera's
picture; with no box in view, 0-0.6%. A trained model (YOLO) would be the next
step if daylight washes the colours out, or to see a box with its LED off.

Each COLOUR is tracked on its own: a sighting of a colour starts when it covers
min_frac of the picture and ends after gap_s without it, so two boxes of
different colours in view one after the other stay two boxes (sim, Oct 16: boxes
0.5-0.75 m apart merged when one sighting took any colour). A sighting is placed
where the camera was when that colour peaked; a sighting of the same colour
within same_m of an earlier box is that box again (the sub hovers and comes
back), so BoxTracker.boxes counts boxes, not sightings. Two filters from the
Oct 15 footage: a colour only counts in a frame where it is at least
minor_share of the strongest colour (a red LED's yellow fringe is not a yellow
box), and a NEW box needs a peak of at least new_frac (glimpses at the picture's
edge peaked at 2%; real boxes at 5-38%).
"""
import math
import threading
import time
import urllib.request

import numpy as np

SMALL = (160, 90)              # frames are judged at this size: plenty for a lit LED box

# HSV ranges, on OpenCV's scale: H 0-179 (half degrees: 0 red, 30 yellow, 60 green,
# 120 blue), S and V 0-255. Measured on the pool footage (down camera, Oct 15-17): the
# LEDs' hue is red 175-179 and 0-4, green 70-89 (a blue-ish green on this camera), blue
# 105-109; between green and blue (90-104) is the glow at a light's edge. LED pixels
# have S 130-255; the washed-out white middle and pale halo S < 65. Red sits at both
# ends of the hue circle, so it needs two ranges; orange (to 20) counts as red.
V_MIN = 140                    # bright: the LED, not the water. 140 found exactly the lights the old RGB
                               # rule did on all the pool footage; 100 also caught a dim red flash the old
                               # rule missed and lights at the picture's edge, and makes every light bigger
                               # (shares x1.4, so pair it with light_min_share ~0.14) (Oct 18)
S_MIN = 100                    # strongly coloured: not the white-hot middle or grey water
HUES = {
    'red': ((0, 20), (160, 180)),
    'yellow': ((20, 35),),
    'green': ((35, 97),),
    'blue': ((97, 140),),
}
# The survey's colours (Ruth, Oct 17): while the pipeline is surveyed its lights are only
# GREEN or RED, so every coloured pixel counts as whichever of the two its hue is nearer:
# the splits are halfway between the LEDs' hues (red ~178, green ~77). A green LED's
# blue-ish edge is green, not a third colour (Oct 16 footage).
SURVEY_HUES = {
    'red': ((0, 38), (128, 180)),
    'green': ((38, 128),),
}


def hsv_masks(rgb, hues=None):
    """{colour: HxW bool} - the pixels that are bright, strongly coloured and in that
    colour's hue ranges (HUES unless hues is given). rgb: HxWx3 uint8."""
    import cv2
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    ok = (v >= V_MIN) & (s >= S_MIN)
    out = {}
    for colour, ranges in (hues or HUES).items():
        m = np.zeros(h.shape, bool)
        for a, b in ranges:
            m |= (h >= a) & (h < b)
        out[colour] = ok & m
    return out


def colour_fractions(rgb):
    """Share of the picture (0-1) that is bright and red, green, blue or yellow."""
    return {c: float(m.mean()) for c, m in hsv_masks(rgb).items()}


def red_green_fractions(rgb):
    """Share of the picture that is a survey colour (SURVEY_HUES): red or green."""
    return {c: float(m.mean()) for c, m in hsv_masks(rgb, SURVEY_HUES).items()}


class BoxTracker:
    def __init__(self, min_frac=0.02, gap_s=1.0, same_m=0.5, new_frac=0.04, minor_share=0.35,
                 palette='all'):
        self.min_frac, self.gap_s, self.same_m = min_frac, gap_s, same_m
        self.new_frac, self.minor_share = new_frac, minor_share
        self.fractions = red_green_fractions if palette == 'red-green' else colour_fractions
        self.boxes = []            # dicts: n, e, colour, colours {colour: peak share}, sightings, first_t, last_t
        self._cur = {}             # colour -> the sighting of it in progress

    def update(self, t, rgb, pos):
        """One frame at time t (s); pos = (north, east) of the camera, or None.
        Returns the boxes whose sightings just ended (new or seen again): a list."""
        f = self.fractions(rgb)
        top = max(f.values())
        done = []
        for colour, share in f.items():
            c = self._cur.get(colour)
            if share >= self.min_frac and share >= self.minor_share * top:
                if c is None:
                    c = self._cur[colour] = {'t0': t, 'best': 0.0, 'pos': pos}
                c['last'] = t
                if share > c['best']:
                    c['best'] = share
                    if pos is not None:
                        c['pos'] = pos
            elif c is not None and t - c['last'] > self.gap_s:
                done.append(self._finish(colour, self._cur.pop(colour)))
        return [b for b in done if b is not None]

    def flush(self):
        """End the sightings still in view (the mission is stopping)."""
        done = [self._finish(k, c) for k, c in self._cur.items()]
        self._cur = {}
        return [b for b in done if b is not None]

    def _finish(self, colour, c):
        pos = c['pos']
        for b in self.boxes:
            if (b['colour'] == colour and pos is not None and b['n'] is not None
                    and math.hypot(pos[0] - b['n'], pos[1] - b['e']) <= self.same_m):
                b['sightings'] += 1
                b['last_t'] = c['last']
                b['colours'][colour] = max(b['colours'][colour], c['best'])
                b['new'] = False
                return b
        if c['best'] < self.new_frac:
            return None                     # a glimpse, not a box
        b = {'n': pos[0] if pos else None, 'e': pos[1] if pos else None, 'colour': colour,
             'colours': {colour: c['best']}, 'sightings': 1, 'first_t': c['t0'], 'last_t': c['last'],
             'new': True, 'number': len(self.boxes) + 1}
        self.boxes.append(b)
        return b


class LightTracker:
    """The survey's lights (Ruth, Oct 17): each lit LED is a BLOB of red or green (the
    hue it is nearer, SURVEY_HUES), followed from frame to frame, so two
    GREEN lights in view one after the other stay two lights. BoxTracker's share of the
    whole picture ran them together when the next came into view before the last had
    gone (sim, Oct 17: lights 0.8 m apart, 4 of 5 found). A light is placed where the
    camera was when its blob was nearest the middle of the picture - straight under the
    camera, whichever way the camera is turned. Repeat sightings of a colour within
    same_m are that light again; boxes holds the lights, in BoxTracker's format."""

    def __init__(self, min_frac=0.02, gap_s=1.0, same_m=0.3, new_frac=0.04, link=0.6):
        self.min_frac, self.gap_s, self.same_m, self.new_frac = min_frac, gap_s, same_m, new_frac
        self.link = link            # a blob this close (picture half-widths) to a track's last is that track:
                                    # 0.6 kept one track per real light on the Oct 15-16 footage (0.35 split
                                    # halos), and still parts lights ~0.4 m apart
        self.boxes = []
        self._tracks = []

    @staticmethod
    def blobs(rgb, min_frac):
        """(colour, x, y, share): x, y from -1 to 1 across the picture, share of its area."""
        import cv2
        h, w = rgb.shape[:2]
        out = []
        for colour, mask in hsv_masks(rgb, SURVEY_HUES).items():
            n, _, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
            for k in range(1, n):
                share = stats[k, cv2.CC_STAT_AREA] / float(h * w)
                if share >= min_frac:
                    out.append((colour, cents[k][0] / w * 2 - 1, cents[k][1] / h * 2 - 1, share))
        return out

    def update(self, t, rgb, pos):
        """One frame at time t (s); pos = (north, east) of the camera, or None.
        Returns the lights whose sightings just ended (new or seen again): a list."""
        free = list(self._tracks)
        for colour, x, y, share in sorted(self.blobs(rgb, self.min_frac), key=lambda b: -b[3]):
            best = None
            for tr in free:
                dd = math.hypot(x - tr['x'], y - tr['y'])
                if tr['colour'] == colour and dd <= self.link and (best is None or dd < best[0]):
                    best = (dd, tr)
            if best:
                tr = best[1]
                free.remove(tr)
            else:
                tr = {'colour': colour, 't0': t, 'best': 0.0, 'mid': 9.0, 'pos': pos}
                self._tracks.append(tr)
            tr['x'], tr['y'], tr['last'], tr['best'] = x, y, t, max(tr['best'], share)
            if math.hypot(x, y) < tr['mid']:
                tr['mid'] = math.hypot(x, y)
                if pos is not None:
                    tr['pos'] = pos
        done = []
        for tr in [tr for tr in self._tracks if t - tr['last'] > self.gap_s]:
            self._tracks.remove(tr)
            done.append(self._finish(tr))
        return [b for b in done if b is not None]

    def flush(self):
        done = [self._finish(tr) for tr in self._tracks]
        self._tracks = []
        return [b for b in done if b is not None]

    def _finish(self, tr):
        colour, pos = tr['colour'], tr['pos']
        for b in self.boxes:
            if (b['colour'] == colour and pos is not None and b['n'] is not None
                    and math.hypot(pos[0] - b['n'], pos[1] - b['e']) <= self.same_m):
                b['sightings'] += 1
                b['last_t'] = tr['last']
                b['colours'][colour] = max(b['colours'][colour], tr['best'])
                b['new'] = False
                return b
        if tr['best'] < self.new_frac:
            return None                     # a glimpse, not a light
        b = {'n': pos[0] if pos else None, 'e': pos[1] if pos else None, 'colour': colour,
             'colours': {colour: tr['best']}, 'sightings': 1, 'first_t': tr['t0'], 'last_t': tr['last'],
             'new': True, 'number': len(self.boxes) + 1}
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

    def __init__(self, url, where, report, on_box=None, tracker=None, **tracker_args):
        self.url, self.where, self.report, self.on_box = url, where, report, on_box
        self.tracker = tracker if tracker is not None else BoxTracker(**tracker_args)
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
                    for b in self.tracker.update(t, rgb, self.where()):
                        if self.on_box:
                            self.on_box(b)
            except Exception as ex:                  # no stream yet, cv2 missing, stream dropped
                if not warned:
                    self.report(f'camera: no picture from {self.url} ({ex}) - is tools/cam_pair.py running? '
                                f'Trying again every 5 s')
                    warned = True
            self.stop.wait(5.0)

    def close(self):
        self.stop.set()
        for b in self.tracker.flush():
            if self.on_box:
                self.on_box(b)
        return self.tracker.boxes
