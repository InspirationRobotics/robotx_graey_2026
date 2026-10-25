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

SMALL = (320, 180)             # frames are judged at this size: a lake LED ring ~1 m below is only
                               # 15-300 px of it (Oct 18)

# HSV ranges, on OpenCV's scale: H 0-179 (half degrees: 0 red, 30 yellow, 60 green,
# 120 blue), S and V 0-255. Measured on the pool footage (down camera, Oct 15-17): the
# LEDs' hue is red 175-179 and 0-4, green 70-89 (a blue-ish green on this camera), blue
# 105-109; between green and blue (90-104) is the glow at a light's edge. LED pixels
# have S 130-255; the washed-out white middle and pale halo S < 65. Red sits at both
# ends of the hue circle, so it needs two ranges; orange (to 20) counts as red.
V_MIN = 100                    # bright: the LED, not the water. Red LEDs are dim (V 80-150): at 150 a red
                               # flash was lost in the pool; the lake water is rejected by S, not V
S_MIN = 150                    # strongly coloured. Lake, daylight (Oct 18): the whole picture is olive-green
                               # water over rocks, S ~100-140 at V 140-180, so S 100 called the water a light;
                               # the LED rings are S 150+. Pool LEDs are S 130-255 (median 254)
HUES = {
    'red': ((0, 10), (160, 180)),
    'yellow': ((20, 35),),
    'green': ((35, 97),),
    'blue': ((97, 140),),
}
# The survey's colours (Ruth, Oct 17): while the pipeline is surveyed its lights are only
# GREEN or RED, so a coloured pixel counts as whichever of the two its hue is nearer - except
# orange (H 10-38): the lake pipe has orange tags and cable, which counted as red lights
# (Oct 18), and the red LEDs are H 175-179 and 0-4. A green LED's blue-ish edge is green,
# not a third colour (Oct 16 footage).
SURVEY_HUES = {
    'red': ((0, 10), (128, 180)),
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


MAGNET_PX = (328.0, 292.0)     # the repair magnet's centre in the 640x360 down picture (Oct 22 footage: its rim
                               # in the median of 400 lit frames, radius 49 px). Picture top = the sub's front,
                               # picture right = its right (checked against the DVL the same night)
OPEN_PX = 5                    # SMALL px: wider than the red glare streaks the acrylic board throws (Oct 22)
CORE_V = 240                   # a light's washed-out middle: the LED ring itself


def light_spot(rgb, min_px=20):
    """Where the light is in a SMALL down frame: ((x, y) in 640x360 pixels, share of the picture) of its
    bright core (V >= CORE_V), or None. Not the coloured haze: that leans along the glare streaks, while
    the core sits on the LED ring (Oct 22 footage: the core's middle was 16-34 px from the magnet's when
    the light switched, the red haze's 65-72 px). Thin streaks are opened away first, then the biggest
    patch counts; with the magnet inside the ring its middle is hidden, but the ring keeps the centre.
    Which colour the light is, is red_green_fractions' job."""
    import cv2
    m = (cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)[..., 2] >= CORE_V).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OPEN_PX, OPEN_PX)))
    n, _, st, cen = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < min_px:
        return None
    h, w = m.shape
    return (float(cen[k][0]) * 640 / w, float(cen[k][1]) * 360 / h), float(st[k, cv2.CC_STAT_AREA]) / m.size


def colour_fractions(rgb):
    """Share of the picture (0-1) that is bright and red, green, blue or yellow."""
    return {c: float(m.mean()) for c, m in hsv_masks(rgb).items()}


def red_green_fractions(rgb):
    """Share of the picture that is a survey colour (SURVEY_HUES): red or green."""
    return {c: float(m.mean()) for c, m in hsv_masks(rgb, SURVEY_HUES).items()}


# ---- the lights judged against the water (Ruth, Oct 25) ----
# Lake, Oct 24: the water itself was bright (V ~167) and strongly green (S ~203, hue 40-63), so the
# fixed test above called 82% of every picture a green light, and a red LED 2 m below (10-170 px of
# 640x360) was far under the survey's smallest blob. Here a light is a ROUND patch much brighter than
# the water around it, with a burnt-out middle (every LED seen on Oct 24 had one); its colour is the
# hue of the strongly coloured glow around it. On the Oct 24 footage (03:43-04:21): red in 1146 frames,
# every red event checked by eye a real light; green in 45 (none real - no green LED was on); the
# board's glare streaks, the pale pipe close up and the label are not counted as lights.
# Close up a red LED's glow burns out to yellow-orange (hue 20-37): it reads 'other', not green.
C_BG_K = 31            # SMALL px: the water's local brightness, a blur this wide
C_DV_MIN = 50          # a light: brighter than the water around it by at least this (V, 0-255) ...
C_CORE_V = 240         # ... with a burnt-out white middle
C_MIN_PX = 3           # smallest patch, SMALL px (a lake LED 2 m below: 3-40 SMALL px)
C_MAX_ASPECT = 3.0     # round-ish: glare streaks and the pipe close up are long and thin ...
C_MIN_FILL = 0.3       # ... and don't fill their box
C_BIG = 0.01           # a patch this big (share of the picture) is a light whatever its shape: close up in
                       # the dark pool a green LED is a huge glow with streaks, often off the edge (Oct 16)
C_EDGE_SMALL = 0.001   # light_colour: patches touching the picture's edge count only from this big
C_RING = 5             # SMALL px: the glow is looked at this far around the patch ...
C_GLOW_S = 90          # ... its pixels this strongly coloured (S) ...
C_GLOW_DV = 20         # ... and this much brighter than the water
C_GLOW_N = 4           # fewer coloured glow pixels than this: 'white'
C_LABEL = (50, 18)     # SMALL px: tools/cam_pair.py writes 'DOWN' in the top-left corner
C_MAGNET = (164, 146, 34)   # SMALL px (x, y, r): the magnet and the bright tab on its rim
C_RED_H = ((0, 25), (165, 180))   # red LEDs 175-179 and 0-4 (pool); their glow 9-21 in the lake's green
                                  # water. Orange tags (Oct 18) don't glow, so they can't be a light here
C_GREEN_H = ((60, 92),)           # green LEDs 65-90 (pool videos); the lake water is 40-63, below this
C_BLUE_H = ((92, 135),)           # blue LEDs 95-110 (Oct 17 colour sequence) - not green (Ruth, Oct 25)
C_FAR = 10             # SMALL px: the water just beyond the glow, out to this far ...
C_GREEN_ADD = 10       # ... and a GREEN light adds this much more green than red to it, and about as much
                       # as blue or more (this camera's green LEDs are blue-ish, Oct 16): sunlit spots on
                       # the pale pipe read hue 64-77 too, but add white - red and blue (Oct 24)


def contrast_lights(rgb):
    """The lights in a SMALL down frame, judged against the water (see above): [dict(colour, x, y,
    share, whole, hue, box)], colour 'red', 'green', 'blue', 'other' (a coloured glow of none) or 'white' (no
    coloured glow); x, y from -1 to 1 across the picture; whole = clear of the picture's edge; box = the
    patch's (left, top, width, height) as shares of the picture's width and height."""
    import cv2
    h, w = rgb.shape[:2]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    v = hsv[..., 2].astype(np.float32)
    # the water's brightness nearby - but no brighter than the picture's own middle brightness + 20:
    # a big glow (the dark pool, a light close up) otherwise raised its own 'water' and hid itself
    bg = np.minimum(cv2.blur(v, (C_BG_K, C_BG_K)), float(np.median(v)) + 20.0)
    m = (v - bg >= C_DV_MIN).astype(np.uint8)
    m[:C_LABEL[1], :C_LABEL[0]] = 0
    cv2.circle(m, C_MAGNET[:2], C_MAGNET[2], 0, -1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, connectivity=8)
    out = []
    for k in range(1, n):
        area = int(st[k, cv2.CC_STAT_AREA])
        x0, y0, bw, bh = (int(c) for c in st[k, :4])
        if area < C_MIN_PX:
            continue
        if area / float(h * w) < C_BIG and (max(bw, bh) / max(1, min(bw, bh)) > C_MAX_ASPECT
                                          or area / float(bw * bh) < C_MIN_FILL):
            continue
        pad = C_FAR + 1
        xa, ya, xb, yb = max(0, x0 - pad), max(0, y0 - pad), min(w, x0 + bw + pad), min(h, y0 + bh + pad)
        blob = (lab[ya:yb, xa:xb] == k).astype(np.uint8)
        sub = hsv[ya:yb, xa:xb]
        if not (sub[..., 2][blob > 0] >= C_CORE_V).any():
            continue
        near = cv2.dilate(blob, np.ones((2 * C_RING + 1,) * 2, np.uint8)) > 0
        glow = near & (sub[..., 1] >= C_GLOW_S) & ((v[ya:yb, xa:xb] - bg[ya:yb, xa:xb]) >= C_GLOW_DV)
        colour, hue = 'white', None
        if glow.sum() >= C_GLOW_N:
            ang = sub[..., 0][glow].astype(np.float32) * (np.pi / 90.0)          # hue 0-179 -> 0-2 pi
            hue = float((np.degrees(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())) / 2.0) % 180.0)
            colour = ('red' if any(a <= hue < b for a, b in C_RED_H) else
                      'green' if any(a <= hue < b for a, b in C_GREEN_H) else
                      'blue' if any(a <= hue < b for a, b in C_BLUE_H) else 'other')
            if colour in ('green', 'blue'):
                water = (cv2.dilate(blob, np.ones((2 * C_FAR + 1,) * 2, np.uint8)) > 0) & ~near
                if water.sum() >= 10:
                    px = rgb[ya:yb, xa:xb].astype(np.float32)
                    r, g, b = px[near].mean(axis=0) - px[water].mean(axis=0)     # what the light adds
                    if colour == 'green' and (g - r < C_GREEN_ADD or g - b < -C_GREEN_ADD):
                        colour = 'other'
                    if colour == 'blue' and b - r < C_GREEN_ADD:      # a blue light adds blue, not white
                        colour = 'other'
        out.append({'colour': colour, 'x': float(cen[k][0] / w * 2 - 1), 'y': float(cen[k][1] / h * 2 - 1),
                    'share': area / float(h * w), 'hue': hue, 'box': (x0 / w, y0 / h, bw / w, bh / h),
                    'whole': x0 > 0 and y0 > 0 and x0 + bw < w and y0 + bh < h})
    return out


def light_colour(rgb):
    """The biggest light judged against the water in a SMALL frame - none of the specks at the picture's
    edge (sparkles there read green, Oct 24): (colour, share), or (None, 0.0)."""
    return biggest_colour(contrast_lights(rgb))


# ---- lights from a trained model (Ruth, Oct 25): YOLO through ultralytics, as the prequal's pole
# tracker runs it (api/vision/pole_tracker.py) - in the graey container, on the Jetson's GPU ----
YOLO_COLOURS = {'red_light': 'red', 'green_light': 'green', 'blue_light': 'blue',
                'red': 'red', 'green': 'green', 'blue': 'blue'}


class YoloLights:
    """yolo(bgr) -> the same list contrast_lights gives, from a trained model, on the full-size down
    picture (BGR, 640x360). Classes red_light / green_light / blue_light (or red / green / blue) are the lights' colours;
    any other class (a pipe end, later) keeps its own name as the colour. A light needs conf, a pipe end
    end_conf (the pipe-end model scores real ends 0.28-0.55, Oct 25)."""

    def __init__(self, path, conf=0.4, end_conf=None):
        from ultralytics import YOLO        # slow import, and only in the graey container
        self.model, self.conf = YOLO(path), conf
        self.end_conf = conf if end_conf is None else end_conf
        try:
            import torch
            self.device = 0 if torch.cuda.is_available() else 'cpu'
        except ImportError:
            self.device = 'cpu'

    def __call__(self, bgr):
        h, w = bgr.shape[:2]
        res = self.model.predict(bgr, conf=min(self.conf, self.end_conf), verbose=False, device=self.device)[0]
        out = []
        for b in res.boxes:
            x0, y0, x1, y1 = (float(v) for v in b.xyxy[0].tolist())
            name = res.names[int(b.cls[0])]
            if float(b.conf[0]) < (self.end_conf if name == 'pipe_end' else self.conf):
                continue
            out.append({'colour': YOLO_COLOURS.get(name, name), 'conf': float(b.conf[0]),
                        'x': (x0 + x1) / w - 1, 'y': (y0 + y1) / h - 1,
                        'share': (x1 - x0) * (y1 - y0) / float(w * h), 'hue': None,
                        'box': (x0 / w, y0 / h, (x1 - x0) / w, (y1 - y0) / h),
                        'whole': x0 > 2 and y0 > 2 and x1 < w - 2 and y1 < h - 2})
        return out


def biggest_colour(lights):
    """(colour, share) of the biggest light - none of the specks at the picture's edge - or (None, 0.0)."""
    L = [l for l in lights if l['whole'] or l['share'] >= C_EDGE_SMALL]
    if not L:
        return None, 0.0
    b = max(L, key=lambda l: l['share'])
    return b['colour'], b['share']


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

    def __init__(self, min_frac=0.02, gap_s=1.0, same_m=0.3, new_frac=0.10, link=0.6, detector='hsv',
                 yolo=None):
        # A NEW light must once be seen whole - its blob clear of the picture's edge - or be big
        # (new_frac of the picture): glimpses at the edge (pool, Oct 16-17: 5-8%) sat in the wrong
        # place and were counted twice, while a lake ring ~1 m below is whole but tiny (0.03-6%,
        # Oct 18). min_frac drops specks: 0.02 suits the pool, where the camera is close and the
        # glow breaks into many small pieces; ~0.0003 the lake (light_min_frac).
        self.min_frac, self.gap_s, self.same_m, self.new_frac = min_frac, gap_s, same_m, new_frac
        self.link = link            # a blob this close (picture half-widths) to a track's last is that track:
                                    # 0.6 kept one track per real light on the Oct 15-16 footage (0.35 split
                                    # halos), and still parts lights ~0.4 m apart
        self.detector = detector    # 'hsv': the fixed HSV test; 'contrast': judged against the water (Oct 25);
        self.yolo = yolo            # 'yolo': a YoloLights, on the full picture update() is given
        self.boxes = []
        self._tracks = []

    @staticmethod
    def blobs(rgb, min_frac):
        """(colour, x, y, share, whole): x, y from -1 to 1 across the picture, share of its area,
        whole = the blob is clear of the picture's edge."""
        import cv2
        h, w = rgb.shape[:2]
        out = []
        for colour, mask in hsv_masks(rgb, SURVEY_HUES).items():
            # close small gaps, so a ring's separate LEDs and a glow's pieces are one blob
            mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
            n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
            for k in range(1, n):
                share = stats[k, cv2.CC_STAT_AREA] / float(h * w)
                if share >= min_frac:
                    x0, y0, bw, bh = (int(v) for v in stats[k, :4])
                    whole = x0 > 0 and y0 > 0 and x0 + bw < w and y0 + bh < h
                    out.append((colour, cents[k][0] / w * 2 - 1, cents[k][1] / h * 2 - 1, share, whole))
        return out

    def update(self, t, rgb, pos, full=None, dets=None):
        """One frame at time t (s); pos = (north, east) of the camera, or None; full = the full-size BGR
        picture, or dets = the yolo detector's finds in it already. Returns the lights whose sightings
        just ended (new or seen again)."""
        free = list(self._tracks)
        if self.detector == 'yolo' and self.yolo is not None and (dets is not None or full is not None):
            found = [(l['colour'], l['x'], l['y'], l['share'], l['whole'])
                     for l in (dets if dets is not None else self.yolo(full)) if l['colour'] in ('red', 'green')]
        elif self.detector in ('contrast', 'yolo'):
            found = [(l['colour'], l['x'], l['y'], l['share'], l['whole']) for l in contrast_lights(rgb)
                     if l['colour'] in ('red', 'green')]
        else:
            found = self.blobs(rgb, self.min_frac)
        for colour, x, y, share, whole in sorted(found, key=lambda b: -b[3]):
            best = None
            for tr in free:
                dd = math.hypot(x - tr['x'], y - tr['y'])
                if tr['colour'] == colour and dd <= self.link and (best is None or dd < best[0]):
                    best = (dd, tr)
            if best:
                tr = best[1]
                free.remove(tr)
            else:
                tr = {'colour': colour, 't0': t, 'best': 0.0, 'mid': 9.0, 'pos': pos, 'whole': False}
                self._tracks.append(tr)
            tr['x'], tr['y'], tr['last'], tr['best'] = x, y, t, max(tr['best'], share)
            tr['whole'] = tr['whole'] or whole
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
        if not tr['whole'] and tr['best'] < self.new_frac:
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


def down_half_rgb(jpg, full=False):
    """cam_pair puts DOWN on the left half, FRONT on the right: the left half, small, RGB - and with
    full, (that, the left half full-size BGR)."""
    import cv2
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return (None, None) if full else None
    h, w = img.shape[:2]
    half = img[:, : w // 2] if w > 1.5 * h * 16 / 9 else img      # one camera only: the whole frame
    small = cv2.resize(half, SMALL, interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
    return (rgb, half) if full else rgb


class NoTracker:
    """CameraWatcher's tracker while it only looks for pipe ends (the yolo model): counts nothing."""
    boxes = []

    def update(self, t, rgb, pos, **kw):
        return []

    def flush(self):
        return []


class CameraWatcher:
    """Runs the tracker on the camera stream in a thread. where() gives the camera's
    (north, east) now; report(text) logs; on_box(box) is called as sightings end."""

    def __init__(self, url, where, report, on_box=None, tracker=None, yolo=None, **tracker_args):
        self.url, self.where, self.report, self.on_box = url, where, report, on_box
        self.yolo = yolo                # a YoloLights: run once per picture, its finds shared (latest_dets)
        self.latest_dets = None         # (time.monotonic(), [finds]) of the newest picture
        self.end_seen = None            # (time.monotonic(), find) - the newest picture with a pipe end in it
        self.tracker = tracker if tracker is not None else BoxTracker(**tracker_args)
        self.stop = threading.Event()
        self.frames = 0
        self.latest = None              # (time, SMALL RGB frame) of the newest picture, for centring
        self.latest_full = None         # ... and the same picture full-size BGR (a yolo detector's)
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _run(self):
        warned = False
        while not self.stop.is_set():
            try:
                for t, jpg in frames_from_mjpeg(self.url, self.stop):
                    rgb, full = down_half_rgb(jpg, full=True)
                    if rgb is None:
                        continue
                    self.frames += 1
                    self.latest, self.latest_full = (t, rgb), full
                    if self.frames == 1:
                        self.report(f'camera: watching for light boxes on {self.url}')
                    dets = None
                    if self.yolo is not None:
                        dets = self.yolo(full)
                        self.latest_dets = (t, dets)
                        ends = [d for d in dets if d['colour'] == 'pipe_end']
                        if ends:
                            self.end_seen = (t, max(ends, key=lambda d: d['conf']))
                    found = (self.tracker.update(t, rgb, self.where(), full=full, dets=dets)
                             if isinstance(self.tracker, LightTracker) else self.tracker.update(t, rgb, self.where()))
                    for b in found:
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
