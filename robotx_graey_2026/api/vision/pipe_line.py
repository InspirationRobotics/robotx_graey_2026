"""Classical pipe finder for the DOWN camera (Ruth, Oct 26 idea): the white PVC pipe as a bright,
colourless, long thin line. Up to two straight arms (a bend), where each arm ends inside the
picture (a pipe end), and a 0-1 confidence. No training.

Picture coordinates are the 320x180 working size; the top of the picture is the sub's front.
Developed and scored in camera_rec/pipe_classical (Ruth's labelled pipe lines and light boxes); the
mission's camera_follow uses detect() and Steady on every picture of the camera thread.
"""
import collections, math
import numpy as np
import cv2

W, H = 320, 180
TOPHAT_K = 61           # bright structures narrower than this (px) stand out from their background
SAT_K = 61
MIN_TOPHAT = 15         # at least this much brighter than around it (0-255 L)
Z_MIN = 4.0             # ... and this many noise widths
SRISE_MAX = 25          # ... and not this much more colourful than around it (the lights)
MIN_LEN = 70            # an arm is at least this long (px)
MIN_ELONG = 4.0         # ... and this many times longer than wide
END_MARGIN = 15         # an arm ending this far inside the picture is a pipe end (or an elbow)
MAGNET = (138, 100, 200, 180)   # x0, y0, x1, y1: the magnet puck hides the pipe here - not an end
CONF_MIN = 0.7          # trust a line at this confidence or more (Oct 26 tuning on Ruth's labels:
                        # fewer wrong lines over more hits - a miss falls back to the sonar)
# parts of the picture fixed to the camera, never the pipe (this mount): the 'DOWN' label, the edge
# of the magnet's clear board (Oct 24 lake mount) - set from a recording's median picture
IGNORE = [(0, 0, 44, 14), (0, 78, W, 100)]
# the light boxes on the pipeline (Ruth, Oct 26): found by their own detector and passed in; the pipe finder
# keeps off them (a box and its glow merged with the pipe and tilted the line), an arm that runs into one
# is not a pipe end, and pipe on either side of one is one pipeline
BOX_GROW = 6            # px around each box also left out: its glow
BOX_TOUCH = 10          # px: an arm's end this near a box runs into it
BEND_CONT = 0.7         # a bend under a box: the next arm a solid line, not scattered blotches lined up
TRY_BLOBS = 1           # the best blobs tried in turn for the pipe's line (3: +4 right, +7 wrong, Oct 26)
BOX_AIM_DEG = 20        # a piece of pipe joins a box when it points at the box's middle this closely (25 let
                        # floor blotches either side of a box join into a line: +3 wrong, Oct 26)


def _mad(x):
    return 1.4826 * float(np.median(np.abs(x - np.median(x)))) + 1e-3


def pipe_mask(im, boxes=()):
    """Pixels that look like white PVC: brighter than around them and less coloured."""
    lab = cv2.cvtColor(im, cv2.COLOR_BGR2LAB)
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    L, S = lab[..., 0], hsv[..., 1]
    p = TOPHAT_K // 2                       # padded by reflection: the edges lit up unpadded
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (TOPHAT_K, TOPHAT_K))
    Lp = cv2.copyMakeBorder(L, p, p, p, p, cv2.BORDER_REFLECT)
    th = cv2.morphologyEx(Lp, cv2.MORPH_TOPHAT, k)[p:-p, p:-p].astype(np.float32)
    ks = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (SAT_K, SAT_K))
    Sp = cv2.copyMakeBorder(S, p, p, p, p, cv2.BORDER_REFLECT)
    # more colour than around it: the lights. NOT 'colourless' - in green lake water the white pipe is as
    # green as the water (S 135-158 beside water at 127-159, Oct 24 frame 46504), and that test lost it
    srise = cv2.morphologyEx(Sp, cv2.MORPH_TOPHAT, ks)[p:-p, p:-p].astype(np.float32)
    use = np.ones(th.shape, bool)
    for x0, y0, x1, y1 in IGNORE:
        use[y0:y1, x0:x1] = False
    for x0, y0, x1, y1 in boxes:
        use[max(0, int(y0)):max(0, int(y1) + 1), max(0, int(x0)):max(0, int(x1) + 1)] = False
    use[:16], use[-12:], use[:, :4], use[:, -4:] = False, False, False, False    # the picture's own edges
    th[~use] = 0                            # made lines: bright bands along the top and bottom of lake frames
    z = th / _mad(th[use])                  # the noise of the part looked at (blanked zeros raised it to 113)
    m = ((th >= MIN_TOPHAT) & (z >= Z_MIN) & (srise < SRISE_MAX) & (L > 40)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    m = np.isin(lab, np.flatnonzero(st[:, cv2.CC_STAT_AREA] >= 8)) & (lab > 0)   # specks out; thin lines stay
    return m, th


def _join_boxes(m, boxes):
    """The pipe runs through each light box (Ruth, Oct 26): a long piece that runs into a box and points at
    its middle is drawn on to the middle, so the pieces either side of a box are one again, and one that
    runs into it isn't cut short (the lake boxes are clear: the pipe showed through, blanking cut it)."""
    if not boxes:
        return m
    out = m.astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(out, connectivity=8)
    for x0, y0, x1, y1 in boxes:
        mid = np.array([(x0 + x1) / 2, (y0 + y1) / 2])
        a0, b0 = max(0, int(y0 - BOX_TOUCH)), max(0, int(y1 + BOX_TOUCH) + 1)
        a1, b1 = max(0, int(x0 - BOX_TOUCH)), max(0, int(x1 + BOX_TOUCH) + 1)
        for k in np.unique(lab[a0:b0, a1:b1]):
            if k == 0 or st[k, cv2.CC_STAT_AREA] < 25:
                continue
            ys, xs = np.nonzero(lab == k)
            P = np.column_stack([xs, ys]).astype(np.float32)
            (_, _), (rw, rh), _ = cv2.minAreaRect(P)
            length, width = max(rw, rh), max(1.0, min(rw, rh))
            if length / width < 1.8:
                continue                        # a blotch, not a piece of pipe (1.8: a piece cut short by the
                                                # box and the picture's edge is stubby)
            c = P.mean(axis=0)
            _, _, vt = np.linalg.svd(P - c, full_matrices=False)
            to = mid - c
            if np.linalg.norm(to) < 1:
                continue
            cosang = abs(float(vt[0] @ to)) / float(np.linalg.norm(to))
            if cosang < math.cos(math.radians(BOX_AIM_DEG)):
                continue                        # passes by the box, not into it
            near = P[np.hypot(*(P - mid).T).argmin()]        # its end nearest the box's middle
            cv2.line(out, tuple(int(v) for v in near), tuple(int(v) for v in mid), 1,
                     int(np.clip(round(width), 3, 12)))     # no wider than the pipe: glare fattens a piece
    return out.astype(bool)


def _components(m):
    """Long thin blobs: (points Nx2 float, length, width)."""
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m.astype(np.uint8), connectivity=8)
    out = []
    for j in range(1, n):
        if stats[j, cv2.CC_STAT_AREA] < 25:
            continue
        ys, xs = np.nonzero(lab == j)
        P = np.column_stack([xs, ys]).astype(np.float32)
        c = P.mean(axis=0)
        _, _, vt = np.linalg.svd(P - c, full_matrices=False)
        s = (P - c) @ vt[0]
        length = float(np.percentile(s, 99) - np.percentile(s, 1)) + 1
        width = len(P) / length
        across = np.abs((P - c) @ vt[1])
        straight = float(np.mean(across <= max(2.0, 0.75 * width)))
        if length >= 20 and length / max(width, 1.0) >= 2.5:
            out.append((P, length, width, straight))
    return out


def _ransac(P, tol, iters=250, rng=None):
    """Best line through P: (point, unit dir, inlier mask)."""
    rng = rng or np.random.default_rng(0)
    best, bn = None, 0
    n = len(P)
    if n < 10:
        return None
    for _ in range(iters):
        i, j = rng.integers(0, n, 2)
        d = P[j] - P[i]
        L = float(np.hypot(*d))
        if L < 15:
            continue
        d = d / L
        dist = np.abs((P[:, 0] - P[i, 0]) * d[1] - (P[:, 1] - P[i, 1]) * d[0])
        inl = dist <= tol
        k = int(inl.sum())
        if k > bn:
            bn, best = k, inl
    if best is None:
        return None
    vx, vy, x0, y0 = cv2.fitLine(P[best], cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    d = np.array([vx, vy], float)
    p0 = np.array([x0, y0], float)
    dist = np.abs((P[:, 0] - p0[0]) * d[1] - (P[:, 1] - p0[1]) * d[0])
    return p0, d, dist <= tol


def _in_box(q, boxes, pad=0.0):
    return any(x0 - pad <= q[0] <= x1 + pad and y0 - pad <= q[1] <= y1 + pad for x0, y0, x1, y1 in boxes)


def _arm(P, p0, d, inl, th_img, boxes=()):
    """An arm's measurements from its inlier points."""
    Q = P[inl]
    s = (Q - p0) @ d
    lo, hi = float(np.percentile(s, 2)), float(np.percentile(s, 98))
    length = hi - lo
    across = (Q - p0) @ np.array([-d[1], d[0]])
    width = max(1.0, float(np.percentile(across, 95) - np.percentile(across, 5)))
    bins = set(np.floor((s - lo) / 4.0).astype(int).tolist())
    for b in range(int(length / 4.0) + 1):      # the pipe hidden under a light box counts as there
        if b not in bins and _in_box(p0 + (lo + 4 * b + 2) * d, boxes):
            bins.add(b)
    cont = len(bins) / max(1.0, length / 4.0 + 1)
    a, b = p0 + lo * d, p0 + hi * d
    vals = th_img[Q[:, 1].astype(int), Q[:, 0].astype(int)]
    seen = np.array([not _in_box(q, boxes) for q in Q], bool) if boxes else np.ones(len(Q), bool)
    contrast = float(np.median(vals[seen] if seen.any() else vals))     # not the drawn-in part under a box
    return {'a': a, 'b': b, 'dir': d, 'len': length, 'width': width, 'cont': min(1.0, cont),
            'contrast': contrast, 'n': int(inl.sum())}


def _inside(p):
    return END_MARGIN <= p[0] <= W - 1 - END_MARGIN and 16 + END_MARGIN <= p[1] <= H - 1 - END_MARGIN


def _angle_up(d):
    """The arm's angle from straight ahead (the picture's top), -90..90 deg, + = turning right."""
    a = math.degrees(math.atan2(d[0], -d[1]))
    while a > 90:
        a -= 180
    while a <= -90:
        a += 180
    return a


def detect(bgr, boxes=None):
    """The pipe in one down-camera picture: dict with conf (0-1); when found also angle_deg,
    offset_px (+ = right of the middle), arms, bend (elbow xy, next arm's angle) and ends.
    boxes: the light boxes in it as (left, top, width, height), shares of the picture (YoloLights' 'box')."""
    im = cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA)
    bx = [(l * W - BOX_GROW, t * H - BOX_GROW, (l + w) * W + BOX_GROW, (t + h) * H + BOX_GROW)
          for l, t, w, h in (boxes or ())]
    m, th = pipe_mask(im, bx)
    m = _join_boxes(m, bx)
    comps = _components(m)
    res = {'conf': 0.0, 'arms': [], 'ends': [], 'bend': None, 'mask': m, 'boxes': bx}
    if not comps:
        return res
    P = np.vstack([c[0] for c in comps])
    if len(P) > 4000:
        P = P[np.random.default_rng(1).choice(len(P), 4000, replace=False)]

    hidden = np.zeros((H, W), bool)         # under a box: drawn in, so no brightness of its own
    for x0, y0, x1, y1 in bx:
        hidden[max(0, int(y0)):max(0, int(y1) + 1), max(0, int(x0)):max(0, int(x1) + 1)] = True

    def bright(c):
        Q = c[0].astype(int)
        v, seen = th[Q[:, 1], Q[:, 0]], ~hidden[Q[:, 1], Q[:, 0]]
        return float(np.median(v[seen] if seen.any() else v))

    # the main blob: long, thin, straight and bright - the pipe is the whitest straight thing. If the best
    # one's line is too short, the next: a box splits the pipe into shorter pieces, and a floor blotch that
    # scored higher but failed must not hide them (Oct 26)
    order = sorted(comps, key=lambda c: -c[1] * min(c[1] / max(c[2], 1), 10) * c[3] ** 2 * bright(c))
    for main in order[:TRY_BLOBS]:
        tol = float(np.clip(0.6 * main[2], 3.0, 14.0))
        r0 = _ransac(main[0], tol)
        if r0 is None:
            continue
        p0, d0, _ = r0
        dist = np.abs((P[:, 0] - p0[0]) * d0[1] - (P[:, 1] - p0[1]) * d0[0])
        r1 = (p0, d0, dist <= tol)          # the main blob's line, with every blob on it
        arm1 = _arm(P, *r1, th, bx)
        if arm1['len'] >= MIN_LEN and arm1['len'] / arm1['width'] >= MIN_ELONG:
            break
    else:
        return res
    arms = [arm1]
    rest = P[~r1[2]]
    r2 = _ransac(rest, tol) if len(rest) >= 30 else None
    if r2 is not None:
        arm2 = _arm(rest, *r2, th, bx)
        ang = abs(_angle_up(arm2['dir']) - _angle_up(arm1['dir']))
        ang = min(ang, 180 - ang)
        ends1, ends2 = (arm1['a'], arm1['b']), (arm2['a'], arm2['b'])
        gap = min(float(np.hypot(*(e1 - e2))) for e1 in ends1 for e2 in ends2)
        via_box = any(_in_box(e1, [b], BOX_TOUCH) and _in_box(e2, [b], BOX_TOUCH)
                      for b in bx for e1 in ends1 for e2 in ends2)     # the bend is under a box
        if (arm2['len'] >= 0.6 * MIN_LEN and arm2['len'] / arm2['width'] >= MIN_ELONG * 0.75
                and ang >= 15 and (gap <= max(18.0, 2 * tol) or via_box and arm2['cont'] >= BEND_CONT)):
            arms.append(arm2)
    # steer by the arm nearest the picture's middle (the one under the sub)
    mid = np.array([W / 2, H / 2])

    def dist_mid(arm):
        d = arm['dir']
        return abs((mid[0] - arm['a'][0]) * d[1] - (mid[1] - arm['a'][1]) * d[0])

    arms.sort(key=dist_mid)
    cur = arms[0]
    ends = []
    elbow = None
    if len(arms) == 2:
        A, B = arms
        e = min(((ea, eb) for ea in (A['a'], A['b']) for eb in (B['a'], B['b'])),
                key=lambda t: float(np.hypot(*(t[0] - t[1]))))
        elbow = (e[0] + e[1]) / 2
    for arm in arms:
        for p, out in ((arm['a'], -arm['dir']), (arm['b'], arm['dir'])):
            if not _inside(p) or (elbow is not None and float(np.hypot(*(p - elbow))) <= 20):
                continue
            q = p + out * 12
            if MAGNET[0] <= q[0] <= MAGNET[2] and MAGNET[1] <= q[1] <= MAGNET[3]:
                continue                    # it goes under the magnet
            if _in_box(p, bx, BOX_TOUCH):
                continue                    # it runs into a light box
            # past a real end there is no bright line; past a gap in the mask there still is
            ts = [p + out * t for t in range(4, 30, 2)]
            ts = [t for t in ts if 0 <= t[0] < W and 0 <= t[1] < H]
            beyond = float(np.median([th[int(t[1]), int(t[0])] for t in ts])) if ts else 0.0
            if beyond < 0.4 * arm['contrast']:
                ends.append(p)
    d = cur['dir']
    n = np.array([-d[1], d[0]])
    if n[0] < 0:
        n = -n
    off = float((cur['a'] - mid) @ n)        # + = the pipe passes right of the middle
    elong = min(1.0, (cur['len'] / cur['width'] - MIN_ELONG) / 8 + 0.3)
    length = min(1.0, cur['len'] / 120)
    contrast = min(1.0, cur['contrast'] / 35)
    conf = float(np.clip((max(elong, 0) * length * contrast) ** (1 / 3) * cur['cont'], 0, 1))
    res.update(conf=conf, angle_deg=_angle_up(d), offset_px=off, arms=arms, ends=ends,
               bend=None if elbow is None else {'at': elbow, 'next_deg': _angle_up(arms[1]['dir'])})
    return res


def draw(bgr, r):
    """The picture at 320x180 with the finding drawn on it."""
    im = cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA)
    ov = im.copy()
    ov[r['mask']] = (255, 0, 255)
    im = cv2.addWeighted(ov, 0.35, im, 0.65, 0)
    for k, arm in enumerate(r['arms']):
        col = (0, 255, 0) if k == 0 else (255, 255, 0)
        cv2.line(im, tuple(int(v) for v in arm['a']), tuple(int(v) for v in arm['b']), col, 2)
    if r['bend']:
        cv2.circle(im, tuple(int(v) for v in r['bend']['at']), 6, (0, 200, 255), 2)
    for e in r['ends']:
        cv2.circle(im, tuple(int(v) for v in e), 7, (0, 0, 255), 2)
    for x0, y0, x1, y1 in r.get('boxes', ()):
        cv2.rectangle(im, (int(x0), int(y0)), (int(x1), int(y1)), (0, 165, 255), 1)
    txt = f"conf {r['conf']:.2f}"
    if r['arms']:
        txt += f"  {r['angle_deg']:+.0f}deg  {r['offset_px']:+.0f}px"
    if r['bend']:
        txt += '  BEND'
    if r['ends']:
        txt += '  END'
    cv2.rectangle(im, (0, H - 16), (W, H), (0, 0, 0), -1)
    cv2.putText(im, txt, (4, H - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return im


class Steady:
    """Trust a line only once it has stayed put (Oct 26): this picture's line counts when at least
    `need` of the last `window` pictures (~1 s at ~12 fps) had a confident line at about the same
    angle and place. A flicker - a rock, a reflection - rarely lasts; the pipe does."""

    def __init__(self, window=12, need=6, ang_tol=12.0, off_tol=25.0, conf_min=None):
        self.need, self.ang_tol, self.off_tol = need, ang_tol, off_tol
        self.conf_min = CONF_MIN if conf_min is None else conf_min
        self.hist = collections.deque(maxlen=window)

    def update(self, r):
        """Feed one detect() result; True when its line is steady enough to steer by."""
        line = None
        if r['conf'] >= self.conf_min and r['arms']:
            a = r['arms'][0]
            line = (r['angle_deg'], (a['a'] + a['b']) / 2, a['dir'])
        self.hist.append(line)
        if line is None:
            return False
        ang, mid, _ = line

        def same(h):
            e = abs((h[0] - ang + 90) % 180 - 90)
            d = h[2]                          # how far this line's middle is off the other line
            off = abs((mid[0] - h[1][0]) * d[1] - (mid[1] - h[1][1]) * d[0])
            return e <= self.ang_tol and off <= self.off_tol

        return sum(1 for h in self.hist if h is not None and same(h)) >= self.need
