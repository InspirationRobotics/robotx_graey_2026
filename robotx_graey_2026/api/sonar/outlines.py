"""Outlines: crumbs that sit close together, joined up, and how pipe-like each group is.

Map v2, step 1 (Ruth's idea): not just straight lines - anything close together,
so a pool wall gets traced round its corners and the pipeline round its elbows.
Then look at each traced shape and say how much it looks like the pipeline.

OUTLINE. Every crumb is joined to the crumbs nearest it by the shortest set of
lines that connects them all (a "minimum spanning tree"). Lines longer than
LINK_M are dropped, which splits the crumbs into GROUPS: two crumbs are in the
same group if you can walk from one to the other in steps shorter than LINK_M.
Groups of fewer than MIN_CRUMBS crumbs are left out as noise.

SPINE. The longest walk through a group's lines, end to end, with wiggles
smaller than SIMPLIFY_M smoothed out. For the pipeline that is the pipe itself,
bends included; for a wall it is the wall. Its length is the group's LENGTH.

WIDTH is measured close up, so bends don't count: round each crumb, the crumbs
within LINK_M of it, and how wide that little cloud is across its narrow way
(twice the spread). Along a pipe or wall that is the scatter, ~0.1-0.2 m; in a
patch of seafloor the cloud is round and it comes out ~0.35 m or more.

PIPE SCORE, 0 to 1, three parts multiplied:
  length  the pipeline is ~4.7 m of pipe, ~2.75 m end to end (handbook 3.5.5),
          and a scan may only catch part of it: 1 from 1.5 m to 6 m, sliding to
          0 at 0.5 m and at 10 m (a pool wall is longer)
  width   a 3" pipe is thin, and crumbs scatter round it: 1 up to 0.2 m,
          sliding to 0 at 0.35 m (a patch of seafloor). Guesses from made-up
          crumbs - to be tuned on real ones
  crumbs  a handful could be anything: n / 15, up to 1

All of this is top-down (x, y). TOP is the height of the group's highest part,
relative to the DVL when it was pinged ("up" in crumbs.py); hold depth while
scanning and it says how far below the sub the pipe's top is.
"""
import numpy as np

LINK_M = 0.5            # crumbs closer than this are joined
MIN_CRUMBS = 6
SIMPLIFY_M = 0.25
LENGTH_OK_M = (1.5, 6.0)
LENGTH_ZERO_M = (0.5, 10.0)
WIDTH_OK_M, WIDTH_ZERO_M = 0.2, 0.35
FULL_CRUMBS = 15


def _tree(p):
    """Minimum spanning tree of points p (n x 2), Prim's: parent[i] and the
    length of the line from i to it (root: -1, 0)."""
    n = len(p)
    parent = np.full(n, -1)
    best = np.full(n, np.inf)
    done = np.zeros(n, bool)
    cur = 0
    for _ in range(n - 1):
        done[cur] = True
        d = np.hypot(p[:, 0] - p[cur, 0], p[:, 1] - p[cur, 1])
        closer = ~done & (d < best)
        best[closer] = d[closer]
        parent[closer] = cur
        cur = int(np.argmin(np.where(done, np.inf, best)))
    best[0] = 0.0
    return parent, best


def _farthest(adj, start):
    """Farthest node from start along the tree's lines, and the path to it."""
    dist, prev, stack = {start: 0.0}, {start: None}, [start]
    while stack:
        a = stack.pop()
        for b, w in adj[a]:
            if b not in dist:
                dist[b], prev[b] = dist[a] + w, a
                stack.append(b)
    end = max(dist, key=dist.get)
    path = [end]
    while prev[path[-1]] is not None:
        path.append(prev[path[-1]])
    return end, path


def _simplify(pts, tol):
    """Douglas-Peucker: drop points within tol of the line through their neighbours."""
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    ab = b - a
    l2 = float(ab @ ab)
    if l2 < 1e-12:
        d = np.hypot(*(pts - a).T)
    else:
        t = np.clip((pts - a) @ ab / l2, 0, 1)
        d = np.hypot(*(pts - (a + t[:, None] * ab)).T)
    i = int(np.argmax(d))
    if d[i] <= tol:
        return np.vstack([a, b])
    return np.vstack([_simplify(pts[:i + 1], tol)[:-1], _simplify(pts[i:], tol)])


def dist_to_polyline(p, line):
    """Distance from each point in p (n x 2) to the polyline (m x 2)."""
    best = np.full(len(p), np.inf)
    for a, b in zip(line, line[1:]):
        ab = b - a
        t = np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0, 1)
        best = np.minimum(best, np.hypot(*(p - (a + t[:, None] * ab)).T))
    return best


def local_width(p, r=LINK_M):
    """Mean, over the crumbs, of the width of the cloud within r of each."""
    w = []
    for q in p:
        nb = p[np.hypot(p[:, 0] - q[0], p[:, 1] - q[1]) < r]
        if len(nb) >= 3:
            w.append(2.0 * np.sqrt(max(np.linalg.eigvalsh(np.cov(nb.T))[0], 0.0)))
    return float(np.mean(w)) if w else 0.0


def _ramp(v, zero_lo, ok_lo, ok_hi, zero_hi):
    if v <= zero_lo or v >= zero_hi:
        return 0.0
    if v < ok_lo:
        return (v - zero_lo) / (ok_lo - zero_lo)
    if v > ok_hi:
        return (zero_hi - v) / (zero_hi - ok_hi)
    return 1.0


def pipe_score(length, width, n):
    """The three parts, each 0-1, and their product."""
    parts = {"length": _ramp(length, LENGTH_ZERO_M[0], *LENGTH_OK_M, LENGTH_ZERO_M[1]),
             "width": float(np.clip((WIDTH_ZERO_M - width) / (WIDTH_ZERO_M - WIDTH_OK_M), 0, 1)),
             "crumbs": min(1.0, n / FULL_CRUMBS)}
    return parts["length"] * parts["width"] * parts["crumbs"], parts


def find(x, y, up=None, link_m=LINK_M, min_crumbs=MIN_CRUMBS):
    """Group the crumbs and score each group, best first. Each group is a dict:
      n, length, width, score, parts   see the module notes
      top                              highest "up" in the group (None without up)
      lines   [[x1, y1, x2, y2], ...]  the outline
      spine   [[x, y], ...]            end to end, simplified
      members indices into x/y
    """
    p = np.column_stack([np.asarray(x, float), np.asarray(y, float)])
    if len(p) < min_crumbs:
        return []
    parent, length = _tree(p)
    joined = (parent >= 0) & (length < link_m)

    # groups: every joined line merges the two crumbs' groups
    root = list(range(len(p)))

    def top(i):
        while root[i] != i:
            root[i] = root[root[i]]
            i = root[i]
        return i
    for i in np.flatnonzero(joined):
        root[top(i)] = top(int(parent[i]))
    label = np.array([top(i) for i in range(len(p))])

    groups = []
    for g in np.unique(label):
        members = np.flatnonzero(label == g)
        if len(members) < min_crumbs:
            continue
        adj = {int(i): [] for i in members}
        lines = []
        for i in members:
            if joined[i]:
                j, w = int(parent[i]), float(length[i])
                adj[int(i)].append((j, w))
                adj[j].append((int(i), w))
                lines.append([*p[i].round(2).tolist(), *p[j].round(2).tolist()])
        end, _ = _farthest(adj, int(members[0]))
        _, path = _farthest(adj, end)
        spine = _simplify(p[path], SIMPLIFY_M)
        span = float(np.sum(np.hypot(*np.diff(spine, axis=0).T))) if len(spine) > 1 else 0.0
        width = local_width(p[members], link_m)
        score, parts = pipe_score(span, width, len(members))
        groups.append({"n": int(len(members)), "length": round(span, 2), "width": round(width, 3),
                       "score": round(score, 3), "parts": {k: round(v, 2) for k, v in parts.items()},
                       "top": None if up is None else round(float(np.max(np.asarray(up)[members])), 2),
                       "lines": lines, "spine": spine.round(2).tolist(),
                       "members": members.tolist()})
    groups.sort(key=lambda gr: -gr["score"])
    return groups


class Outliner:
    """For the Map tab: find() again when the crumbs have changed, at most
    every_s seconds apart (3000 crumbs take ~0.25 s), as JSON-ready dicts."""

    def __init__(self, every_s=2.0):
        self.every_s = every_s
        self.clear()

    def clear(self):
        self.groups, self._key, self._t = [], None, -1e9

    def update(self, c, now, force=False):
        """c: CrumbMap.crumbs(). Returns the groups, best first, without members."""
        key = (len(c["x"]), float(np.sum(c["x"])), float(np.sum(c["seen"])))
        if key != self._key and (force or now - self._t >= self.every_s):
            self._key, self._t = key, now
            self.groups = [{k: v for k, v in g.items() if k != "members"}
                           for g in find(c["x"], c["y"], c["up"])]
        return self.groups
