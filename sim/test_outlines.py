"""Checks for outlines.py: pipe vs wall vs blob vs noise, bends kept, timing.

    PYTHONPATH=. python3 sim/test_outlines.py
"""
import time
import numpy as np
from robotx_graey_2026.api.sonar import outlines as O

rng = np.random.default_rng(0)
def along(pts, step=0.15, sd=0.12):
    out = []
    for a, b in zip(pts, pts[1:]):
        a, b = np.array(a, float), np.array(b, float)
        n = int(np.linalg.norm(b - a) / step)
        out.append(a + (b - a) * np.linspace(0, 1, n, endpoint=False)[:, None])
    p = np.vstack(out)
    return p + rng.normal(0, sd, p.shape)

# pipeline, top view: 0.95 straight, 0.71 (45 up, projected), 0.95, 0.71 at 45 right, 0.95 at 90
pipe = along([(0, 0), (0, 1.66), (0, 2.61), (0.5, 3.11), (1.45, 3.11)])
pipe += (5, 5)
# pool wall with a 90 deg corner, 12 m long
wall = along([(-8, -6), (-8, 2), (-4, 2)])
# a 2.5 x 2.5 m patch of seafloor
blob = rng.uniform(0, 2.5, (150, 2)) + (6, -6)
# scattered noise
noise = rng.uniform(-10, 10, (30, 2))
allp = np.vstack([pipe, wall, blob, noise])
kind = np.array(["pipe"] * len(pipe) + ["wall"] * len(wall) + ["blob"] * len(blob) + ["noise"] * len(noise))
up = np.where(kind == "pipe", -7.5, -9.0)

t = time.perf_counter()
gs = O.find(allp[:, 0], allp[:, 1], up)
dt = time.perf_counter() - t
for g in gs:
    ks, cnt = np.unique(kind[g["members"]], return_counts=True)
    print(f"score {g['score']:.2f} n {g['n']:3d} length {g['length']:5.2f} width {g['width']:.2f} "
          f"top {g['top']} parts {g['parts']} bends {len(g['spine'])-2} members {dict(zip(ks, cnt))}")
best = gs[0]
assert np.mean(kind[best["members"]] == "pipe") > 0.9, "best group should be the pipe"
assert best["score"] > 0.8, best["score"]
assert best["top"] == -7.5
others = [g for g in gs[1:]]
assert all(g["score"] < 0.3 for g in others), [g["score"] for g in others]
assert O.find([0, 1], [0, 1]) == []
# timing with a big map
big = rng.uniform(-15, 15, (3000, 2))
t = time.perf_counter(); O.find(big[:, 0], big[:, 1]); dbig = time.perf_counter() - t
print(f"time: {dt*1000:.0f} ms for {len(allp)} crumbs, {dbig*1000:.0f} ms for 3000")
print("ok")
