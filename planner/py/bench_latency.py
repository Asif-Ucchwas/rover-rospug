"""Task 3.9: SE(2) planning latency on 80 fixed corridor queries (built inside this script, so every machine runs the same ones).
Prints median / 95th percentile / max, split into queries that find a path and queries with no path.  Usage: python3 py/bench_latency.py [repeats]"""
import sys, os, time, math, hashlib, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from fpp import Planner, DEFAULT_PRM
from worlds import corridor_world, RES

def queries(P, n=80, seed=123):
    rng = np.random.default_rng(seed); free = np.argwhere(P.clearance_map() > 0.6); out = []
    for _ in range(n):
        s = free[rng.integers(len(free))]; e = free[rng.integers(len(free))]
        sx, sy = s[1] * RES + RES / 2, s[0] * RES + RES / 2; th = float(rng.uniform(-math.pi, math.pi))
        for _ in range(200):
            if not P.rect_hits(sx, sy, th): break
            th = float(rng.uniform(-math.pi, math.pi))
        out.append(((sx, sy, th), (e[1] * RES + RES / 2, e[0] * RES + RES / 2)))
    return out

def run(prm=None, repeats=3, verbose=True):
    g = corridor_world(seed=0); P = Planner(g, RES, margin=0.02); qs = queries(P)
    ts, ex, oks, bad, L = [], [], [], 0, []; h = hashlib.sha1()
    for s, t in qs:
        best = 1e9
        for _ in range(repeats):
            t0 = time.perf_counter(); r = P.plan('se2', s, t, tight=True, prm=prm); best = min(best, time.perf_counter() - t0)
        ts.append(best * 1000); ex.append(r['expanded']); oks.append(bool(r['ok']))
        if r['ok']:
            bad += P.validate(r['path']); h.update(np.round(r['path'], 6).tobytes())
            L.append(float(np.hypot(*np.diff(r['path'][:, :2], axis=0).T).sum()))
        else: L.append(float('nan'))
    ts = np.array(ts); oks = np.array(oks)
    if verbose:
        for name, m in (('all queries', np.ones(len(ts), bool)), ('found a path', oks), ('no path', ~oks)):
            if m.any(): print(f'  {name:14s} n={m.sum():3d}  median {np.median(ts[m]):7.1f} ms  p95 {np.percentile(ts[m], 95):7.1f} ms  max {ts[m].max():7.0f} ms')
        print(f'  found {oks.sum()}/{len(qs)}, collisions on found paths: {bad}, median nodes expanded {int(np.median(ex))}, path hash {h.hexdigest()[:12]}')
    return dict(ts=ts, oks=oks, L=np.array(L), bad=bad)
if __name__ == '__main__': run(repeats=int(sys.argv[1]) if len(sys.argv) > 1 else 3)
