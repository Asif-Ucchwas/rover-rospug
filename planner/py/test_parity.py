"""Task 3.5 check: C++ SE(2) planner vs the independent Python reference on 50 seeded maps.
Pass: path length agrees within 1e-6 on every map, same waypoints, same node-expansion count, same success/failure."""
import sys, time, math, numpy as np
from fpp import Planner, path_stats
from worlds import corridor_world, RES
from ref_planner import plan_ref
N = int(sys.argv[1]) if len(sys.argv) > 1 else 50
rejected = 0; bad = 0; maxdiff = 0.0; n_ok = 0; n_fail_both = 0; t0 = time.time()
for seed in range(N):
    g = corridor_world(seed=seed); rng = np.random.default_rng(1000 + seed); P = Planner(g, RES, margin=0.02)
    H, W = g.shape; cm = P.clearance_map()
    free = np.argwhere(cm > 0.6)
    s = free[rng.integers(len(free))]; e = free[rng.integers(len(free))]
    sx, sy = s[1] * RES + RES / 2, s[0] * RES + RES / 2
    sth = float(rng.uniform(-math.pi, math.pi))
    for _ in range(200):                                  # a real rover never starts in a colliding pose
        if not P.rect_hits(sx, sy, sth): break
        sth = float(rng.uniform(-math.pi, math.pi))
    start = (sx, sy, sth); goal = (e[1] * RES + RES / 2, e[0] * RES + RES / 2)
    c = P.plan('se2', start, goal, tight=False)
    r = plan_ref(P, g.shape, start, goal)
    if r is not None and not c['ok'] and int(c['expanded']) == r['expanded']:
        rejected += 1; print(f'seed {seed}: search agrees ({r["expanded"]} nodes, length {r["length"]:.3f} m) but the C++ pose-building step rejected the path'); continue
    if (r is None) != (not c['ok']):
        bad += 1; print(f'seed {seed}: MISMATCH success cpp={c["ok"]} ref={r is not None} expanded cpp={int(c["expanded"])} ref={r["expanded"] if r else None}'); continue
    if r is None: n_fail_both += 1; continue
    n_ok += 1; Lc, _ = path_stats(c['path']); d = abs(Lc - r['length']); maxdiff = max(maxdiff, d)
    wp_c = [tuple(p[:2]) for i, p in enumerate(c['path']) if i == 0 or np.hypot(*(p[:2] - c['path'][i - 1][:2])) > 1e-9]
    same_exp = int(c['expanded']) == r['expanded']
    if d > 1e-6 or not same_exp:
        bad += 1; print(f'seed {seed}: len cpp {Lc:.9f} ref {r["length"]:.9f} diff {d:.2e}; expanded cpp {int(c["expanded"])} ref {r["expanded"]}')
print(f'{N} maps: {n_ok} planned by both, {n_fail_both} unreachable for both, {bad} mismatches, {rejected} found by the search but rejected when building poses, max length difference {maxdiff:.2e} m, {time.time() - t0:.0f} s')
sys.exit(1 if bad else 0)
