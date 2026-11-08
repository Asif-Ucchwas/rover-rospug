"""C++ vs Python: governor on random scenes, tracker + governor on full simulated drives. Differences above 1e-9 are failures."""
import sys, json, numpy as np; sys.path.insert(0, 'py')
import dyn, dyn_cpp
from fpp import Planner
from worlds import corridor_world, RES
rng = np.random.default_rng(5); P = dyn.PROFILES['rover']; pg, cg = dyn.Governor(P), dyn_cpp.Governor(P)
worst = 0.0; n = 0; inf_mismatch = 0
for trial in range(3000):
    segs = []; x, y, th = rng.uniform(0, 5, 2).tolist() + [rng.uniform(-3, 3)]; cx, cy, cth = x, y, th
    for _ in range(rng.integers(1, 5)):
        if rng.random() < 0.4: d = rng.uniform(-3, 3); segs.append(dict(t='rot', x=cx, y=cy, th0=cth, th1=dyn.math.atan2(dyn.math.sin(cth + d), dyn.math.cos(cth + d)), d=d)); cth = segs[-1]['th1']
        else: L = rng.uniform(0.5, 6); dr = rng.uniform(-3, 3); nx, ny = cx + L * np.cos(dr), cy + L * np.sin(dr); segs.append(dict(t='line', x0=cx, y0=cy, x1=nx, y1=ny, th=cth, dir=dr, sign=1)); cx, cy = nx, ny
    tr = [dict(x=rng.uniform(-1, 9), y=rng.uniform(-1, 9), vx=rng.normal(0, 1), vy=rng.normal(0, 1), r=rng.uniform(0.25, 0.4), age=int(rng.integers(1, 12)), miss=0) for _ in range(rng.integers(0, 4))]
    v = rng.uniform(0, 0.8); k = 0
    a = pg.allowed_speed(segs, k, x, y, th, v, tr); b = cg.allowed_speed(segs, k, x, y, th, v, tr); n += 1
    for u, w in zip(a, b):
        if np.isinf(u) != np.isinf(w): inf_mismatch += 1
        elif not np.isinf(u): worst = max(worst, abs(u - w))
    worst = max(worst, abs(pg.proximity_limit(x, y, th, tr) - cg.proximity_limit(x, y, th, tr)))
print(f'governor: {n} random scenes, worst difference {worst:.2e}, infinity mismatches {inf_mismatch}')
assert worst < 1e-9 and inf_mismatch == 0
# whole drives: C++ tracker + governor against the Python ones, same walkers
g = corridor_world(seed=0); pl = Planner(g, RES, margin=0.02); pp = Planner(g, RES, margin=0.0)
pairs = json.load(open('results_corridor.json'))['pairs'][:6]; clr = pp.clearance_map(); free = np.argwhere(clr > 0.5)
def drive(impl, qi):
    s, t, h = pairs[qi]; r = pl.plan('se2', (s[0], s[1], h), t, tight=True)
    if not r['ok']: return None
    rg = np.random.default_rng(100 + qi); peds = []
    for _ in range(3):
        a, b = free[rg.integers(len(free))], free[rg.integers(len(free))]
        A_ = ((a[1] + .5) * RES, (a[0] + .5) * RES); B_ = ((b[1] + .5) * RES, (b[0] + .5) * RES); q = pp.plan('point_insc', (A_[0], A_[1], 0), B_)
        if q['ok']: peds.append(dyn.Ped(q['path'][::6, :2], rg.uniform(0.9, 1.5), delay=rg.uniform(0, 6)))
    return dyn.run_episode(g, RES, r['path'], t, peds, seed=qi, impl=impl, t_max=60)
for qi in range(len(pairs)):
    a = drive('py', qi); b = drive('cpp', qi)
    if a is None: continue
    same = (a['status'], round(a['t'], 2), a['collisions'], a['stops']) == (b['status'], round(b['t'], 2), b['collisions'], b['stops'])
    print(f"drive {qi}: python {a['status']} {a['t']:.2f}s coll {a['collisions']} stops {a['stops']} | c++ {b['status']} {b['t']:.2f}s coll {b['collisions']} stops {b['stops']}  {'SAME' if same else 'DIFFERENT'}")
    assert same
print('C++ and Python agree')
