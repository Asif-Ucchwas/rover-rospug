import sys, numpy as np, json, time; sys.path.insert(0, 'py')
from fpp import Planner, path_stats, ROBOT, DEFAULT_PRM
from worlds import corridor_world, RES
g = corridor_world(seed=0); pl = Planner(g, RES, margin=0.02); vpl = Planner(g, RES, margin=0.0)   # plan with a 2 cm margin, validate on the true footprint
rng = np.random.default_rng(7); clr = pl.clearance_map()
# queries: free poses (collision-free at some heading) with at least 0.45 m clearance, spread over rooms and corridor
free = np.argwhere(clr > ROBOT['b'] + 0.06); pairs = []
while len(pairs) < 80:
    a, b = free[rng.integers(len(free))], free[rng.integers(len(free))]
    s = ((a[1] + .5) * RES, (a[0] + .5) * RES); t = ((b[1] + .5) * RES, (b[0] + .5) * RES)
    if np.hypot(s[0] - t[0], s[1] - t[1]) > 8:
        hs = [h for h in np.linspace(-np.pi, np.pi, 72, endpoint=False) if not pl.rect_hits(s[0], s[1], h)]
        if hs: pairs.append((s, t, float(hs[rng.integers(len(hs))])))
methods = [('point_circ', True), ('point_insc', True), ('theta_rect', True), ('theta_rect_raw', False), ('se2', True), ('se2_raw', False), ('se2_fine', True)]
rows = []
FINE = list(DEFAULT_PRM); FINE[1] = 72; FINE[2] = 0.10; FINE[7] = 60.0
for qi, (s, t, h0) in enumerate(pairs):
    for name, tight in methods:
        m = name.replace('_raw', '').replace('_fine', '')
        r = pl.plan(m, (s[0], s[1], h0), t, tight=tight, prm=FINE if name == 'se2_fine' else None)
        d = dict(q=qi, method=name, ok=bool(r['ok']), secs=r['secs'], expanded=r['expanded'])
        if r['ok']:
            L, rot = path_stats(r['path']); d.update(length=L, rot=rot, bad=vpl.validate(r['path']), minclr=float(min(pl.clearance(x, y) for x, y, _ in r['path'][::5])))
        rows.append(d)
json.dump(dict(pairs=pairs, rows=rows), open('results_corridor.json', 'w'))
import collections
print(f'{len(pairs)} queries, corridor world, margin 2 cm')
feas = {r['q'] for r in rows if r['ok'] and r['bad'] == 0}
print(f'{len(feas)} of {len(pairs)} queries have at least one collision-free path (reference set)')
for name, _ in methods:
    R = [r for r in rows if r['method'] == name and r['q'] in feas]; ok = [r for r in R if r['ok']]; val = [r for r in ok if r['bad'] == 0]
    print(f"{name:15s} planned {len(ok):3d}/{len(R)}  valid {len(val):3d}  median time {np.median([r['secs'] for r in ok])*1000 if ok else float('nan'):7.1f} ms  median len {np.median([r['length'] for r in val]) if val else float('nan'):6.2f} m")
