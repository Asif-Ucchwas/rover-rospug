"""Decision task 3.2, part 2: does RRT* (plain, corridor, corridor + seed fallback) beat the SE(2) lattice + shortcut under the exact rectangle rule?"""
import sys, json, numpy as np; sys.path.insert(0, 'py')
from fpp import Planner, path_stats, DEFAULT_PRM
from worlds import corridor_world, RES
C = json.load(open('results_corridor.json')); rows = C['rows']; pairs = C['pairs']
feas = sorted({r['q'] for r in rows if r['ok'] and r['bad'] == 0}); base = {r['q']: r for r in rows if r['method'] == 'se2' and r['ok'] and r['bad'] == 0}
g = corridor_world(seed=0); pl = Planner(g, RES, margin=0.02); vpl = Planner(g, RES, margin=0.0)
cfgs = [('rrt', 2000), ('rrt', 4000), ('rrt_corr', 2000), ('rrt_corr', 4000), ('rrt_corr_fb', 2000), ('rrt_corr_fb', 4000)]
out = []
for q in feas:
    s, t, h0 = pairs[q]
    for m, it in cfgs:
        p = list(DEFAULT_PRM); p[9] = it
        r = pl.plan(m, (s[0], s[1], h0), t, tight=True, prm=p); d = dict(q=q, method=m, iters=it, ok=bool(r['ok']), secs=r['secs'], fb=bool(r['fallback']))
        if r['ok']: L, rot = path_stats(r['path']); d.update(length=L, bad=vpl.validate(r['path']))
        out.append(d)
json.dump(out, open('results_rrt.json', 'w'))
print(f'{len(feas)} reachable queries; SE(2)+shortcut solved {len(base)} of them (median length {np.median([r["length"] for r in base.values()]):.2f} m)')
for m, it in cfgs:
    R = [d for d in out if d['method'] == m and d['iters'] == it]; ok = [d for d in R if d['ok'] and d['bad'] == 0]
    both = [d for d in ok if d['q'] in base]; ratio = [d['length'] / base[d['q']]['length'] for d in both] if both else []
    print(f"{m:12s} iters {it:5d}: valid {len(ok):3d}/{len(R)}   fallback used {sum(d['fb'] for d in R):3d}   median time {np.median([d['secs'] for d in R])*1000:6.0f} ms   length vs SE(2): median {np.median(ratio) if ratio else float('nan'):.3f}, shorter in {sum(x < 0.999 for x in ratio)} of {len(ratio)}")
