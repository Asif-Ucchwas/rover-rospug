import sys, numpy as np, json; sys.path.insert(0, 'py')
from fpp import Planner, path_stats
from worlds import gap_world
gaps = np.round(np.arange(0.70, 1.55, 0.05), 2); methods = ['point_circ', 'point_insc', 'theta_rect', 'se2']
res = {}
for gp in gaps:
    pl = Planner(gap_world(gp), 0.05)
    for m in methods:
        r = pl.plan(m, (3.0, 7.0, 0.0), (17.0, 3.0), tight=True)
        if r['ok']:
            bad = pl.validate(r['path']); L, rot = path_stats(r['path']); res[(gp, m)] = (True, bad == 0, L, rot, r['secs'], r['expanded'])
        else: res[(gp, m)] = (False, False, None, None, r['secs'], r['expanded'])
json.dump({f'{k[0]}|{k[1]}':v for k,v in res.items()}, open('results_gap.json','w'))
print('gap   ' + ' '.join(f'{m:>22s}' for m in methods))
for gp in gaps:
    row = []
    for m in methods:
        ok, val, L, rot, t, ex = res[(gp, m)]
        row.append(f"{'--' if not ok else ('VALID' if val else 'COLLIDES')} {'' if L is None else f'{L:5.1f}m'} {t*1000:6.0f}ms".rjust(22))
    print(f'{gp:4.2f}  ' + ' '.join(row))
