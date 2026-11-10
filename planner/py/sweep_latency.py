import numpy as np, sys
from fpp import DEFAULT_PRM
from bench_latency import run
base = run(DEFAULT_PRM, repeats=1, verbose=False)
cfgs = {'base 5cm/32/0.2': (0.05, 32, 0.2), '10cm/16/0.2': (0.10, 16, 0.2), '5cm/32/0.4': (0.05, 32, 0.4), '10cm/16/0.4': (0.10, 16, 0.4), '7.5cm/24/0.3': (0.075, 24, 0.3), '10cm/32/0.3': (0.10, 32, 0.3), '5cm/16/0.3': (0.05, 16, 0.3)}
print(f'{"config":18s} found  coll  med(found) p95(found)  max(nopath)  len vs base (median, worst)  newly lost')
for name, (rs, nt, st) in cfgs.items():
    prm = list(DEFAULT_PRM); prm[0], prm[1], prm[2] = rs, nt, st
    r = run(prm, repeats=1, verbose=False); f = r['oks']; both = f & base['oks']
    ratio = r['L'][both] / base['L'][both]
    print(f'{name:18s} {f.sum():3d}/80 {r["bad"]:4d}  {np.median(r["ts"][f]):8.1f}  {np.percentile(r["ts"][f],95):8.1f}  {r["ts"][~f].max() if (~f).any() else 0:10.0f}   {np.median(ratio):.3f}, {ratio.max():.3f}   {int((base["oks"] & ~f).sum())}', flush=True)
