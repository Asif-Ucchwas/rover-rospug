"""Task 3.9: SE(2) latency on the 80 corridor queries. Saves paths so optimisations can be checked for identical output."""
import sys, json, time, hashlib, numpy as np; sys.path.insert(0, 'py')
from fpp import Planner
from worlds import corridor_world, RES
g = corridor_world(seed=0); pl = Planner(g, RES, margin=0.02)
pairs = json.load(open('results_corridor.json'))['pairs']
ts, ex, ok, h = [], [], 0, hashlib.sha1()
for s, t, h0 in pairs:
    best = 1e9
    for _ in range(3):
        t0 = time.perf_counter(); r = pl.plan('se2', (s[0], s[1], h0), t, tight=True); best = min(best, time.perf_counter() - t0)
    ts.append(best); ex.append(r['expanded']); ok += bool(r['ok'])
    if r['ok']: h.update(np.round(np.array(r['path']), 6).tobytes())
ts = np.array(ts) * 1000
print(f'se2: ok {ok}/{len(pairs)}  median {np.median(ts):.1f} ms  p90 {np.percentile(ts,90):.1f} ms  max {ts.max():.0f} ms  median expanded {int(np.median(ex))}  path hash {h.hexdigest()[:12]}')
