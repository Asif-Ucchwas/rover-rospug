"""Task 3.3 check: swept-rectangle collision vs brute-force dense sampling, 10,000 random queries."""
import numpy as np, sys, time; sys.path.insert(0, '.'); sys.path.insert(0, 'py')
from fpp import Planner, ROBOT
DENS = float(sys.argv[1]) if len(sys.argv)>1 else 0.12
rng = np.random.default_rng(1); res = 0.05; H = W = 200
occ = (rng.random((H, W)) < DENS).astype(np.uint8); occ[:3] = 1; occ[-3:] = 1; occ[:, :3] = 1; occ[:, -3:] = 1
pl = Planner(occ, res)
a, b = ROBOT['a'], ROBOT['b']
# brute force: dense points inside rectangle (1 cm interior grid + 1 mm boundary); a point hits a cell if it lies strictly inside it
u = np.arange(-a, a + 1e-9, 0.01); v = np.arange(-b, b + 1e-9, 0.01); U, V = np.meshgrid(u, v); U = U.ravel(); V = V.ravel()
bu = np.arange(-a, a + 1e-9, 0.002); bv = np.arange(-b, b + 1e-9, 0.002)
BU = np.concatenate([bu, bu, np.full_like(bv, -a), np.full_like(bv, a)]); BV = np.concatenate([np.full_like(bu, -b), np.full_like(bu, b), bv, bv])
U = np.concatenate([U, BU]); V = np.concatenate([V, BV])
def brute(x, y, th):
    c, s = np.cos(th), np.sin(th); X = x + U * c - V * s; Y = y + U * s + V * c
    i = np.floor(X / res).astype(int); j = np.floor(Y / res).astype(int)
    if (i < 0).any() or (j < 0).any() or (i >= W).any() or (j >= H).any(): return True
    return bool(occ[j, i].any())
def brute_sweep(x0, y0, x1, y1, th):
    L = np.hypot(x1 - x0, y1 - y0); n = max(2, int(L / 0.01) + 1)
    return any(brute(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, th) for t in np.linspace(0, 1, n))
N = 10000; miss = fa = agree = 0; nfree = 0; t0 = time.time()
for q in range(N):
    x, y = rng.uniform(1.5, 8.5, 2); th = rng.uniform(-np.pi, np.pi)
    e, bf = pl.rect_hits(x, y, th), brute(x, y, th); nfree += (not bf)
    if e == bf: agree += 1
    elif bf and not e: miss += 1          # brute says hit, exact says free: a MISSED collision
    else: fa += 1                         # exact says hit, brute says free: sub-millimetre contact
print(f'pose queries {N} ({nfree} collision-free): agree {agree}, missed collisions {miss}, exact-only hits {fa}  ({time.time()-t0:.0f}s)')
Ns = 1500; miss = fa = agree = 0
for q in range(Ns):
    x0, y0 = rng.uniform(1.5, 8.5, 2); ang = rng.uniform(-np.pi, np.pi); L = rng.uniform(0.2, 2.0); x1, y1 = x0 + L * np.cos(ang), y0 + L * np.sin(ang)
    if not (1.2 < x1 < 8.8 and 1.2 < y1 < 8.8): continue
    th = rng.uniform(-np.pi, np.pi); e, bf = pl.sweep_hits(x0, y0, x1, y1, th), brute_sweep(x0, y0, x1, y1, th)
    if e == bf: agree += 1
    elif bf and not e: miss += 1
    else: fa += 1
print(f'sweep queries: agree {agree}, missed collisions {miss}, exact-only hits {fa}')
