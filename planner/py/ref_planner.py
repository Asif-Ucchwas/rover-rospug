"""Independent pure-Python reference for the SE(2) lattice planner (task 3.5).
It re-implements the SEARCH (heuristic, lattice moves, costs, tie-breaking, dominance table, goal test, waypoint extraction)
from the spec, line by line, in Python. For collision it calls the primitives that task 3.3 already validated against brute force
(rect_hits, sweep_hits). So agreement with the C++ planner checks the C++ search code, not the collision code a second time."""
import math, heapq, numpy as np
from fpp import Planner, DEFAULT_PRM

PI = math.pi; CHK = 0.5 * PI / 180.0; F32 = np.float32
def wrap(a):
    while a > PI: a -= 2 * PI
    while a <= -PI: a += 2 * PI
    return a
def lround(x): return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)

def rot_hits(P, x, y, th0, th1, direction):
    d = th1 - th0
    if abs(wrap(d)) < 1e-9: d = 0
    elif direction > 0:
        while d < 0: d += 2 * PI
        while d > 2 * PI: d -= 2 * PI
    else:
        while d > 0: d -= 2 * PI
        while d < -2 * PI: d += 2 * PI
    if P.clearance(x, y) >= math.hypot(P.a, P.b) + P.margin + P.res: return False
    n = max(1, math.ceil(abs(d) / CHK))
    for k in range(n + 1):
        if P.rect_hits(x, y, th0 + d * k / n): return True
    return False

def plan_ref(P, occ_shape, start, goal, prm=None):
    prm = DEFAULT_PRM if prm is None else prm
    rs, NT, step, wrot, revp, gtol = prm[0], int(prm[1]), prm[2], prm[3], prm[4], prm[5]
    res = P.res; H, W = occ_shape; b, margin = P.b, P.margin
    cm = P.clearance_map()                                   # float32 (H, W): distance to nearest obstacle surface
    thr = b + margin - 0.5 * res - 1e-9
    ok = lambda i, j: 0 <= i < W and 0 <= j < H and float(cm[j, i]) >= thr
    sx, sy, sth = start; gx, gy = goal
    INF = 1e30; hf = {}
    gi, gj = int(gx / res), int(gy / res)
    if not ok(gi, gj): return None
    hf[(gi, gj)] = F32(0); pq = [(F32(0), gj * W + gi)]
    while pq:
        d, k = heapq.heappop(pq); i, j = k % W, k // W
        if d > hf[(i, j)]: continue
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                if not di and not dj: continue
                ni, nj = i + di, j + dj
                if not ok(ni, nj): continue
                nd = F32(d + F32(res * (math.sqrt(2.0) if (di and dj) else 1.0)))
                if nd < hf.get((ni, nj), F32(INF)): hf[(ni, nj)] = nd; heapq.heappush(pq, (nd, nj * W + ni))
    def H0(x, y):
        i, j = int(x / res), int(y / res)
        if i < 0 or j < 0 or i >= W or j >= H: return F32(INF)
        return hf.get((i, j), F32(INF))
    W2, H2 = math.ceil(W * res / rs), math.ceil(H * res / rs)
    def key(x, y, it):
        i, j = int(x / rs), int(y / rs)
        if i < 0 or j < 0 or i >= W2 or j >= H2: return None
        return (j * W2 + i) * NT + it
    dth = 2 * PI / NT
    def binof(th):
        it = lround(wrap(th) / dth) % NT; return it
    if H0(sx, sy) > F32(1e29): return None
    it0 = binof(sth); th0 = it0 * dth
    if P.rect_hits(sx, sy, th0):
        found = False; base = it0
        for dq in range(1, NT // 2 + 1):
            for sg in (1, -1):
                q = (base + sg * dq) % NT
                if not P.rect_hits(sx, sy, q * dth): it0, th0, found = q, q * dth, True; break
            if found: break
        if not found: return None
    nodes = [(sx, sy, th0, F32(0), -1, it0)]; closed = [False]; best = {key(sx, sy, it0): 0}
    pq = [(F32(H0(sx, sy)), 0)]; goal_node = -1; expanded = 0
    while pq:
        f, k = heapq.heappop(pq)
        if closed[k]: continue
        closed[k] = True; expanded += 1
        cx, cy, cth, cg, _, cit = nodes[k]
        if math.hypot(cx - gx, cy - gy) <= gtol: goal_node = k; break
        ca, sa = math.cos(cth), math.sin(cth)
        sstep = min(step, math.hypot(cx - gx, cy - gy))
        if sstep < 0.02: sstep = min(step, 0.02)
        acts = [(cx + sstep * ca, cy + sstep * sa, cth, sstep, False, cit),
                (cx - sstep * ca, cy - sstep * sa, cth, sstep * revp, False, cit),
                (cx, cy, wrap(cth + dth), wrot * dth, True, (cit + 1) % NT),
                (cx, cy, wrap(cth - dth), wrot * dth, True, (cit + NT - 1) % NT)]
        for q, (ax, ay, ath, c, rot, ait) in enumerate(acts):
            if rot:
                if rot_hits(P, cx, cy, cth, ath, 1 if q == 2 else -1): continue
            elif P.sweep_hits(cx, cy, ax, ay, cth): continue
            kk = key(ax, ay, ait)
            if kk is None: continue
            ng = F32(cg + F32(c)); bi = best.get(kk, -1)
            if bi >= 0 and nodes[bi][3] <= ng: continue
            nodes.append((ax, ay, ath, ng, k, ait)); closed.append(False); idn = len(nodes) - 1; best[kk] = idn
            h = H0(ax, ay)
            if h > F32(1e29): continue
            heapq.heappush(pq, (F32(ng + h), idn))
    if goal_node < 0: return None
    chain = []; k = goal_node
    while k >= 0: chain.append(k); k = nodes[k][4]
    chain.reverse(); wp = [(sx, sy)]
    for q in range(1, len(chain) - 1):
        a, bb, c = nodes[chain[q - 1]], nodes[chain[q]], nodes[chain[q + 1]]
        mv1 = math.hypot(bb[0] - a[0], bb[1] - a[1]) > 1e-9; mv2 = math.hypot(c[0] - bb[0], c[1] - bb[1]) > 1e-9
        if mv1 and mv2:
            a1 = math.atan2(bb[1] - a[1], bb[0] - a[0]); a2 = math.atan2(c[1] - bb[1], c[0] - bb[0])
            if abs(wrap(a2 - a1)) > 1e-6: wp.append((bb[0], bb[1]))
        elif mv1 != mv2: wp.append((bb[0], bb[1]))
    wp.append((nodes[chain[-1]][0], nodes[chain[-1]][1]))
    dd = []
    for p in wp:
        if not dd or math.hypot(p[0] - dd[-1][0], p[1] - dd[-1][1]) > 1e-9: dd.append(p)
    return dict(wp=dd, expanded=expanded, length=sum(math.hypot(dd[i + 1][0] - dd[i][0], dd[i + 1][1] - dd[i][1]) for i in range(len(dd) - 1)))
