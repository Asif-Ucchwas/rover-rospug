"""Dynamic layer (task 4.x): lidar people detection, tracking, speed governor, replanning, and a 2D kinematic simulator.
Everything the robot 'knows' comes from lidar hits and the static map. The simulator's pedestrians are never read directly by the governor."""
import numpy as np, math
from fpp import Planner, ROBOT

PROFILES = {   # a, b = half length / half width of the footprint (m). speeds and accelerations are parameters until measured on hardware.
    'rover': dict(a=0.56, b=0.405, v_max=0.8, w_max=0.8, a_acc=0.6, a_brake=1.0, a_emerg=2.0, t_react=0.25),
    'dog':   dict(a=0.23, b=0.155, v_max=0.20, w_max=0.8, a_acc=0.4, a_brake=0.6, a_emerg=1.2, t_react=0.25)}
PED_R = 0.25; DT = 0.05

def rect_circle_dist(x, y, th, a, b, cx, cy):
    """distance from circle centre to the rectangle (0 if inside)."""
    c, s = math.cos(th), math.sin(th); dx, dy = cx - x, cy - y
    u, v = dx * c + dy * s, -dx * s + dy * c
    return math.hypot(max(abs(u) - a, 0), max(abs(v) - b, 0))

# ---------------- lidar ----------------
class Lidar:
    def __init__(s, occ, res, n=720, rng=12.0, r0=0.72, noise=0.02, seed=0):
        s.occ, s.res, s.H, s.W = occ, res, *occ.shape; s.n, s.rng, s.noise = n, rng, noise
        s.ang = np.linspace(-np.pi, np.pi, n, endpoint=False); s.r = np.arange(r0, rng, 0.05); s.rs = np.random.default_rng(seed)
    def scan(s, x, y, th, peds):
        A = s.ang + th; X = x + np.outer(np.cos(A), s.r); Y = y + np.outer(np.sin(A), s.r)
        i = np.floor(X / s.res).astype(int); j = np.floor(Y / s.res).astype(int)
        inb = (i >= 0) & (j >= 0) & (i < s.W) & (j < s.H); hit = ~inb
        hit[inb] = s.occ[j[inb], i[inb]] > 0
        first = np.where(hit.any(1), hit.argmax(1), len(s.r) - 1); rr = s.r[first].copy(); rr[~hit.any(1)] = s.rng
        for (px, py) in peds:   # ray / disc intersection
            dx, dy = px - x, py - y; proj = dx * np.cos(A) + dy * np.sin(A); perp2 = dx * dx + dy * dy - proj * proj
            ok = (proj > 0) & (perp2 < PED_R ** 2); t = proj - np.sqrt(np.maximum(PED_R ** 2 - perp2, 0)); ok &= (t > 0.3) & (t < rr); rr[ok] = t[ok]
        rr = rr + s.rs.normal(0, s.noise, s.n); return rr, A

# ---------------- perception ----------------
class Tracker:
    """Dynamic points = lidar hits that the static map does not explain. Cluster by range gaps, associate, estimate velocity."""
    def __init__(s, occ, res, rng=12.0):
        s.occ, s.res, s.rng = occ, res, rng; s.tracks = []; s.nid = 0
        s.occd = occ.copy()
        for dj in (-2, -1, 0, 1, 2):
            for di in (-2, -1, 0, 1, 2): s.occd |= np.roll(np.roll(occ, dj, 0), di, 1)
    def update(s, x, y, rr, A, dt):
        H, W = s.occ.shape; ok = rr < s.rng - 0.3; px = x + rr * np.cos(A); py = y + rr * np.sin(A)
        i = np.floor(px / s.res).astype(int); j = np.floor(py / s.res).astype(int); inb = (i >= 0) & (j >= 0) & (i < W) & (j < H)
        dyn = ok & inb & (s.occd[np.clip(j, 0, H - 1), np.clip(i, 0, W - 1)] == 0)
        idx = np.where(dyn)[0]; clusters = []
        if len(idx):
            cur = [idx[0]]
            for k in idx[1:]:
                if k == cur[-1] + 1 and math.hypot(px[k] - px[cur[-1]], py[k] - py[cur[-1]]) < 0.25: cur.append(k)
                else: clusters.append(cur); cur = [k]
            clusters.append(cur)
        dets = []
        for c in clusters:
            if len(c) < 3: continue
            cx, cy = px[c].mean(), py[c].mean(); br = math.atan2(cy - y, cx - x); cx += 0.18 * math.cos(br); cy += 0.18 * math.sin(br)   # visible surface -> body centre
            ext = math.hypot(np.ptp(px[c]), np.ptp(py[c])); dets.append((cx, cy, max(PED_R, 0.5 * ext + 0.05)))
        used = set()
        for t in s.tracks:
            t['miss'] += 1; best, bd = None, 0.9
            for k, d in enumerate(dets):
                if k in used: continue
                dist = math.hypot(d[0] - (t['x'] + t['vx'] * dt), d[1] - (t['y'] + t['vy'] * dt))
                if dist < bd: best, bd = k, dist
            if best is not None:
                used.add(best); d = dets[best]; vx, vy = (d[0] - t['x']) / dt, (d[1] - t['y']) / dt
                al = 0.15; t['vx'] = (1 - al) * t['vx'] + al * vx; t['vy'] = (1 - al) * t['vy'] + al * vy
                t['x'], t['y'], t['r'], t['age'], t['miss'] = d[0], d[1], d[2], t['age'] + 1, 0
        for k, d in enumerate(dets):
            if k not in used: s.tracks.append(dict(id=s.nid, x=d[0], y=d[1], vx=0.0, vy=0.0, r=d[2], age=1, miss=0)); s.nid += 1
        s.tracks = [t for t in s.tracks if t['miss'] <= 10]
        return [t for t in s.tracks if t['miss'] <= 2 and t['age'] >= 2]

# ---------------- path -> segments ----------------
def segments(path):
    segs = []; P = np.asarray(path)
    for k in range(len(P) - 1):
        a, b = P[k], P[k + 1]
        if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-9:
            d = (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi
            if segs and segs[-1]['t'] == 'rot' and segs[-1]['d'] * d > 0: segs[-1]['d'] += d; segs[-1]['th1'] = b[2]
            elif abs(d) > 1e-9: segs.append(dict(t='rot', x=a[0], y=a[1], th0=a[2], th1=b[2], d=d))
        else:
            dirv = math.atan2(b[1] - a[1], b[0] - a[0]); sign = 1 if math.cos(a[2] - dirv) > 0 else -1
            if segs and segs[-1]['t'] == 'line' and abs(segs[-1]['dir'] - dirv) < 1e-6 and abs(segs[-1]['th'] - a[2]) < 1e-9: segs[-1]['x1'], segs[-1]['y1'] = b[0], b[1]
            else: segs.append(dict(t='line', x0=a[0], y0=a[1], x1=b[0], y1=b[1], th=a[2], dir=dirv, sign=sign))
    return segs

# ---------------- governor ----------------
class Governor:
    def __init__(s, prof, margin=0.10, keepout=0.30, buf=0.25):
        s.p = prof; s.margin, s.keepout, s.buf = margin, keepout, buf; s.ko = keepout; s.creep = 0.25; s.keepout_min = 0.12
    def obj_radius(s, t, tt):
        r = t['r'] + s.ko
        if t['age'] < 4: r += min(1.0, 0.5 * tt)     # velocity not known yet: grow the uncertainty instead
        return r
    def predict(s, t, tt):
        if t['age'] < 4: return t['x'], t['y']
        return t['x'] + t['vx'] * tt, t['y'] + t['vy'] * tt
    def allowed_speed(s, segs, k, x, y, th, v, tracks):
        """(speed limit, free distance). Comfortable keep-out (0.30 m) at normal speed; when that is not free the robot may still creep (0.25 m/s) past people as long as a 0.12 m keep-out is free."""
        s.ko = s.keepout; d1 = s.free_distance(segs, k, x, y, th, v, tracks); lim = s.speed_limit(d1)
        if d1 != float('inf'):
            s.ko = s.keepout_min; d2 = s.free_distance(segs, k, x, y, th, v, tracks); s.ko = s.keepout
            lim = max(lim, min(s.creep, s.speed_limit(d2)))
            if lim > 0 and d1 < s.buf + 0.05: d1 = s.buf + 0.06
        return lim, d1
    def free_distance(s, segs, k, x, y, th, v, tracks, look=7.0):
        """travel distance along the remaining path before the footprint (+margin) would meet a predicted person. inf if clear."""
        a, b = s.p['a'] + s.margin, s.p['b'] + s.margin; vref = max(v, 0.4); acc = 0.0; tt = 0.0
        def hit(px, py, pth, tt):
            for t in tracks:
                cx, cy = s.predict(t, tt)
                if rect_circle_dist(px, py, pth, a, b, cx, cy) < s.obj_radius(t, tt): return True
            return False
        if hit(x, y, th, 0.0): return 0.0
        for q in range(k, len(segs)):
            sg = segs[q]
            if sg['t'] == 'rot':
                n = max(1, int(abs(sg['d']) / 0.087)); th0 = th if q == k else sg['th0']
                rem = sg['d'] if q > k else (sg['th1'] - th + math.pi) % (2 * math.pi) - math.pi
                for m in range(1, n + 1):
                    if hit(sg['x'], sg['y'], th + rem * m / n if q == k else sg['th0'] + sg['d'] * m / n, tt + abs(rem) * m / n / s.p['w_max']): return acc
                tt += abs(rem) / s.p['w_max']; th = sg['th1']
            else:
                x0, y0 = (x, y) if q == k else (sg['x0'], sg['y0']); L = math.hypot(sg['x1'] - x0, sg['y1'] - y0); n = max(1, int(L / 0.1))
                for m in range(1, n + 1):
                    px, py = x0 + (sg['x1'] - x0) * m / n, y0 + (sg['y1'] - y0) * m / n
                    if hit(px, py, sg['th'], tt + (acc + L * m / n) / vref): return acc + L * (m - 1) / n
                acc += L
            if acc > look: break
        return float('inf')
    def speed_limit(s, d_free):
        """highest speed from which we can still stop with `buf` to spare, given reaction time and braking decel."""
        if d_free == float('inf'): return s.p['v_max']
        d = d_free - s.buf
        if d <= 0: return 0.0
        a, tr = s.p['a_brake'], s.p['t_react']
        return min(s.p['v_max'], a * (-tr + math.sqrt(tr * tr + 2 * d / a)))
    def proximity_limit(s, x, y, th, tracks):
        dmin = min([rect_circle_dist(x, y, th, s.p['a'], s.p['b'], t['x'], t['y']) - t['r'] for t in tracks] or [9.0])
        return s.p['v_max'] * (0.3 + 0.7 * min(1.0, max(0.0, (dmin - 0.4) / 1.6)))

# ---------------- pedestrians (simulation only) ----------------
class Ped:
    """Simulated walker. Does not walk into the robot; if the robot blocks them for 1.5 s they sidestep around it (people do not stand still in a deadlock)."""
    clr = None   # optional function (x, y) -> distance to nearest wall
    def __init__(s, route, speed, delay=0.0): s.route, s.speed, s.i, s.delay = [np.array(p, float) for p in route], speed, 0, delay; s.pos = s.route[0].copy(); s.wait = 0.0
    def step(s, dt, robot_xy, rob_a, rob=None):
        if s.delay > 0: s.delay -= dt; return
        if s.i >= len(s.route) - 1: return
        tgt = s.route[s.i + 1]; d = tgt - s.pos; L = np.linalg.norm(d)
        if L < 0.05: s.i += 1; return
        step = d / L * min(s.speed * dt, L); nxt = s.pos + step
        dn, dp = np.hypot(*(robot_xy - nxt)), np.hypot(*(robot_xy - s.pos))
        if rob is not None: dn = rect_circle_dist(rob[0], rob[1], rob[2], rob[3], rob[4], nxt[0], nxt[1]) + rob_a * 0; dp = rect_circle_dist(rob[0], rob[1], rob[2], rob[3], rob[4], s.pos[0], s.pos[1]); lim = PED_R + 0.12
        else: lim = rob_a + PED_R + 0.15
        if dn < lim and dn < dp:
            s.wait += dt
            if s.wait > 1.5:      # sidestep: move perpendicular to the line to the robot, away from walls
                u = (s.pos - robot_xy) / max(dp, 1e-6); perp = np.array([-u[1], u[0]])
                for sgn in (1, -1):
                    cand = s.pos + sgn * perp * 0.5 * dt
                    if s.clr is None or s.clr(*cand) > PED_R + 0.1: s.pos = cand; break
            return
        s.wait = 0.0; s.pos = nxt

# ---------------- episode runner ----------------
def paint(occ, res, tracks, extra):
    o = occ.copy(); H, W = o.shape
    for t in tracks:
        R = t['r'] + extra; i0, i1 = int((t['x'] - R) / res), int((t['x'] + R) / res) + 1; j0, j1 = int((t['y'] - R) / res), int((t['y'] + R) / res) + 1
        for j in range(max(j0, 0), min(j1, H)):
            for i in range(max(i0, 0), min(i1, W)):
                if math.hypot((i + .5) * res - t['x'], (j + .5) * res - t['y']) <= R: o[j, i] = 1
    return o

def run_episode(occ, res, path, goal, peds, prof='rover', t_max=150.0, seed=0, t_wait=4.0, t_giveup=40.0, spawn=None, log=False):
    P = PROFILES[prof]; gov = Governor(P); lid = Lidar(occ, res, seed=seed); trk = Tracker(occ, res)
    truth = Planner(occ, res, margin=0.0); truth.a, truth.b = P['a'], P['b']
    x, y, th = map(float, path[0]); v = 0.0; w = 0.0; segs = segments(path); k = 0; t = 0.0
    out = dict(collisions=0, coll_moving=0, min_gap=9.0, stops=0, replans=0, replan_fail=0, status='timeout', t=0.0, dist=0.0, max_v_near=0.0, emerg=0, trace=[])
    mark_t = 0.0; mark_dist = 0.0; mark_k = 0; was_stopped = False; last_replan = -99.0; px_, py_ = x, y
    while t < t_max:
        if spawn: spawn(t, peds)
        for p in peds: p.step(DT, np.array([x, y]), P['a'], (x, y, th, P['a'], P['b']))
        rr, A = lid.scan(x, y, th, [tuple(p.pos) for p in peds]); tracks = trk.update(x, y, rr, A, DT)
        # truth metrics
        gaps = [rect_circle_dist(x, y, th, P['a'], P['b'], p.pos[0], p.pos[1]) - PED_R for p in peds]
        if gaps:
            g = min(gaps); out['min_gap'] = min(out['min_gap'], g)
            if g < 0:
                out['collisions'] += 1
                if v > 0.05 or w > 0.05: out['coll_moving'] += 1     # robot was moving: its fault; otherwise a simulated walker stepped into a stopped robot
            if g < 1.0: out['max_v_near'] = max(out['max_v_near'], v)
        if truth.rect_hits(x, y, th): out['collisions'] += 1
        if tracks and not peds: out['phantom'] = out.get('phantom', 0) + 1
        if k >= len(segs): out['status'] = 'reached'; break
        sg = segs[k]; vlim, d_free = gov.allowed_speed(segs, k, x, y, th, v, tracks); blocked = vlim <= 0.0
        if sg['t'] == 'line':
            d_end = math.hypot(sg['x1'] - x, sg['y1'] - y); v_end = math.sqrt(2 * P['a_brake'] * 0.8 * max(d_end, 0)) if k == len(segs) - 1 or True else P['v_max']
            vt = min(P['v_max'], vlim, gov.proximity_limit(x, y, th, tracks), max(v_end, 0.05) if d_end > 0.02 else 0.0)
            if v > vt: dec = P['a_emerg'] if v > vlim + 0.05 else P['a_brake']; 
            if v > vt: 
                if v > vlim + 0.05: out['emerg'] += 1; out.setdefault('why', []).append((round(t,2), round(v,2), vlim, d_free))
                v = max(vt, v - dec * DT)
            else: v = min(vt, v + P['a_acc'] * DT)
            step = min(v * DT, d_end); x += step * math.cos(sg['dir']); y += step * math.sin(sg['dir']); out['dist'] += step
            if d_end < 0.02: k += 1; v = 0.0
        else:
            rem = (sg['th1'] - th + math.pi) % (2 * math.pi) - math.pi
            wt = 0.0 if blocked else min(P['w_max'], math.sqrt(2 * 1.5 * abs(rem)) + 0.02)
            w = min(wt, w + 1.5 * DT) if wt >= w else max(wt, w - 3.0 * DT)
            th += math.copysign(min(w * DT, abs(rem)), rem); v = 0.0
            if abs(rem) < 0.01: k += 1; w = 0.0
        moving = v > 0.02 or w > 0.02
        if was_stopped and moving: pass
        if (not moving) and not was_stopped and blocked: out['stops'] += 1
        was_stopped = (not moving) and blocked
        # stuck = people-limited (stopped or creeping) without real progress for a while: wait, then replan around them, then give up and report
        if vlim >= P['v_max'] * 0.5 or out['dist'] - mark_dist >= 0.5 or k != mark_k: mark_t, mark_dist, mark_k = t, out['dist'], k
        stall = t - mark_t
        if stall > t_giveup:
            out['status'] = 'blocked'; out['at_end'] = dict(robot=(round(x, 2), round(y, 2), round(th, 2)), vlim=round(vlim, 2), d_free=d_free, tracks=[(round(q['x'], 2), round(q['y'], 2), round(q['r'], 2), round(q['vx'], 2), round(q['vy'], 2)) for q in tracks], peds=[tuple(np.round(p.pos, 2)) for p in peds]); break
        if stall > t_wait and t - last_replan > 3.0:
            last_replan = t; gx, gy = goal; r = dict(ok=False)
            for extra in (gov.keepout - gov.margin + 0.02, gov.keepout_min - gov.margin + 0.02 + 0.06):    # comfortable detour first, tighter ones if that does not exist
                o2 = paint(occ, res, tracks, extra); pl = Planner(o2, res, margin=0.02); pl.a, pl.b = P['a'], P['b']
                r = pl.plan('se2', (x, y, th), (gx, gy), tight=True)
                if r['ok'] and pl.validate(r['path']) == 0:
                    gov.ko = gov.keepout_min; df = gov.free_distance(segments(r['path']), 0, x, y, th, 0.0, tracks, look=60.0); gov.ko = gov.keepout   # the governor must accept the detour too
                    if df == float('inf'): break
                r = dict(ok=False)
            if r['ok']: segs = segments(r['path']); k = 0; out['replans'] += 1; t += r['secs']; mark_t, mark_dist, mark_k = t, out['dist'], 0
            else: out['replan_fail'] += 1
        t += DT
        if log and int(t / DT) % 20 == 0: out['trace'].append((round(t, 2), round(x, 2), round(y, 2), round(v, 2), len(tracks), round(d_free, 2) if d_free != float('inf') else -1))
    out['t'] = t; return out
