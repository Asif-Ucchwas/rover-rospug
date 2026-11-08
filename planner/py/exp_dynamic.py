"""Dynamic-obstacle experiments (simulation). A: crowds in the corridor world. B: person steps out of a doorway. C: person standing in the way."""
import sys, json, time, numpy as np; sys.path.insert(0, 'py')
from fpp import Planner, ROBOT
from worlds import corridor_world, RES
import dyn
which = sys.argv[1] if len(sys.argv) > 1 else 'ABC'; res_all = {}
if 'A' in which:
    g = corridor_world(seed=0); pl = Planner(g, RES, margin=0.02); pp = Planner(g, RES, margin=0.0); clr = pp.clearance_map()
    pairs = json.load(open('results_corridor.json'))['pairs']; free = np.argwhere(clr > 0.5); dyn.Ped.clr = staticmethod(lambda x, y: clr[int(y / RES), int(x / RES)]); rng = np.random.default_rng(3); rows = []; used = 0
    for qi, (s, t, h) in enumerate(pairs):
        r = pl.plan('se2', (s[0], s[1], h), t, tight=True)
        if not r['ok']: continue
        used += 1
        if used > 14: break
        for npeds in ((0, 3, 6) if len(sys.argv) < 3 else (int(sys.argv[2]),)):
            peds = []
            for _ in range(npeds):
                pts = [free[rng.integers(len(free))] for _ in range(4)]; route = []
                for a, b in zip(pts[:-1], pts[1:]):   # three legs: people keep walking instead of parking in the corridor
                    A_ = ((a[1] + .5) * RES, (a[0] + .5) * RES); B_ = ((b[1] + .5) * RES, (b[0] + .5) * RES); q = pp.plan('point_insc', (A_[0], A_[1], 0), B_)
                    if q['ok']: route += [tuple(p) for p in q['path'][::6, :2]]
                if len(route) > 2 and np.hypot(route[0][0] - r['path'][0][0], route[0][1] - r['path'][0][1]) > 3.0: peds.append(dyn.Ped(route, rng.uniform(0.9, 1.5), delay=rng.uniform(0, 6)))
            o = dyn.run_episode(g, RES, r['path'], t, peds, seed=qi); o.pop('trace'); o.update(q=qi, npeds=len(peds), speed_note=''); rows.append(o)
            if 'at_end' in o: print('   at end:', o['at_end'])
            print(f"q{qi:2d} peds {len(peds)}  {o['status']:8s} t {o['t']:6.1f}s  coll {o['collisions']} (robot moving: {o['coll_moving']})  min gap {o['min_gap']:.2f}  stops {o['stops']} replans {o['replans']}  emerg {o['emerg']}  phantom {o.get('phantom',0)}", flush=True)
    res_all['A'] = rows
def hall(door_w=1.0, W_=3.0, L=30.0):
    H_ = int((W_ + 2.0) / RES); W__ = int(L / RES); o = np.zeros((H_, W__), np.uint8)
    o[:int(0.3 / RES)] = 1; o[-int(0.3 / RES):] = 1      # outer walls
    y_lo, y_hi = int(0.3 / RES), int((0.3 + W_) / RES)
    o[y_hi - 1:y_hi + 1, :] = 1; o[:y_lo + 1, :] = 1      # corridor walls
    o[y_hi - 1:y_hi + 1, int((15 - door_w / 2) / RES):int((15 + door_w / 2) / RES)] = 0   # doorway at x=15 into the alcove above... (y direction up in grid index)
    return o
if 'B' in which or 'C' in which:
    base = hall(); ymid = 0.3 + 1.5
    path = np.array([[2, ymid, 0], [28, ymid, 0]]); P = dyn.PROFILES
    # fill straight path with intermediate poses (segments() merges them)
    path = np.array([[x, ymid, 0.0] for x in np.arange(2, 28.01, 0.5)])
if 'B' in which:
    rows = []
    for prof in ('rover', 'dog'):
        sp = dyn.PROFILES[prof]['v_max']
        for d in (0.8, 1.2, 1.6, 2.0, 2.5, 3.0, 4.0, 5.0):
            # person starts in the alcove (hidden by the wall), crosses the corridor at 1.4 m/s; timed so the robot is d metres before the door when they step out
            t_emerge = (15 - d - 2) / sp + 0.0
            ped = dyn.Ped([(15.0, 0.3 + 3.0 + 0.9), (15.0, 0.3 + 0.2)], 1.4, delay=max(0.0, t_emerge))
            ped.pos = np.array([15.0, 0.3 + 3.0 + 0.9])
            o = dyn.run_episode(base, RES, path, (28, ymid), [ped], prof=prof, t_max=200); o.pop('trace')
            rows.append(dict(profile=prof, d=d, **o)); print(f"B {prof:5s} appears {d:3.1f} m ahead: {o['status']:8s} coll {o['collisions']} min gap {o['min_gap']:.2f}  emerg {o['emerg']}  replans {o['replans']}/{o['replan_fail']} t {o['t']:.0f}s", flush=True)
    res_all['B'] = rows
if 'C' in which:
    rows = []
    for Wc in (3.0, 4.5):
        hb = hall(W_=Wc); ym = 0.3 + Wc / 2; pth = np.array([[x, ym, 0.0] for x in np.arange(2, 28.01, 0.5)])
        for prof in ('rover', 'dog'):
            ped = dyn.Ped([(15.0, ym), (15.0, ym)], 0.0); ped.pos = np.array([15.0, ym])
            o = dyn.run_episode(hb, RES, pth, (28, ym), [ped], prof=prof, t_max=240); o.pop('trace'); rows.append(dict(profile=prof, corridor=Wc, **o))
            print(f"C {prof:5s} corridor {Wc} m, person standing in the path: {o['status']:8s} t {o['t']:.0f}s coll {o['collisions']} min gap {o['min_gap']:.2f} stops {o['stops']} replans {o['replans']}/{o['replan_fail']}", flush=True)
    res_all['C'] = rows
json.dump(res_all, open(f'results_dynamic_{which}.json', 'w'), default=float)
