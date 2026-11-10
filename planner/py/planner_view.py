"""See the planner work. Click a start and a goal on the map; the planner draws the rover's real 1.12 x 0.81 m footprint along its path.
Run from the planner folder:   python3 py/planner_view.py            (engineering floor, placeholder map)
                               python3 py/planner_view.py --world gap (one wall with a door: change the door width with up/down arrows)
Mouse:  left click = start   |   right click = goal
Keys :  1 point planner, circle model   2 point planner, inscribed circle   3 Theta* with rectangle   4 SE(2) lattice planner (ours, default)
        up/down = door gap wider/narrower (gap world), r = rotate start heading, q = quit
Why it matters: with the circle model (key 1) the rover looks too big for most doors and refuses; the rectangle planners (3, 4) use its real shape."""
import sys, os, argparse, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import matplotlib
ap = argparse.ArgumentParser(); ap.add_argument('--world', default='corridor', choices=['corridor', 'gap']); ap.add_argument('--test', action='store_true', help='draw one example, save planner_view_test.png and exit'); args = ap.parse_args()
if args.test: matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from fpp import Planner, ROBOT
from worlds import corridor_world, gap_world, RES
A_, B_ = ROBOT['a'], ROBOT['b']; METHODS = {'1': ('point_circ', 'point planner, circle model'), '2': ('point_insc', 'point planner, inscribed circle'), '3': ('theta_rect', 'Theta* with rectangle'), '4': ('se2', 'SE(2) lattice planner (ours)')}
st = dict(method='4', gap=0.95, start=None, goal=None, rot=0, quit=False)
def world(): return corridor_world(seed=0) if args.world == 'corridor' else gap_world(st['gap'])
g = world(); H, W = g.shape
fig, ax = plt.subplots(figsize=(14, 7.5)); fig.subplots_adjust(0.02, 0.02, 0.98, 0.94)
def rect(x, y, th, **kw):
    c, s = np.cos(th), np.sin(th); return Polygon(np.array([[A_, B_], [A_, -B_], [-A_, -B_], [-A_, B_]]) @ np.array([[c, s], [-s, c]]) + [x, y], closed=True, **kw)
arts = []
def redraw():
    global g, H, W
    for a in arts: a.remove()
    arts.clear(); ax.clear(); g = world(); H, W = g.shape
    ax.imshow(1 - g, cmap='gray', origin='lower', extent=(0, W * RES, 0, H * RES), vmin=-0.3, vmax=1); ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    name, desc = METHODS[st['method']]; msg = f"[{st['method']}] {desc}"
    if args.world == 'gap': msg += f"   |   door gap {st['gap']:.2f} m (rover width {2 * B_:.2f} m)"
    if st['start'] is not None: ax.plot(*st['start'][:2], 'o', c='#2ca02c', ms=12, zorder=6)
    if st['goal'] is not None: ax.plot(*st['goal'], '*', c='#d62728', ms=18, zorder=6)
    if st['start'] is not None and st['goal'] is not None:
        pl = Planner(g, RES, margin=0.02); truth = Planner(g, RES, margin=0.0); sx, sy = st['start'][:2]
        hs = np.linspace(-np.pi, np.pi, 72, endpoint=False); toward = np.arctan2(st['goal'][1] - sy, st['goal'][0] - sx) + st['rot'] * np.pi / 4
        order = sorted(hs, key=lambda h: abs((h - toward + np.pi) % (2 * np.pi) - np.pi)); th0 = next((h for h in order if not pl.rect_hits(sx, sy, h)), None)
        if th0 is None: msg += '   |   START IS TOO CLOSE TO A WALL for the rover (no heading fits)'
        else:
            r = pl.plan(name, (sx, sy, th0), st['goal'], tight=True)
            if not r['ok']: msg += f"   |   NO PATH ({r['secs'] * 1000:.0f} ms): the planner says the rover cannot get there"
            else:
                P = r['path']; bad = truth.validate(P); d = np.hypot(*np.diff(P[:, :2], axis=0).T).sum(); ax.plot(P[:, 0], P[:, 1], '-', c='#1f77b4', lw=2, zorder=4)
                acc = 0; last = None
                for k in range(len(P)):
                    if last is None or np.hypot(P[k, 0] - last[0], P[k, 1] - last[1]) > 0.7 or (k > 0 and abs(P[k, 2] - P[k - 1, 2]) > 1e-9 and k % 6 == 0):
                        ax.add_patch(rect(P[k, 0], P[k, 1], P[k, 2], fc='none', ec='#ff7f0e', lw=1.2, alpha=0.9, zorder=5)); last = P[k, :2]
                ax.add_patch(rect(P[-1, 0], P[-1, 1], P[-1, 2], fc='#2ca02c', ec='k', alpha=0.6, zorder=5)); msg += f"   |   path {d:.1f} m, planned in {r['secs'] * 1000:.0f} ms, " + ('0 collisions on the true footprint' if bad == 0 else f'{bad} COLLISIONS (this model is too optimistic)')
    ax.set_title(msg + ("\nleft click = start, right click = goal, 1-4 = method, r = rotate start, q = quit" + ("   up/down = door width" if args.world == 'gap' else '')), fontsize=11, loc='left'); fig.canvas.draw_idle()
def click(e):
    if e.inaxes is not ax: return
    if e.button == 1: st['start'] = (e.xdata, e.ydata, 0.0)
    elif e.button == 3: st['goal'] = (e.xdata, e.ydata)
    redraw()
def key(e):
    if e.key in METHODS: st['method'] = e.key
    elif e.key == 'up': st['gap'] = round(min(st['gap'] + 0.05, 2.0), 2)
    elif e.key == 'down': st['gap'] = round(max(st['gap'] - 0.05, 0.5), 2)
    elif e.key == 'r': st['rot'] = (st['rot'] + 1) % 8
    elif e.key == 'q': plt.close(fig); return
    redraw()
fig.canvas.mpl_connect('button_press_event', click); fig.canvas.mpl_connect('key_press_event', key)
if args.world == 'corridor': st['start'], st['goal'] = (2.0, 7.2, 0.0), (30.0, 7.2)
else: st['start'], st['goal'] = (3.0, 7.0, 0.0), (17.0, 3.0)
redraw()
if args.test: fig.savefig('planner_view_test.png', dpi=60)
else: plt.show()
