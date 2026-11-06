import sys, json, numpy as np; sys.path.insert(0, 'py')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt, matplotlib.animation as an
from matplotlib.patches import Polygon
from fpp import Planner, ROBOT; from worlds import corridor_world, RES; import viz
a, b = ROBOT['a'], ROBOT['b']
g = corridor_world(0); pl = Planner(g, RES, margin=0.02); C = json.load(open('results_corridor.json')); QI = json.load(open('fig_query.json'))['q']; s, t, h0 = C['pairs'][QI]
P = pl.plan('se2', (s[0], s[1], h0), t, tight=True)['path']
T = [0.0]
for k in range(1, len(P)):
    d = np.hypot(*(P[k, :2] - P[k - 1, :2])); dth = abs(np.arctan2(np.sin(P[k, 2] - P[k - 1, 2]), np.cos(P[k, 2] - P[k - 1, 2]))); T.append(T[-1] + max(d / 0.8, dth / np.radians(60)))
T = np.array(T); fps = 20; n = int(T[-1] * fps) + 1
def pose(tt):
    k = min(max(np.searchsorted(T, tt) - 1, 0), len(P) - 2); u = (tt - T[k]) / max(T[k + 1] - T[k], 1e-9); d = np.arctan2(np.sin(P[k + 1, 2] - P[k, 2]), np.cos(P[k + 1, 2] - P[k, 2]))
    return P[k, 0] + (P[k + 1, 0] - P[k, 0]) * u, P[k, 1] + (P[k + 1, 1] - P[k, 1]) * u, P[k, 2] + d * u
def poly(x, y, th):
    c, s_ = np.cos(th), np.sin(th); Q = np.array([[a, b], [a, -b], [-a, -b], [-a, b]]); return np.c_[x + Q[:, 0] * c - Q[:, 1] * s_, y + Q[:, 0] * s_ + Q[:, 1] * c]
Hh, W = g.shape; x0, x1 = min(P[:, 0].min(), t[0]) - 3, max(P[:, 0].max(), t[0]) + 3; y0, y1 = min(P[:, 1].min(), t[1]) - 3, max(P[:, 1].max(), t[1]) + 3
fig, ax = plt.subplots(figsize=(9, 9 * (y1 - y0) / (x1 - x0) + .5)); viz.draw_map(ax, g, RES, xlim=(x0, x1), ylim=(y0, y1)); ax.plot(*t, '*', color='#12333a', ms=14, zorder=6)
ax.plot(P[:, 0], P[:, 1], color='#0f6f73', lw=1, alpha=.6, zorder=2); rover = viz.Rover(ax, z=6).set(*P[0])
ax.set_title('SE(2) planner: 1100 x 770 mm rover through the placeholder engineering floor', fontsize=10, loc='left'); fig.tight_layout()
def upd(f):
    x, y, th = pose(f / fps); rover.set(x, y, th)
    if f % 6 == 0:
        p = Polygon(poly(x, y, th), closed=True, fc='#16a39428', ec='none', zorder=3); ax.add_patch(p)
    return []
ani = an.FuncAnimation(fig, upd, frames=n, blit=False); ani.save('rover_se2.mp4', writer=an.FFMpegWriter(fps=fps, bitrate=2500), dpi=90); print('frames', n, 'seconds', n / fps)
