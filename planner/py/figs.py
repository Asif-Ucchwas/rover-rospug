import sys, json, numpy as np; sys.path.insert(0, 'py')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
from fpp import Planner, ROBOT, path_stats, DEFAULT_PRM
from worlds import corridor_world, RES
import viz
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
COL = {'point_circ': '#e2685a', 'point_insc': '#eba33f', 'theta_rect': '#5b7b8c', 'se2': '#16a394'}
LAB = {'point_circ': 'Point A*, circumscribed inflation (stock-style)', 'point_insc': 'Point A*, inscribed inflation', 'theta_rect': 'Theta* + rectangle check', 'se2': 'SE(2) lattice A* + rectangle check'}
a, b = ROBOT['a'], ROBOT['b']

def rect_poly(x, y, th):
    c, s = np.cos(th), np.sin(th); P = np.array([[a, b], [a, -b], [-a, -b], [-a, b]]); return np.c_[x + P[:, 0] * c - P[:, 1] * s, y + P[:, 0] * s + P[:, 1] * c]

def draw_map(ax, g, ext=None): viz.draw_map(ax, g, RES)

# ---- Fig 1: gap sweep
R = json.load(open('results_gap.json')); gaps = sorted({float(k.split('|')[0]) for k in R}); ms = ['point_circ', 'point_insc', 'theta_rect', 'se2']
fig, ax = plt.subplots(figsize=(9, 2.9))
for yi, m in enumerate(ms[::-1]):
    for gp in gaps:
        ok, val, *_ = R[f'{gp}|{m}']; c = '#2e9e5b' if (ok and val) else ('#d9473a' if ok else '#c9d3d6')
        ax.add_patch(Rectangle((gp - 0.025, yi - 0.4), 0.05, 0.8, color=c))
ax.set_yticks(range(4)); ax.set_yticklabels([LAB[m].split(' (')[0] for m in ms[::-1]], fontsize=8.5)
ax.axvline(2 * b, color='k', ls='--', lw=1); ax.text(2 * b, 3.55, ' rover width 0.77 m', fontsize=8, va='bottom'); ax.axvline(2 * np.hypot(a, b), color='k', ls=':', lw=1); ax.text(2 * np.hypot(a, b), 3.55, ' circumscribed diameter 1.34 m', fontsize=8, va='bottom')
ax.set_xlim(min(gaps) - .03, max(gaps) + .03); ax.set_ylim(-.6, 4.1); ax.set_xlabel('door gap width (m)')
ax.legend(handles=[Rectangle((0, 0), 1, 1, color=c) for c in ('#2e9e5b', '#d9473a', '#c9d3d6')], labels=['path found, collision-free', 'path found, but the rover collides', 'no path'], ncol=3, loc='upper center', bbox_to_anchor=(.5, -.35), frameon=False, fontsize=8.5)
ax.set_title('Which door gaps each planner can use (start and goal fixed, 20 cm wall)', fontsize=10.5, loc='left'); fig.tight_layout(); fig.savefig('fig1_gap.png', dpi=170); plt.close(fig)

# ---- Fig 2: paths in the corridor world
g = corridor_world(seed=0); pl = Planner(g, RES, margin=0.02); vpl = Planner(g, RES, margin=0.0); C = json.load(open('results_corridor.json')); rows = C['rows']; pairs = C['pairs']
ok_by = {}
for r in rows: ok_by.setdefault(r['q'], {})[r['method']] = r
cands = [q for q in ok_by if all(ok_by[q].get(m, {}).get('ok') and ok_by[q][m]['bad'] == 0 for m in ('theta_rect', 'se2'))]
cands.sort(key=lambda q: -ok_by[q]['se2']['rot'] if ok_by[q]['se2']['length'] < 40 else 0); QI = cands[0] if cands else 0; s, t, h0 = pairs[QI]
json.dump(dict(q=QI), open('fig_query.json', 'w'))
FINE = list(DEFAULT_PRM)
fig, axs = plt.subplots(2, 1, figsize=(10, 7.4), gridspec_kw=dict(height_ratios=[1, 1.0]))
ax = axs[0]; draw_map(ax, g)
for m, tight in (('point_circ', True), ('point_insc', True), ('theta_rect', True), ('se2', True)):
    r = pl.plan(m, (s[0], s[1], h0), t, tight=tight)
    if r['ok']: ax.plot(r['path'][:, 0], r['path'][:, 1], color=COL[m], lw=2.2 if m == 'se2' else 1.6, label=LAB[m] + ('' if vpl.validate(r['path']) == 0 else '  [collides]'), zorder=4)
    else: ax.plot([], [], color=COL[m], label=LAB[m] + '  [no path]')
ax.plot(*s[:2], 'o', color='#12333a', ms=7, zorder=5); ax.plot(*t, '*', color='#12333a', ms=12, zorder=5); ax.legend(loc='upper center', bbox_to_anchor=(.5, -.17), fontsize=8, ncol=2, frameon=False)
ax.set_title(f'Engineering-floor world (placeholder dimensions), query {QI}: all four planners', fontsize=10.5, loc='left'); 
ax = axs[1]; draw_map(ax, g); r = pl.plan('se2', (s[0], s[1], h0), t, tight=True); P = r['path']
sel = [0]; acc = 0.0
for k in range(1, len(P)):
    acc += np.hypot(*(P[k, :2] - P[k - 1, :2]))
    if acc >= 1.2 or abs(np.arctan2(np.sin(P[k, 2] - P[sel[-1], 2]), np.cos(P[k, 2] - P[sel[-1], 2]))) > np.radians(40): sel.append(k); acc = 0.0
sel.append(len(P) - 1)
for k in sel:
    ax.add_patch(Polygon(rect_poly(*P[k]), closed=True, fc='#16a39430', ec='#0f6f73', lw=.6, zorder=3)); viz.Rover(ax, z=5).set(*P[k])
ax.plot(P[:, 0], P[:, 1], color='#0f6f73', lw=1.2, zorder=4); ax.plot(*s[:2], 'o', color='#12333a', ms=7, zorder=5); ax.plot(*t, '*', color='#12333a', ms=12, zorder=5)
L, rot = path_stats(P); ax.set_title(f'The 1100 x 770 mm footprint along the SE(2) path ({L:.1f} m driven, {rot:.0f} deg turning in place, 0 collisions)', fontsize=10, loc='left')
fig.tight_layout(); fig.savefig('fig2_paths.png', dpi=170); plt.close(fig)

# ---- Fig 3: summary
feas = {r['q'] for r in rows if r['ok'] and r['bad'] == 0}; names = ['point_circ', 'point_insc', 'theta_rect', 'se2']
stat = {}
for m in names:
    Rm = [r for r in rows if r['method'] == m and r['q'] in feas]; stat[m] = (sum(r['ok'] for r in Rm), sum(1 for r in Rm if r['ok'] and r['bad'] == 0), len(Rm), np.median([r['secs'] for r in Rm if r['ok']] or [np.nan]) * 1000)
fig, axs = plt.subplots(1, 2, figsize=(10, 3.4)); x = np.arange(4)
axs[0].bar(x - .2, [100 * stat[m][0] / stat[m][2] for m in names], .38, color='#c9d3d6', label='path returned'); axs[0].bar(x + .2, [100 * stat[m][1] / stat[m][2] for m in names], .38, color=[COL[m] for m in names], label='collision-free path')
axs[0].set_xticks(x); axs[0].set_xticklabels(['point A*\ncircumscribed', 'point A*\ninscribed', 'Theta*\n+ rectangle', 'SE(2) A*\n+ rectangle'], fontsize=8.5); axs[0].set_ylabel(f'% of {stat[names[0]][2]} reachable queries'); axs[0].set_ylim(0, 125); axs[0].set_yticks([0, 20, 40, 60, 80, 100]); axs[0].legend(frameon=False, fontsize=8.5, ncol=2, loc='upper center')
axs[1].bar(x, [stat[m][3] for m in names], .6, color=[COL[m] for m in names]); axs[1].set_yscale('log'); axs[1].set_xticks(x); axs[1].set_xticklabels(['point A*\ncircumscribed', 'point A*\ninscribed', 'Theta*\n+ rectangle', 'SE(2) A*\n+ rectangle'], fontsize=8.5); axs[1].set_ylabel('median planning time (ms, log)')
for i, m in enumerate(names): axs[1].text(i, stat[m][3] * 1.15, f'{stat[m][3]:.1f}' if stat[m][3] < 10 else f'{stat[m][3]:.0f}', ha='center', fontsize=8.5)
fig.suptitle('Corridor world, 80 random queries (reachable = some planner found a collision-free path)', fontsize=10, x=.01, ha='left'); fig.tight_layout(); fig.savefig('fig3_summary.png', dpi=170); plt.close(fig)
print(stat)
