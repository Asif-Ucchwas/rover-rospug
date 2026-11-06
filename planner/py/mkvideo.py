"""60 s explainer video (simulation only): title, the door problem, a four-stop tour, end card. Segments -> concat with ffmpeg."""
import sys, json, subprocess, numpy as np; sys.path.insert(0, 'py')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt, matplotlib.animation as an
from matplotlib.patches import Circle, Polygon
from fpp import Planner, ROBOT, DEFAULT_PRM
from worlds import corridor_world, gap_world, RES
import viz
NAVY, TEAL, AMBER, RED, GREEN = '#11333d', '#16a394', '#eba33f', '#d9473a', '#2e9e5b'
FPS = 20; SIZE = (12.8, 7.2); DPI = 100
which = sys.argv[1]

def writer(name): return an.FFMpegWriter(fps=FPS, bitrate=4000, codec='libx264', extra_args=['-pix_fmt', 'yuv420p'])
def newfig(bg=NAVY):
    fig = plt.figure(figsize=SIZE, dpi=DPI, facecolor=bg); return fig

def build_traj(legs, v=1.0, w=np.radians(60), pause=0.0, target=None):
    """legs: list of (n,3) pose arrays. Returns T (n,), P (n,3) with pauses at leg ends, scaled so total time == target."""
    P = [legs[0][0]]; dts = [0.0]; kinds = [0]
    for li, leg in enumerate(legs):
        for k in range(1, len(leg)):
            d = np.hypot(*(leg[k, :2] - leg[k - 1, :2])); dth = abs(np.arctan2(np.sin(leg[k, 2] - leg[k - 1, 2]), np.cos(leg[k, 2] - leg[k - 1, 2])))
            P.append(leg[k]); dts.append(max(d / v, dth / w)); kinds.append(0)
        if pause > 0 and li < len(legs) - 1: P.append(leg[-1]); dts.append(pause); kinds.append(1)
    dts = np.array(dts); kinds = np.array(kinds); mv = dts[kinds == 0].sum(); ps = dts[kinds == 1].sum()
    if target: dts[kinds == 0] *= (target - ps) / mv
    return np.cumsum(dts), np.array(P)

def pose_at(T, P, t):
    t = min(max(t, 0), T[-1]); k = min(max(np.searchsorted(T, t, side='right') - 1, 0), len(P) - 2); u = (t - T[k]) / max(T[k + 1] - T[k], 1e-9); d = np.arctan2(np.sin(P[k + 1, 2] - P[k, 2]), np.cos(P[k + 1, 2] - P[k, 2]))
    return P[k, 0] + (P[k + 1, 0] - P[k, 0]) * u, P[k, 1] + (P[k + 1, 1] - P[k, 1]) * u, P[k, 2] + d * u

def card(lines, name, secs):   # text cards with a fade-in
    fig = newfig(); ax = fig.add_axes([0, 0, 1, 1]); ax.axis('off'); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.set_facecolor(NAVY)
    arts = []
    for ln_ in lines:
        txt, y, size, col, wt = ln_[:5]; xx = ln_[5] if len(ln_) > 5 else 0.07; arts.append(ax.text(xx, y, txt, color=col, fontsize=size, fontweight=wt, va='center', ha='left', alpha=0, wrap=True))
    ax.add_patch(plt.Rectangle((0.07, 0.115), 0.12, 0.008, color=TEAL))
    n = int(secs * FPS)
    def upd(f):
        for i, a_ in enumerate(arts): a_.set_alpha(min(1, max(0, (f / FPS - 0.3 * i) / 0.6)))
        return []
    ani = an.FuncAnimation(fig, upd, frames=n); ani.save(name, writer=writer(name), dpi=DPI); plt.close(fig)

if which == 'title':
    card([('SIMULATION', 0.78, 14, '#7fe0d3', 'bold'), ('A path planner for a\nlarge rectangular rover', 0.55, 40, 'white', 'bold'),
          ('1100 x 770 mm footprint. Door gaps down to 0.8 m.\nPlaceholder floor plan, not yet the real building.', 0.28, 17, '#c9d9dc', 'normal')], 'seg1.mp4', 5)

elif which == 'end':
    card([('WHAT THE TEST SHOWED', 0.84, 14, '#7fe0d3', 'bold'),
          ('60 trips that at least one planner could solve', 0.72, 25, 'white', 'bold'),
          ('Rover planned as a big circle', 0.58, 22, '#e9f3f3', 'normal', 0.07), ('1 collision-free route', 0.58, 22, '#e2685a', 'bold', 0.55),
          ('Rover planned as a smaller circle', 0.48, 22, '#e9f3f3', 'normal', 0.07), ('19 collision-free routes', 0.48, 22, '#eba33f', 'bold', 0.55),
          ('Rover planned as its real rectangle', 0.38, 22, '#e9f3f3', 'normal', 0.07), ('51 collision-free routes', 0.38, 22, '#7fe0d3', 'bold', 0.55),
          ('Next: measure the real ground floor, compare with Nav2, drive the real rover.', 0.25, 17, '#c9d9dc', 'normal'),
          ('Md Asifuzzaman  |  Lamar University', 0.16, 14, '#7fe0d3', 'normal')], 'seg4.mp4', 9)

elif which == 'problem':
    GAP = 0.85; g = gap_world(GAP); pl = Planner(g, RES, margin=0.0)
    start, goal = (3.0, 7.0, 0.0), (17.0, 3.0)
    ri = pl.plan('point_insc', start, goal, tight=True); rs = pl.plan('se2', start, goal, tight=True)
    assert ri['ok'] and rs['ok']
    Pi, Ps = ri['path'], rs['path']
    # first colliding sample along the inscribed-circle path
    hit = None
    for k in range(len(Pi) - 1):
        for q in np.linspace(0, 1, 40):
            x = Pi[k, 0] + (Pi[k + 1, 0] - Pi[k, 0]) * q; y = Pi[k, 1] + (Pi[k + 1, 1] - Pi[k, 1]) * q; th = Pi[k, 2]
            if pl.rect_hits(x, y, th): hit = (k, q, x, y, th); break
        if hit: break
    assert hit; k0, q0 = hit[0], hit[1]; Pi_c = np.vstack([Pi[:k0 + 1], [hit[2], hit[3], hit[4]]])
    DUR = 16.0; mt = 12.0
    Ti, PPi = build_traj([Pi_c], v=1.0, target=mt); Ts, PPs = build_traj([Ps], v=1.0, target=mt)
    # the circle model never gets a path: it drives to the door and stops
    circ_path = np.array([[3.0, 7.0, 0.0], [8.9, 5.0, 0.0]]); Tc, PPc = build_traj([circ_path], v=1.0, target=7.0)
    fig = newfig('#f6f9fb'); axs = [fig.add_axes([0.02 + 0.325 * i, 0.27, 0.31, 0.47], anchor='N') for i in range(3)]
    titles = ['Circle model (stock-style)', 'Smaller circle model', 'Real rectangle (this planner)']
    for ax, tt in zip(axs, titles):
        viz.draw_map(ax, g, RES, xlim=(7.8, 12.2), ylim=(2.8, 7.2)); ax.set_xticks([]); ax.set_yticks([]); ax.set_xlabel(''); ax.set_ylabel(''); ax.plot(*goal[:2], '*', color=NAVY, ms=13, zorder=7)
        ax.set_title(tt, fontsize=15, fontweight='bold', color=NAVY, pad=8)
    fig.text(0.02, 0.93, 'The same 0.85 m door, three ways of planning', fontsize=24, fontweight='bold', color=NAVY, va='center')
    fig.text(0.02, 0.865, 'Simulation. The rover is 0.77 m wide, so it fits, but only a planner that knows its real shape can use the door.', fontsize=13, color='#4f6b75', va='center')
    rc = viz.Rover(axs[1], z=6); rr = viz.Rover(axs[2], z=6)
    circ = Circle((3, 7), np.hypot(ROBOT['a'], ROBOT['b']), fc='#e2685a40', ec=RED, lw=2, zorder=6); axs[0].add_patch(circ)
    ln_i, = axs[1].plot([], [], color=AMBER, lw=2, zorder=4); ln_s, = axs[2].plot([], [], color=TEAL, lw=2.5, zorder=4)
    tx = [fig.text(0.02 + 0.325 * i + 0.155, 0.245, '', fontsize=16, fontweight='bold', ha='center', va='center') for i in range(3)]
    fig.text(0.5, 0.09, 'Circle model needs a gap of 1.35 m      |      The rover is 0.77 m wide      |      This door is 0.85 m', ha='center', va='center', fontsize=17, fontweight='bold', color=NAVY)
    boom = Circle((hit[2], hit[3]), 0.0, fc='none', ec=RED, lw=3, zorder=8); axs[1].add_patch(boom)
    def upd(f):
        t = f / FPS; tt = max(0, t - 1.5)
        x, y, th = pose_at(Tc, PPc, tt); circ.center = (x, y)
        if tt >= Tc[-1]: tx[0].set_text('No route: needs a 1.35 m gap'); tx[0].set_color(RED)
        x, y, th = pose_at(Ti, PPi, tt); rc.set(x, y, th); idx = np.searchsorted(Ti, tt); ln_i.set_data(PPi[:idx + 1, 0], PPi[:idx + 1, 1])
        if tt >= Ti[-1] - 0.01:
            tx[1].set_text('Route found, but the rover hits the wall'); tx[1].set_color(RED); boom.set_radius(0.15 + 0.3 * min(1, (tt - Ti[-1]) * 3) if tt >= Ti[-1] else 0)
        x, y, th = pose_at(Ts, PPs, tt); rr.set(x, y, th); idx = np.searchsorted(Ts, tt); ln_s.set_data(PPs[:idx + 1, 0], PPs[:idx + 1, 1])
        if tt >= Ts[-1] - 0.01: tx[2].set_text('Safe route through the door'); tx[2].set_color(GREEN)
        return []
    ani = an.FuncAnimation(fig, upd, frames=int(DUR * FPS)); ani.save('seg2.mp4', writer=writer('seg2'), dpi=DPI); plt.close(fig)

elif which == 'tour':
    g = corridor_world(0); pl = Planner(g, RES, margin=0.02); clr = pl.clearance_map(); H, W = g.shape
    def near_free(x, y, need=0.52):
        best = None
        for r in np.arange(0, 3, 0.1):
            for ang in np.linspace(0, 2 * np.pi, 24, endpoint=False):
                xx, yy = x + r * np.cos(ang), y + r * np.sin(ang)
                if pl.clearance(xx, yy) >= need: return (xx, yy)
        return None
    cxn = lambda k: 1 + 6 * k + 2.9 + (0.6 if k % 2 else -0.6)
    stops = [(2.5, 7.2), near_free(cxn(1), 10.9), near_free(cxn(2), 3.3), near_free(cxn(3), 10.9), (32.9, 12.2)]
    legs = []; cur = (stops[0][0], stops[0][1], 0.0)
    for s_ in stops[1:]:
        r = pl.plan('se2', cur, s_, tight=True)
        if not r['ok']: FINE = list(DEFAULT_PRM); FINE[1] = 72; FINE[2] = 0.1; r = pl.plan('se2', cur, s_, tight=True, prm=FINE)
        assert r['ok'], ('leg failed', cur, s_); legs.append(r['path']); cur = tuple(r['path'][-1])
    T, P = build_traj(legs, v=1.0, pause=1.0, target=31.0); DUR = T[-1] + 1.0
    print('tour', DUR, 'legs', len(legs), 'lengths', [round(float(np.hypot(*np.diff(l[:, :2], axis=0).T).sum()), 1) for l in legs])
    fig = newfig('#f6f9fb'); ax = fig.add_axes([0.0, 0.0, 1, 0.9]); viz.draw_map(ax, g, RES, xlim=(0, 16), ylim=(0, 9)); ax.set_xticks([]); ax.set_yticks([]); ax.set_xlabel(''); ax.set_ylabel('')
    for i, s_ in enumerate(stops[1:]): ax.plot(*s_, 'o', color=AMBER, ms=16, mec=NAVY, zorder=5); ax.text(s_[0], s_[1], str(i + 1), color=NAVY, fontsize=10, fontweight='bold', ha='center', va='center', zorder=6)
    allp = np.vstack(legs); ax.plot(allp[:, 0], allp[:, 1], color='#9fd8d0', lw=1.5, zorder=3); drawn, = ax.plot([], [], color=TEAL, lw=3, zorder=4); rover = viz.Rover(ax, z=7)
    fig.text(0.02, 0.955, 'Four-stop tour of the simulated floor', fontsize=22, fontweight='bold', color=NAVY, va='center'); status = fig.text(0.98, 0.955, '', fontsize=16, color='#4f6b75', va='center', ha='right')
    mm = fig.add_axes([0.665, 0.03, 0.32, 0.19]); viz.draw_map(mm, g, RES); mm.set_xticks([]); mm.set_yticks([]); mm.set_xlabel(''); mm.set_ylabel(''); mm.plot(allp[:, 0], allp[:, 1], color=TEAL, lw=1); dot, = mm.plot([], [], 'o', color=RED, ms=6, zorder=8)
    for s_ in stops[1:]: mm.plot(*s_, 'o', color=AMBER, ms=4, zorder=7)
    for sp in mm.spines.values(): sp.set_color(NAVY); sp.set_linewidth(2)
    # smooth camera
    cam = []; c = np.array(pose_at(T, P, 0)[:2]); 
    for f in range(int(DUR * FPS)):
        x, y, _ = pose_at(T, P, f / FPS); c = c + 0.12 * (np.array([x, y]) - c); cam.append(c.copy())
    # stop counter: leg boundaries
    bounds = [np.searchsorted(T, 0)]; 
    ends = []; acc = 0
    for li, leg in enumerate(legs): acc += len(leg) - 1 + (1 if li < len(legs) - 1 else 0); ends.append(T[min(acc, len(T) - 1)])
    def upd(f):
        t = f / FPS; x, y, th = pose_at(T, P, t); rover.set(x, y, th); cx, cy = cam[f]; hw, hh = 8.0, 4.05
        cx = min(max(cx, hw - 0.5), W * RES - hw + 0.5); cy = min(max(cy, hh - 0.5), H * RES - hh + 0.5); ax.set_xlim(cx - hw, cx + hw); ax.set_ylim(cy - hh, cy + hh)
        idx = np.searchsorted(T, t); drawn.set_data(P[:idx + 1, 0], P[:idx + 1, 1]); dot.set_data([x], [y])
        done = sum(t >= e for e in ends); status.set_text(f'Stops reached: {done} of {len(legs)}')
        return []
    ani = an.FuncAnimation(fig, upd, frames=int(DUR * FPS)); ani.save('seg3.mp4', writer=writer('seg3'), dpi=DPI); plt.close(fig)
