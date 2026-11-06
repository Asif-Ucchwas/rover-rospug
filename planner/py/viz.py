"""Shared drawing helpers: graph-paper map and a top-down rover icon (1100 x 770 mm footprint)."""
import numpy as np
from matplotlib.patches import Polygon, Circle
PAPER, WALL, GRID = (0.965, 0.978, 0.992), (0.16, 0.20, 0.25), '#9fb8cc'

def draw_map(ax, g, res, xlim=None, ylim=None):
    H, W = g.shape; img = np.zeros((H, W, 3)); img[:] = PAPER; img[g == 1] = WALL
    ax.imshow(img, origin='lower', extent=[0, W * res, 0, H * res], interpolation='nearest', zorder=0)
    xs = np.arange(0, W * res + 1e-9, 1.0); ys = np.arange(0, H * res + 1e-9, 1.0)
    for x in xs: ax.axvline(x, color=GRID, lw=0.9 if x % 5 == 0 else 0.35, alpha=0.8 if x % 5 == 0 else 0.55, zorder=1)
    for y in ys: ax.axhline(y, color=GRID, lw=0.9 if y % 5 == 0 else 0.35, alpha=0.8 if y % 5 == 0 else 0.55, zorder=1)
    ax.set_xticks(np.arange(0, W * res + 1e-9, 5)); ax.set_yticks(np.arange(0, H * res + 1e-9, 5)); ax.tick_params(labelsize=8, length=3, colors='#4f6b75')
    ax.set_xlabel('x (m)', fontsize=8.5, color='#4f6b75'); ax.set_ylabel('y (m)', fontsize=8.5, color='#4f6b75'); ax.set_aspect('equal')
    for s in ax.spines.values(): s.set_color('#9fb8cc')
    ax.set_xlim(*(xlim or (0, W * res))); ax.set_ylim(*(ylim or (0, H * res)))

def _rect(cx, cy, hl, hw): return np.array([[cx + hl, cy + hw], [cx + hl, cy - hw], [cx - hl, cy - hw], [cx - hl, cy + hw]])

# (kind, local params, face, edge, zorder offset)
PARTS = [('poly', _rect(0, 0, 0.50, 0.27), '#16a394', '#0f6f73', 0),               # chassis
         ('poly', _rect(0.33, 0.335, 0.17, 0.05), '#12333a', '#12333a', 1), ('poly', _rect(0.33, -0.335, 0.17, 0.05), '#12333a', '#12333a', 1),
         ('poly', _rect(-0.33, 0.335, 0.17, 0.05), '#12333a', '#12333a', 1), ('poly', _rect(-0.33, -0.335, 0.17, 0.05), '#12333a', '#12333a', 1),   # wheels
         ('poly', _rect(-0.17, 0, 0.27, 0.20), '#7fe0d3', '#0f6f73', 1),             # cargo deck
         ('poly', _rect(0.475, 0, 0.025, 0.20), '#eba33f', '#eba33f', 1),            # front light bar
         ('circ', (0.34, 0.0, 0.085), '#12333a', '#12333a', 2), ('circ', (0.34, 0.0, 0.035), '#eba33f', '#eba33f', 3)]  # LiDAR

class Rover:
    def __init__(self, ax, z=5, alpha=1.0):
        self.parts = []
        for kind, p, fc, ec, dz in PARTS:
            if kind == 'poly': art = Polygon(p, closed=True, fc=fc, ec=ec, lw=0.8, zorder=z + dz, alpha=alpha)
            else: art = Circle((p[0], p[1]), p[2], fc=fc, ec=ec, lw=0.8, zorder=z + dz, alpha=alpha)
            ax.add_patch(art); self.parts.append((kind, p, art))
    def set(self, x, y, th):
        c, s = np.cos(th), np.sin(th); R = np.array([[c, -s], [s, c]])
        for kind, p, art in self.parts:
            if kind == 'poly': art.set_xy(p @ R.T + [x, y])
            else: art.center = (R @ np.array(p[:2]) + [x, y])
        return self
