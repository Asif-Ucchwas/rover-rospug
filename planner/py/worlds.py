"""Seeded simulation worlds (occupancy grids, 1 = occupied), 5 cm cells.
corridor_world: an engineering-building floor (parameters are placeholders until the real corridor is surveyed)."""
import numpy as np
RES = 0.05

def _box(g, x0, y0, x1, y1, v=1):
    c = lambda v_: int(np.floor(v_ / RES + 0.5 + 1e-9))
    g[c(y0):c(y1), c(x0):c(x1)] = v

def gap_world(gap, size=(20.0, 10.0), wall_x=10.0, wall_t=0.2, gap_y=5.0):
    g = np.zeros((int(size[1] / RES), int(size[0] / RES)), np.uint8)
    _box(g, 0, 0, size[0], 0.1, 1); _box(g, 0, size[1] - 0.1, size[0], size[1], 1); _box(g, 0, 0, 0.1, size[1], 1); _box(g, size[0] - 0.1, 0, size[0], size[1], 1)
    _box(g, wall_x - wall_t / 2, 0, wall_x + wall_t / 2, size[1], 1); _box(g, wall_x - wall_t / 2, gap_y - gap / 2, wall_x + wall_t / 2, gap_y + gap / 2, 0)
    return g

def corridor_world(seed=0, doors_n=(0.85, 0.95, 1.05, 1.2, 1.4), doors_s=(0.80, 0.90, 1.00, 1.10), clutter=True):
    W, H = 36.0, 14.0; g = np.zeros((int(H / RES), int(W / RES)), np.uint8); rng = np.random.default_rng(seed)
    g[:] = 1
    _box(g, 1.0, 6.0, 33.8, 8.4, 0)                 # main corridor, 2.4 m wide
    _box(g, 32.0, 1.0, 33.8, 13.0, 0)               # north-south side corridor, 1.8 m wide (a 90 degree turn)
    xs = [1.0 + 6.0 * k for k in range(6)]          # six 5.8 m wide rooms per side up to x = 31
    for side, (y0, y1, doors, wy0, wy1) in {'n': (8.6, 13.0, doors_n, 8.4, 8.6), 's': (1.0, 5.8, doors_s, 5.8, 6.0)}.items():
        for k in range(len(doors)):
            x0 = xs[k] + 0.1; x1 = xs[k] + 5.7; _box(g, x0, y0, x1, y1, 0)
            cx = (x0 + x1) / 2 + (0.6 if k % 2 else -0.6); _box(g, cx - doors[k] / 2, wy0, cx + doors[k] / 2, wy1, 0)   # door gap
            if clutter:                              # furniture: a few boxes leaving at least 1.1 m of free floor next to walls
                for _ in range(3):
                    bw, bh = rng.uniform(0.4, 0.9, 2); bx = rng.uniform(x0 + 1.2, x1 - 1.2 - bw); by = rng.uniform(y0 + 1.2, y1 - 1.2 - bh) if y1 - y0 > 2.6 + bh else (y0 + y1) / 2
                    _box(g, bx, by, bx + bw, by + bh, 1)
    g[:2] = 1; g[-2:] = 1; g[:, :2] = 1; g[:, -2:] = 1
    return g

def clutter_world(size=20.0, density=0.06, seed=0, box=(0.3, 0.9)):
    rng = np.random.default_rng(seed); n = int(size * size / 0.36 * density); g = np.zeros((int(size / RES),) * 2, np.uint8)
    for _ in range(n):
        bw, bh = rng.uniform(*box, 2); x, y = rng.uniform(0.3, size - 1.3, 2); _box(g, x, y, x + bw, y + bh, 1)
    _box(g, 0, 0, size, 0.1, 1); _box(g, 0, size - 0.1, size, size, 1); _box(g, 0, 0, 0.1, size, 1); _box(g, size - 0.1, 0, size, size, 1)
    return g

def maze_world(cells=7, corridor=1.2, seed=0, wall=0.2, loops=0.1):
    rng = np.random.default_rng(seed); pitch = corridor + wall; n = cells; side = n * pitch + wall; N = int(round(side / RES))
    g = np.ones((N, N), np.uint8); vis = np.zeros((n, n), bool); st = [(0, 0)]; vis[0, 0] = True
    def cell(cx, cy): _box(g, wall + cx * pitch, wall + cy * pitch, wall + cx * pitch + corridor, wall + cy * pitch + corridor, 0)
    def link(a, b):
        (ax, ay), (bx, by) = a, b; x0 = wall + min(ax, bx) * pitch; y0 = wall + min(ay, by) * pitch
        if ax != bx: _box(g, x0 + corridor, y0, x0 + pitch, y0 + corridor, 0)
        else: _box(g, x0, y0 + corridor, x0 + corridor, y0 + pitch, 0)
    cell(0, 0)
    while st:
        cx, cy = st[-1]; nb = [(cx + dx, cy + dy) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)) if 0 <= cx + dx < n and 0 <= cy + dy < n and not vis[cy + dy, cx + dx]]
        if not nb: st.pop(); continue
        nx, ny = nb[rng.integers(len(nb))]; vis[ny, nx] = True; cell(nx, ny); link((cx, cy), (nx, ny)); st.append((nx, ny))
    for _ in range(int(n * n * loops)):
        cx, cy = rng.integers(0, n - 1), rng.integers(0, n); 
        if rng.random() < .5: link((cx, cy), (cx + 1, cy))
        else: link((cy, cx), (cy, cx + 1))
    return g, (wall + corridor / 2, wall + corridor / 2), (side - wall - corridor / 2, side - wall - corridor / 2)
