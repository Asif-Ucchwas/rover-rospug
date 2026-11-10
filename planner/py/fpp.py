"""ctypes wrapper around libfpp.so (footprint-aware planning core)."""
import ctypes as C, os, numpy as np
def _find_lib():
    root = os.path.join(os.path.dirname(__file__), '..')
    for p in ('build/libfpp.so', 'libfpp.so'):
        if os.path.exists(os.path.join(root, p)):
            return os.path.join(root, p)
    raise OSError('libfpp.so not found: run  cmake -S . -B build && cmake --build build -j')
_lib = C.CDLL(_find_lib())
D, P = C.c_double, C.c_void_p
_lib.fpp_create.restype = P
_lib.fpp_create.argtypes = [C.c_void_p, C.c_int, C.c_int, D, D, D, D]
_lib.fpp_destroy.argtypes = [P]
_lib.fpp_set_cost.argtypes = [P, C.c_void_p, D]
_lib.fpp_rect_hits.argtypes = [P, D, D, D]
_lib.fpp_sweep_hits.argtypes = [P, D, D, D, D, D]
_lib.fpp_clearance.restype = D; _lib.fpp_clearance.argtypes = [P, D, D]
_lib.fpp_edt.argtypes = [P, C.c_void_p]
_lib.fpp_validate.argtypes = [P, C.c_void_p, C.c_int, D, D]
_lib.fpp_plan.argtypes = [P, C.c_int, C.c_int, D, D, D, D, D, D, C.c_void_p, C.c_void_p, C.c_int, C.c_void_p]

METHODS = {'point_circ': 0, 'point_insc': 1, 'theta_rect': 2, 'se2': 3, 'rrt': 4, 'rrt_corr': 5, 'rrt_corr_fb': 6}
# defaults: se2 cell, theta bins, step, w_rot, reverse penalty, goal tol, max nodes, time limit s
# ... then RRT*: iterations, corridor half-width m, rewire radius m, goal bias, rng seed
DEFAULT_PRM = [0.09, 24, 0.30, 0.5, 1.5, 0.15, 3e6, 30.0, 0.0, 2000, 2.0, 2.0, 0.05, 1]
ROBOT = dict(a=0.56, b=0.405)   # Rover MEGA measured 1120 x 810 mm (vendor spec 1100 x 770)


class Planner:
    def __init__(self, occ, res, a=ROBOT['a'], b=ROBOT['b'], margin=0.0):
        self.occ = np.ascontiguousarray(occ.astype(np.uint8)); self.res = res
        self.H, self.W = self.occ.shape; self.a, self.b, self.margin = a, b, margin
        self.h = _lib.fpp_create(self.occ.ctypes.data, self.W, self.H, res, a, b, margin)

    def __del__(self):
        try: _lib.fpp_destroy(self.h)
        except Exception: pass

    def set_cost(self, cost, weight):
        """cost: uint8 grid, Nav2 convention (0 free, 1..252 inflated, 253 inscribed, 254 lethal), or None. weight scales (1 + weight*cost/252)."""
        self._cost = None if cost is None else np.ascontiguousarray(cost.astype(np.uint8))
        _lib.fpp_set_cost(self.h, None if cost is None else self._cost.ctypes.data, weight)

    def rect_hits(self, x, y, th): return bool(_lib.fpp_rect_hits(self.h, x, y, th))
    def sweep_hits(self, x0, y0, x1, y1, th): return bool(_lib.fpp_sweep_hits(self.h, x0, y0, x1, y1, th))
    def clearance(self, x, y): return _lib.fpp_clearance(self.h, x, y)

    def clearance_map(self):
        out = np.zeros((self.H, self.W), np.float32); _lib.fpp_edt(self.h, out.ctypes.data); return out

    def validate(self, path, step_xy=0.005, step_rot=np.radians(0.25)):
        p = np.ascontiguousarray(path, dtype=np.float64)
        return int(_lib.fpp_validate(self.h, p.ctypes.data, len(p), step_xy, step_rot))

    def plan(self, method, start, goal, tight=True, prm=None, maxn=200000):
        """start=(x,y,th) goal=(x,y). Returns dict(ok, path (n,3), expanded, secs, checks)."""
        prm = np.array(DEFAULT_PRM if prm is None else prm, dtype=np.float64)
        out = np.zeros(maxn * 3); st = np.zeros(4)
        n = _lib.fpp_plan(self.h, METHODS[method] if isinstance(method, str) else method, int(tight),
                          start[0], start[1], start[2], goal[0], goal[1], float('nan'),
                          prm.ctypes.data, out.ctypes.data, maxn, st.ctypes.data)
        r = dict(ok=n > 0, expanded=st[0], secs=st[1], checks=st[2], fallback=bool(st[3]), path=None)
        if n > 0: r['path'] = out[:3 * n].reshape(n, 3).copy()
        return r


def path_stats(path):
    """length (translation), rotation total deg."""
    d = np.diff(path[:, :2], axis=0); L = float(np.hypot(d[:, 0], d[:, 1]).sum())
    dth = np.abs((np.diff(path[:, 2]) + np.pi) % (2 * np.pi) - np.pi); return L, float(np.degrees(dth.sum()))


def inflation_costmap(planner, inflation_radius=1.2, scaling=3.0, inscribed=None):
    """Nav2-style inflation layer from distance to the nearest obstacle surface d:
    lethal 254 inside obstacles, 253 for d <= inscribed radius, else 252*exp(-scaling*(d-inscribed)) up to inflation_radius."""
    ins = planner.b if inscribed is None else inscribed
    d = planner.clearance_map().astype(np.float64); c = np.where(d <= ins, 253.0, 252.0 * np.exp(-scaling * (d - ins)))
    c[d >= inflation_radius] = 0; c[planner.occ == 1] = 254
    return np.clip(np.rint(c), 0, 254).astype(np.uint8)
