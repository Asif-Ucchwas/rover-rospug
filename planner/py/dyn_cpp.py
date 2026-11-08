"""ctypes bindings for the C++ Tracker and Governor (src/dynamic.cpp); same interface as dyn.Tracker / dyn.Governor."""
import ctypes as C, numpy as np
from fpp import _lib
D, P, I = C.c_double, C.c_void_p, C.c_int
_lib.fpp_gov_allowed.restype = D; _lib.fpp_gov_allowed.argtypes = [P, P, I, I, D, D, D, D, P, I, P]
_lib.fpp_gov_free.restype = D; _lib.fpp_gov_free.argtypes = [P, P, I, I, D, D, D, D, P, I, D, D]
_lib.fpp_gov_proximity.restype = D; _lib.fpp_gov_proximity.argtypes = [P, D, D, D, P, I]
_lib.fpp_gov_speed_limit.restype = D; _lib.fpp_gov_speed_limit.argtypes = [P, D]
_lib.fpp_trk_create.restype = P; _lib.fpp_trk_create.argtypes = [P, I, I, D]
_lib.fpp_trk_destroy.argtypes = [P]
_lib.fpp_trk_update.restype = I; _lib.fpp_trk_update.argtypes = [P, D, D, P, P, I, D, P, I]

def _segs(segs):
    a = np.zeros((max(len(segs), 1), 9))
    for i, s in enumerate(segs):
        if s['t'] == 'line': a[i, :6] = (0, s['x0'], s['y0'], s['x1'], s['y1'], s['th'])
        else: a[i, :] = (1, s['x'], s['y'], 0, 0, 0, s['th0'], s['th1'], s['d'])
    return np.ascontiguousarray(a)
def _tracks(tr):
    a = np.zeros((max(len(tr), 1), 8))
    for i, t in enumerate(tr): a[i] = (t['x'], t['y'], t['vx'], t['vy'], t['r'], t['age'], t.get('miss', 0), t.get('id', 0))
    return np.ascontiguousarray(a)

class Governor:
    def __init__(s, prof, margin=0.10, keepout=0.30, buf=0.25):
        s.p = prof; s.keepout, s.buf, s.margin = keepout, buf, margin; s.creep = 0.25; s.keepout_min = 0.12
        s.prm = np.array([prof['a'], prof['b'], prof['v_max'], prof['w_max'], prof['a_brake'], prof['t_react'], margin, keepout, buf, s.creep, s.keepout_min])
    def allowed_speed(s, segs, k, x, y, th, v, tracks):
        d = C.c_double(); S, T = _segs(segs), _tracks(tracks)
        lim = _lib.fpp_gov_allowed(s.prm.ctypes.data, S.ctypes.data, len(segs), k, x, y, th, v, T.ctypes.data, len(tracks), C.byref(d)); return lim, d.value
    def free_distance(s, segs, k, x, y, th, v, tracks, look=7.0, keepout=None):
        S, T = _segs(segs), _tracks(tracks)
        return _lib.fpp_gov_free(s.prm.ctypes.data, S.ctypes.data, len(segs), k, x, y, th, v, T.ctypes.data, len(tracks), s.keepout if keepout is None else keepout, look)
    def speed_limit(s, d): return _lib.fpp_gov_speed_limit(s.prm.ctypes.data, d)
    def proximity_limit(s, x, y, th, tracks):
        T = _tracks(tracks); return _lib.fpp_gov_proximity(s.prm.ctypes.data, x, y, th, T.ctypes.data, len(tracks))

class Tracker:
    def __init__(s, occ, res, rng=12.0):
        s._occ = np.ascontiguousarray(occ.astype(np.uint8)); s.h = _lib.fpp_trk_create(s._occ.ctypes.data, occ.shape[1], occ.shape[0], res)
    def update(s, x, y, rr, A, dt):
        rr = np.ascontiguousarray(rr, dtype=np.float64); A = np.ascontiguousarray(A, dtype=np.float64); out = np.zeros((64, 8))
        n = _lib.fpp_trk_update(s.h, x, y, rr.ctypes.data, A.ctypes.data, len(rr), dt, out.ctypes.data, 64)
        return [dict(x=o[0], y=o[1], vx=o[2], vy=o[3], r=o[4], age=int(o[5]), miss=int(o[6]), id=int(o[7])) for o in out[:n]]
    def __del__(s):
        try: _lib.fpp_trk_destroy(s.h)
        except Exception: pass
