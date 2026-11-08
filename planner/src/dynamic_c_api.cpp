// dynamic_c_api.cpp: plain C API for the moving-people layer (Python ctypes and tests).
#include "fpp/dynamic.hpp"
using namespace fpp;

namespace {
GovParams gp(const double* p) { GovParams g; g.a = p[0]; g.b = p[1]; g.v_max = p[2]; g.w_max = p[3]; g.a_brake = p[4]; g.t_react = p[5]; g.margin = p[6]; g.keepout = p[7]; g.buf = p[8]; g.creep = p[9]; g.keepout_min = p[10]; return g; }
std::vector<Seg> sg(const double* s, int n) { std::vector<Seg> v(n); for (int i = 0; i < n; i++) { const double* q = s + 9 * i; v[i].rot = (int)q[0]; v[i].x0 = q[1]; v[i].y0 = q[2]; v[i].x1 = q[3]; v[i].y1 = q[4]; v[i].th = q[5]; v[i].th0 = q[6]; v[i].th1 = q[7]; v[i].d = q[8]; } return v; }
std::vector<Track> tk(const double* t, int n) { std::vector<Track> v(n); for (int i = 0; i < n; i++) { const double* q = t + 8 * i; v[i].x = q[0]; v[i].y = q[1]; v[i].vx = q[2]; v[i].vy = q[3]; v[i].r = q[4]; v[i].age = (int)q[5]; v[i].miss = (int)q[6]; v[i].id = (int)q[7]; } return v; }
}

extern "C" {
// prm: a, b, v_max, w_max, a_brake, t_react, margin, keepout, buf, creep, keepout_min.  segs: 9 doubles each.  tracks: 8 doubles each (x y vx vy r age miss id).
double fpp_gov_allowed(const double* prm, const double* segs, int nseg, int k, double x, double y, double th, double v, const double* tracks, int ntr, double* d_free) {
  Governor g(gp(prm)); return g.allowed_speed(sg(segs, nseg), k, x, y, th, v, tk(tracks, ntr), d_free); }
double fpp_gov_free(const double* prm, const double* segs, int nseg, int k, double x, double y, double th, double v, const double* tracks, int ntr, double keepout, double look) {
  Governor g(gp(prm)); return g.free_distance(sg(segs, nseg), k, x, y, th, v, tk(tracks, ntr), keepout, look); }
double fpp_gov_proximity(const double* prm, double x, double y, double th, const double* tracks, int ntr) { Governor g(gp(prm)); return g.proximity_limit(x, y, th, tk(tracks, ntr)); }
double fpp_gov_speed_limit(const double* prm, double d_free) { Governor g(gp(prm)); return g.speed_limit(d_free); }

void* fpp_trk_create(const uint8_t* occ, int W, int H, double res) { return new Tracker(occ, W, H, res); }
void fpp_trk_destroy(void* h) { delete (Tracker*)h; }
int fpp_trk_update(void* h, double x, double y, const double* rr, const double* A, int n, double dt, double* out, int maxout) {
  std::vector<Track> t = ((Tracker*)h)->update(x, y, rr, A, n, dt); int m = std::min((int)t.size(), maxout);
  for (int i = 0; i < m; i++) { double* q = out + 8 * i; q[0] = t[i].x; q[1] = t[i].y; q[2] = t[i].vx; q[3] = t[i].vy; q[4] = t[i].r; q[5] = t[i].age; q[6] = t[i].miss; q[7] = t[i].id; }
  return m; }
}
