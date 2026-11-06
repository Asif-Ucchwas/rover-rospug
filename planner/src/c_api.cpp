// c_api.cpp: plain C API over the planner (used from Python via ctypes, and by the tests).
#include <limits>
#include "fpp/planners.hpp"
#include "fpp/path.hpp"

using namespace fpp;

extern "C" {
void* fpp_create(const uint8_t* occ, int W, int H, double res, double a, double b, double margin) {
  World* w = new World(); w->W = W; w->H = H; w->res = res; w->a = a; w->b = b; w->margin = margin; w->occ.assign(occ, occ + (size_t)W * H);
  w->pre.assign((size_t)(W + 1) * (H + 1), 0);
  for (int j = 0; j < H; j++) for (int i = 0; i < W; i++) w->pre[(j + 1) * (W + 1) + i + 1] = w->occ[j * W + i] + w->pre[j * (W + 1) + i + 1] + w->pre[(j + 1) * (W + 1) + i] - w->pre[j * (W + 1) + i];
  build_edt(*w); return w;
}
void fpp_set_cost(void* p, const uint8_t* cost, double weight) { World* w = (World*)p; if (cost) w->cost.assign(cost, cost + (size_t)w->W * w->H); else w->cost.clear(); w->wc = weight; }
void fpp_destroy(void* p) { delete (World*)p; }
int fpp_rect_hits(void* p, double x, double y, double th) { return ((World*)p)->rect_hits(x, y, th) ? 1 : 0; }
int fpp_sweep_hits(void* p, double x0, double y0, double x1, double y1, double th) { return ((World*)p)->sweep_hits(x0, y0, x1, y1, th) ? 1 : 0; }
int fpp_rot_dir(void* p, double x, double y, double th0, double th1) { return ((World*)p)->rot_dir(x, y, th0, th1, CHK); }
double fpp_clearance(void* p, double x, double y) { return ((World*)p)->clearance(x, y); }
void fpp_edt(void* p, float* out) { World* w = (World*)p; for (int k = 0; k < w->W * w->H; k++) out[k] = std::max(0.f, (float)(w->edt[k] * w->res - 0.5 * w->res)); }

// Independent validator: dense pose sampling (step_xy metres, step_rot rad), point-in-cell tests on the
// rectangle's own sampled boundary-free area: tests every cell centre... uses SAT per pose (not the swept hull).
// Returns number of colliding sampled poses (0 = valid).
int fpp_validate(void* p, const double* path, int n, double step_xy, double step_rot) {
  World* w = (World*)p; int bad = 0;
  for (int k = 0; k < n; k++) {
    double x = path[3 * k], y = path[3 * k + 1], th = path[3 * k + 2];
    if (w->rect_hits(x, y, th)) bad++;
    if (k + 1 < n) { double x1 = path[3 * (k + 1)], y1 = path[3 * (k + 1) + 1], th1 = path[3 * (k + 1) + 2];
      double L = std::hypot(x1 - x, y1 - y), dth = std::fabs(wrap(th1 - th)); int m = std::max((int)std::ceil(L / step_xy), (int)std::ceil(dth / step_rot)); m = std::max(m, 1);
      for (int q = 1; q < m; q++) { double t = (double)q / m; if (w->rect_hits(x + (x1 - x) * t, y + (y1 - y) * t, th + wrap(th1 - th) * t)) bad++; } }
  }
  return bad;
}

// method: 0 point A* (circumscribed), 1 point A* (inscribed), 2 Theta*, 3 SE(2) A*; tight=1 applies shortcut.
// prm[0..8] = se2 res, NT, step, w_rot, rev_penalty, goal_tol, max_nodes, time_limit, unused
// out: capacity maxn poses (x,y,th); stats: [expanded, seconds, collision_checks]. Returns pose count or -1.
int fpp_plan(void* p, int method, int tight, double sx, double sy, double sth, double gx, double gy, double gth,
             const double* prm, double* out, int maxn, double* stats) {
  World* w = (World*)p; Stats st; auto t0 = Clock::now(); long c0 = w->checks; Waypoints wp; bool ok = false; stats[3] = 0.0;
  if (method == 0) ok = plan_point(*w, sx, sy, gx, gy, std::hypot(w->a, w->b) + w->margin, wp, st);
  else if (method == 1) ok = plan_point(*w, sx, sy, gx, gy, w->b + w->margin, wp, st);
  else if (method == 2) ok = plan_theta(*w, sx, sy, sth, gx, gy, prm, wp, st);
  else if (method == 3) ok = plan_se2(*w, sx, sy, sth, gx, gy, prm, wp, st);
  else if (method == 4) ok = plan_rrt(*w, sx, sy, gx, gy, prm, nullptr, wp, st);
  else if (method == 5 || method == 6) {   // corridor RRT* seeded by the SE(2) lattice path; 6 = with seed fallback
    Waypoints seedwp; Stats s0; bool sok = plan_se2(*w, sx, sy, sth, gx, gy, prm, seedwp, s0); st.expanded += s0.expanded;
    if (sok) { ok = plan_rrt(*w, sx, sy, gx, gy, prm, &seedwp, wp, st);
      if (ok) { std::vector<P3> tmp; auto wt = tight ? tighten(*w, wp, sth) : wp; if (!build_poses(*w, wt, sth, std::numeric_limits<double>::quiet_NaN(), tmp)) ok = false; }
      if (!ok && method == 6) { wp = seedwp; ok = true; stats[3] = 1; } } }
  if (ok && tight) wp = tighten(*w, wp, sth);
  std::vector<P3> poses; int n = -1;
  if (ok) {
    if (method <= 1) { // build poses without rotation feasibility (the point planners ignore it); mark by sampled direct rotation
      poses.clear(); double th = sth; poses.push_back({wp[0].first, wp[0].second, th}); double cx = wp[0].first, cy = wp[0].second;
      for (size_t i = 1; i < wp.size(); i++) { double seg = std::atan2(wp[i].second - cy, wp[i].first - cx); double d = wrap(seg - th); double segh = std::fabs(d) > PI / 2 ? wrap(seg + PI) : seg;
        double dd = wrap(segh - th); int m = std::max(1, (int)std::ceil(std::fabs(dd) / (2 * PI / 180)));
        for (int q = 1; q <= m; q++) poses.push_back({cx, cy, wrap(th + dd * q / m)}); th = segh; cx = wp[i].first; cy = wp[i].second; poses.push_back({cx, cy, th}); }
    } else if (!build_poses(*w, wp, sth, std::numeric_limits<double>::quiet_NaN(), poses)) poses.clear();
    n = (int)poses.size(); if (n > maxn) n = -2; else for (int k = 0; k < n; k++) { out[3 * k] = poses[k].x; out[3 * k + 1] = poses[k].y; out[3 * k + 2] = poses[k].th; }
    if (n == 0) n = -1;
  }
  if (method != 6) stats[3] = (method == 6) ? stats[3] : 0.0; stats[0] = st.expanded; stats[1] = since(t0); stats[2] = (double)(w->checks - c0); (void)gth; return n;
}
}

