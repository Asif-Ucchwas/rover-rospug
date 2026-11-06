// rrt_star.cpp: Planners 4/5: RRT* with exact rectangle edges.
#include <limits>
#include <queue>
#include <random>
#include "fpp/planners.hpp"
#include "fpp/path.hpp"

namespace fpp {

bool plan_rrt(World& w, double sx, double sy, double gx, double gy, const double* prm, const Waypoints* seed,
              Waypoints& wp, Stats& st) {
  int iters = (int)prm[9]; double cw = prm[10], rr = prm[11], gb = prm[12]; unsigned long long sd = (unsigned long long)prm[13];
  std::mt19937_64 rng(sd * 1000003ULL + (unsigned long long)(sx * 1000) * 7919ULL + (unsigned long long)(gy * 1000));
  std::uniform_real_distribution<double> U(0, 1); const double STEP = 1.0; double tlim = prm[7]; auto t0 = Clock::now();
  struct N { double x, y; int par; double c; }; std::vector<N> T; T.reserve(iters + 2); T.push_back({sx, sy, -1, 0});
  std::vector<double> cum; if (seed) { cum.push_back(0); for (size_t i = 1; i < seed->size(); i++) cum.push_back(cum.back() + std::hypot((*seed)[i].first - (*seed)[i - 1].first, (*seed)[i].second - (*seed)[i - 1].second)); }
  auto edge_ok = [&](double x0, double y0, double x1, double y1) { return !w.sweep_hits(x0, y0, x1, y1, std::atan2(y1 - y0, x1 - x0)); };
  int best_goal = -1; double best_c = 1e18; double W_m = w.W * w.res, H_m = w.H * w.res;
  for (int it = 0; it < iters; it++) {
    if ((it & 63) == 0 && since(t0) > tlim) break; st.expanded++;
    double qx, qy;
    if (U(rng) < gb) { qx = gx; qy = gy; }
    else if (seed) { double u = U(rng) * cum.back(); size_t k = std::upper_bound(cum.begin(), cum.end(), u) - cum.begin(); if (k >= seed->size()) k = seed->size() - 1; if (k == 0) k = 1;
      double f = (u - cum[k - 1]) / std::max(cum[k] - cum[k - 1], 1e-9); double px = (*seed)[k - 1].first + f * ((*seed)[k].first - (*seed)[k - 1].first), py = (*seed)[k - 1].second + f * ((*seed)[k].second - (*seed)[k - 1].second);
      double rad = cw * std::sqrt(U(rng)), an = 2 * PI * U(rng); qx = px + rad * std::cos(an); qy = py + rad * std::sin(an); }
    else { qx = U(rng) * W_m; qy = U(rng) * H_m; }
    if (w.clearance(qx, qy) < w.b + w.margin - 0.5 * w.res) continue;
    int ni = 0; double bd = 1e18; for (size_t k = 0; k < T.size(); k++) { double d = std::hypot(T[k].x - qx, T[k].y - qy); if (d < bd) { bd = d; ni = (int)k; } }
    double nx = qx, ny = qy; if (bd > STEP) { nx = T[ni].x + (qx - T[ni].x) * STEP / bd; ny = T[ni].y + (qy - T[ni].y) * STEP / bd; }
    if (w.clearance(nx, ny) < w.b + w.margin - 0.5 * w.res) continue;
    std::vector<int> near; for (size_t k = 0; k < T.size(); k++) if (std::hypot(T[k].x - nx, T[k].y - ny) <= rr) near.push_back((int)k);
    int bp = -1; double bc = 1e18;
    for (int k : near) { double c = T[k].c + std::hypot(T[k].x - nx, T[k].y - ny); if (c < bc && edge_ok(T[k].x, T[k].y, nx, ny)) { bc = c; bp = k; } }
    if (bp < 0) continue;
    T.push_back({nx, ny, bp, bc}); int id = (int)T.size() - 1;
    for (int k : near) { if (k == bp) continue; double c = bc + std::hypot(T[k].x - nx, T[k].y - ny); if (c + 1e-9 < T[k].c && edge_ok(nx, ny, T[k].x, T[k].y)) { double dc = c - T[k].c; T[k].par = id; T[k].c = c; (void)dc; } }
    // propagate cost changes lazily: recompute costs of descendants at the end
    if (std::hypot(nx - gx, ny - gy) <= rr && edge_ok(nx, ny, gx, gy)) { double c = bc + std::hypot(nx - gx, ny - gy); if (c < best_c) { best_c = c; best_goal = id; } }
  }
  if (best_goal < 0) return false;
  Waypoints rev{{gx, gy}}; int k = best_goal; int guard = 0;
  while (k >= 0 && guard++ < (int)T.size() + 2) { rev.push_back({T[k].x, T[k].y}); k = T[k].par; }
  if (guard > (int)T.size() + 1) return false;
  std::reverse(rev.begin(), rev.end()); rev.front() = {sx, sy}; wp = rev; return true;
}

}  // namespace fpp
