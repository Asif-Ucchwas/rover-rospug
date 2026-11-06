// theta_star.cpp: Planner 2: Theta* with rectangle line of sight.
#include <limits>
#include <queue>
#include <random>
#include "fpp/planners.hpp"
#include "fpp/path.hpp"

namespace fpp {

bool plan_theta(World& w, double sx, double sy, double sth, double gx, double gy, const double* prm, Waypoints& wp, Stats& st) {
  int W = w.W, H = w.H; double tlim = prm[7]; auto t0 = Clock::now();
  auto ok = [&](int i, int j) { return i >= 0 && j >= 0 && i < W && j < H && w.edt[j * W + i] * w.res - 0.5 * w.res >= w.b + w.margin - 0.5 * w.res - 1e-9; };
  int si = (int)(sx / w.res), sj = (int)(sy / w.res), gi = (int)(gx / w.res), gj = (int)(gy / w.res);
  if (!ok(si, sj) || !ok(gi, gj)) return false;
  const double RSTEP = CHK;
  std::vector<float> g((size_t)W * H, 1e30f); std::vector<int> par((size_t)W * H, -1); std::vector<uint8_t> closed((size_t)W * H, 0);
  auto cx = [&](int k) { return (k % W + 0.5) * w.res; }; auto cy = [&](int k) { return (k / W + 0.5) * w.res; };
  auto hd = [&](int k) { return std::hypot(cx(k) - gx, cy(k) - gy); };
  auto heading_in = [&](int k) { if (par[k] < 0 || par[k] == k) return sth; double d = std::atan2(cy(k) - cy(par[k]), cx(k) - cx(par[k])); return d; };
  auto seg_ok = [&](int p, int n) { // line of sight p->n incl. rotation at p from its incoming heading
    double dx = cx(n) - cx(p), dy = cy(n) - cy(p); double seg = std::atan2(dy, dx);
    double hin = (p == sj * W + si) ? sth : heading_in(p); double d = wrap(seg - hin); double segh = std::fabs(d) > PI / 2 ? wrap(seg + PI) : seg;
    if (w.sweep_hits(cx(p), cy(p), cx(n), cy(n), segh)) return false;
    return w.rot_dir(cx(p), cy(p), hin, segh, RSTEP) != 0;
  };
  using QE = std::pair<float, int>; std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  int sk = sj * W + si, gk = gj * W + gi; g[sk] = 0; par[sk] = sk; pq.push({(float)hd(sk), sk});
  while (!pq.empty()) {
    if ((st.expanded > 0) && ((long)st.expanded % 256 == 0) && since(t0) > tlim) break;
    auto [f, k] = pq.top(); pq.pop(); if (closed[k]) continue; closed[k] = 1; st.expanded++;
    if (k == gk) break; int i = k % W, j = k / W;
    for (int dj = -1; dj <= 1; dj++) for (int di = -1; di <= 1; di++) {
      if (!di && !dj) continue; int ni = i + di, nj = j + dj; if (!ok(ni, nj)) continue; int nk = nj * W + ni; if (closed[nk]) continue;
      int p = par[k]; bool via_parent = (p >= 0 && p != k) && seg_ok(p, nk);
      float ng; int np;
      if (via_parent) { np = p; ng = g[p] + (float)std::hypot(cx(nk) - cx(p), cy(nk) - cy(p)); }
      else { if (!seg_ok(k, nk)) continue; np = k; ng = g[k] + (float)std::hypot(cx(nk) - cx(k), cy(nk) - cy(k)); }
      if (ng < g[nk]) { g[nk] = ng; par[nk] = np; pq.push({ng + (float)hd(nk), nk}); }
    }
  }
  if (par[gk] < 0) return false;
  Waypoints rev; int k = gk; int guard = 0;
  while (guard++ < W * H) { rev.push_back({cx(k), cy(k)}); if (par[k] == k) break; k = par[k]; }
  std::reverse(rev.begin(), rev.end()); rev.front() = {sx, sy}; rev.back() = {gx, gy};
  wp = rev; return true;
}

}  // namespace fpp
