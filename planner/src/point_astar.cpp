// point_astar.cpp: Planners 0/1: point A* on an inflated grid.
#include <limits>
#include <queue>
#include <random>
#include "fpp/planners.hpp"
#include "fpp/path.hpp"

namespace fpp {

bool plan_point(World& w, double sx, double sy, double gx, double gy, double radius, Waypoints& wp, Stats& st) {
  int W = w.W, H = w.H; auto ok = [&](int i, int j) { return i >= 0 && j >= 0 && i < W && j < H && w.edt[j * W + i] * w.res - 0.5 * w.res >= radius - 1e-9; };
  int si = (int)(sx / w.res), sj = (int)(sy / w.res), gi = (int)(gx / w.res), gj = (int)(gy / w.res);
  if (!ok(si, sj) || !ok(gi, gj)) return false;
  std::vector<float> g((size_t)W * H, 1e30f); std::vector<int> par((size_t)W * H, -1);
  using QE = std::pair<float, int>; std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  auto h = [&](int i, int j) { double dx = std::abs(i - gi), dy = std::abs(j - gj); return (float)((dx + dy) + (std::sqrt(2.0) - 2) * std::min(dx, dy)) * (float)w.res; };
  g[sj * W + si] = 0; pq.push({h(si, sj), sj * W + si});
  while (!pq.empty()) {
    auto [f, k] = pq.top(); pq.pop(); int i = k % W, j = k / W; st.expanded++;
    if (f > g[k] + h(i, j) + 1e-6f) continue;
    if (i == gi && j == gj) break;
    for (int dj = -1; dj <= 1; dj++) for (int di = -1; di <= 1; di++) {
      if (!di && !dj) continue; int ni = i + di, nj = j + dj; if (!ok(ni, nj)) continue;
      if (di && dj && (!ok(i + di, j) || !ok(i, j + dj))) continue;
      float ng = g[k] + (float)(w.res * ((di && dj) ? std::sqrt(2.0) : 1.0)); int nk = nj * W + ni;
      if (ng < g[nk]) { g[nk] = ng; par[nk] = k; pq.push({ng + h(ni, nj), nk}); }
    }
  }
  if (par[gj * W + gi] < 0 && !(si == gi && sj == gj)) return false;
  Waypoints rev; int k = gj * W + gi;
  while (k >= 0) { rev.push_back({(k % W + 0.5) * w.res, (k / W + 0.5) * w.res}); k = par[k]; }
  std::reverse(rev.begin(), rev.end()); rev.front() = {sx, sy}; rev.back() = {gx, gy};
  // drop collinear points
  wp.clear(); wp.push_back(rev[0]);
  for (size_t q = 1; q + 1 < rev.size(); q++) {
    double a1 = std::atan2(rev[q].second - rev[q - 1].second, rev[q].first - rev[q - 1].first);
    double a2 = std::atan2(rev[q + 1].second - rev[q].second, rev[q + 1].first - rev[q].first);
    if (std::fabs(wrap(a2 - a1)) > 1e-6) wp.push_back(rev[q]);
  }
  wp.push_back(rev.back()); return true;
}

}  // namespace fpp
