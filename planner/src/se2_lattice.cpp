// se2_lattice.cpp: Planner 3: SE(2) lattice A*.
#include <limits>
#include <queue>
#include <random>
#include "fpp/planners.hpp"
#include "fpp/path.hpp"

namespace fpp {

bool plan_se2(World& w, double sx, double sy, double sth, double gx, double gy, const double* prm, Waypoints& wp, Stats& st) {
  double rs = prm[0]; int NT = (int)prm[1]; double step = prm[2], wrot = prm[3], revp = prm[4], gtol = prm[5]; double maxn = prm[6], tlim = prm[7];
  auto t0 = Clock::now(); int W = w.W, H = w.H; int W2 = (int)std::ceil(W * w.res / rs), H2 = (int)std::ceil(H * w.res / rs);
  // heuristic: holonomic obstacle-aware distance to goal over cells where the inscribed circle fits
  std::vector<float> hf((size_t)W * H, 1e30f);
  {
    using QE = std::pair<float, int>; std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
    auto ok = [&](int i, int j) { return i >= 0 && j >= 0 && i < W && j < H && w.edt[j * W + i] * w.res - 0.5 * w.res >= w.b + w.margin - 0.5 * w.res - 1e-9; };
    int gi = (int)(gx / w.res), gj = (int)(gy / w.res); if (!ok(gi, gj)) return false; hf[gj * W + gi] = 0; pq.push({0, gj * W + gi});
    while (!pq.empty()) { auto [d, k] = pq.top(); pq.pop(); if (d > hf[k]) continue; int i = k % W, j = k / W;
      for (int dj = -1; dj <= 1; dj++) for (int di = -1; di <= 1; di++) { if (!di && !dj) continue; int ni = i + di, nj = j + dj; if (!ok(ni, nj)) continue;
        float nd = d + (float)(w.res * ((di && dj) ? std::sqrt(2.0) : 1.0)); if (nd < hf[nj * W + ni]) { hf[nj * W + ni] = nd; pq.push({nd, nj * W + ni}); } } }
  }
  auto H0 = [&](double x, double y) { int i = (int)(x / w.res), j = (int)(y / w.res); if (i < 0 || j < 0 || i >= W || j >= H) return 1e30f; return hf[j * W + i]; };
  struct Node { double x, y, th; float g; int par; int it; };
  std::vector<Node> nodes; nodes.reserve(1 << 16);
  std::vector<int> best((size_t)W2 * H2 * NT, -1); std::vector<uint8_t> closed;
  auto key = [&](double x, double y, int it) { int i = (int)(x / rs), j = (int)(y / rs); if (i < 0 || j < 0 || i >= W2 || j >= H2) return (size_t)-1; return ((size_t)j * W2 + i) * NT + it; };
  auto bin = [&](double th) { int it = (int)std::lround(wrap(th) / (2 * PI / NT)); it %= NT; if (it < 0) it += NT; return it; };
  if (H0(sx, sy) > 1e29f) return false;
  using QE = std::pair<float, int>; std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  int it0 = bin(sth); double th0 = it0 * (2 * PI / NT);
  if (w.rect_hits(sx, sy, th0)) { // start heading bin collides: try all bins
    bool found = false; for (int q = 0; q < NT && !found; q++) if (!w.rect_hits(sx, sy, q * (2 * PI / NT))) { it0 = q; th0 = q * (2 * PI / NT); found = true; }
    if (!found) return false; }
  nodes.push_back({sx, sy, th0, 0.f, -1, it0}); closed.push_back(0); best[key(sx, sy, it0)] = 0; pq.push({H0(sx, sy), 0});
  int goal_node = -1; double dth = 2 * PI / NT;
  while (!pq.empty()) {
    if ((long)st.expanded % 256 == 0 && since(t0) > tlim) break; if (st.expanded > maxn) break;
    auto [f, k] = pq.top(); pq.pop(); if (closed[k]) continue; closed[k] = 1; st.expanded++; Node cur = nodes[k];
    if (std::hypot(cur.x - gx, cur.y - gy) <= gtol) { goal_node = k; break; }
    struct Act { double x, y, th; double c; bool rot; int it; };
    Act acts[4]; int na = 0;
    double ca = std::cos(cur.th), sa = std::sin(cur.th);
    double sstep = std::min(step, std::hypot(cur.x - gx, cur.y - gy)); if (sstep < 0.02) sstep = std::min(step, 0.02);
    double cf = 1.0, cb = 1.0;
    if (w.wc > 0 && !w.cost.empty()) { cf = w.seg_cost(cur.x, cur.y, cur.x + sstep * ca, cur.y + sstep * sa) / sstep; cb = w.seg_cost(cur.x, cur.y, cur.x - sstep * ca, cur.y - sstep * sa) / sstep; }
    acts[na++] = {cur.x + sstep * ca, cur.y + sstep * sa, cur.th, sstep * cf, false, cur.it};
    acts[na++] = {cur.x - sstep * ca, cur.y - sstep * sa, cur.th, sstep * revp * cb, false, cur.it};
    acts[na++] = {cur.x, cur.y, wrap(cur.th + dth), wrot * dth, true, (cur.it + 1) % NT};
    acts[na++] = {cur.x, cur.y, wrap(cur.th - dth), wrot * dth, true, (cur.it + NT - 1) % NT};
    for (int q = 0; q < na; q++) { Act& A = acts[q];
      if (A.rot) { if (w.rot_hits(cur.x, cur.y, cur.th, A.th, (q == 2) ? 1 : -1, CHK)) continue; }
      else if (w.sweep_hits(cur.x, cur.y, A.x, A.y, cur.th)) continue;
      size_t kk = key(A.x, A.y, A.it); if (kk == (size_t)-1) continue; float ng = cur.g + (float)A.c; int bi = best[kk];
      if (bi >= 0 && nodes[bi].g <= ng) continue;
      nodes.push_back({A.x, A.y, A.th, ng, k, A.it}); closed.push_back(0); int id = (int)nodes.size() - 1; best[kk] = id;
      float h = H0(A.x, A.y); if (h > 1e29f) continue; pq.push({ng + h, id});
    }
  }
  if (goal_node < 0) return false;
  Waypoints rev; int k = goal_node; double lx = 1e18, ly = 1e18, lth = 1e18;
  // keep only turning points of the translation heading (and start/end)
  std::vector<int> chain; while (k >= 0) { chain.push_back(k); k = nodes[k].par; } std::reverse(chain.begin(), chain.end());
  wp.clear(); wp.push_back({sx, sy});
  for (size_t q = 1; q + 1 < chain.size(); q++) {
    const Node &a = nodes[chain[q - 1]], &b = nodes[chain[q]], &c = nodes[chain[q + 1]];
    bool mv1 = std::hypot(b.x - a.x, b.y - a.y) > 1e-9, mv2 = std::hypot(c.x - b.x, c.y - b.y) > 1e-9;
    if (mv1 && mv2) { double a1 = std::atan2(b.y - a.y, b.x - a.x), a2 = std::atan2(c.y - b.y, c.x - b.x); if (std::fabs(wrap(a2 - a1)) > 1e-6) wp.push_back({b.x, b.y}); }
    else if (mv1 != mv2) wp.push_back({b.x, b.y});
  }
  wp.push_back({nodes[chain.back()].x, nodes[chain.back()].y});
  Waypoints dd; for (auto& p : wp) if (dd.empty() || std::hypot(p.first - dd.back().first, p.second - dd.back().second) > 1e-9) dd.push_back(p);
  wp = dd; (void)lx; (void)ly; (void)lth; return true;
}

}  // namespace fpp
