#pragma once
// fpp/world.hpp: occupancy grid, exact rectangle collision (SAT + prefix sums), swept volume,
// in-place rotation checks, distance transform, optional costmap.
#include <algorithm>
#include "fpp/common.hpp"

namespace fpp {
struct World {
  int W, H; double res, a, b, margin;
  std::vector<uint8_t> occ;
  std::vector<int> pre;       // (W+1)*(H+1) prefix sums of occupied cells
  std::vector<float> edt;     // distance (cells) from each cell centre to nearest occupied centre
  long checks = 0;
  std::vector<uint8_t> cost; double wc = 0.0;   // optional costmap (0 free .. 252 near obstacles, 253 inscribed, 254 lethal) and its weight
  double cost_at(double x, double y) const { int i = (int)std::floor(x / res), j = (int)std::floor(y / res); if (cost.empty() || i < 0 || j < 0 || i >= W || j >= H) return 254.0; return cost[(size_t)j * W + i]; }
  double seg_cost(double x0, double y0, double x1, double y1) const {   // length weighted by (1 + wc * cost / 252), sampled every 5 cm
    double L = std::hypot(x1 - x0, y1 - y0); if (L < 1e-9) return 0; int n = std::max(1, (int)std::ceil(L / 0.05)); double acc = 0;
    for (int k = 0; k < n; k++) { double t = (k + 0.5) / n; acc += (L / n) * (1.0 + wc * std::min(cost_at(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t), 252.0) / 252.0); } return acc; }

  int cnt(int i0, int j0, int i1, int j1) const {  // inclusive cell box, clipped
    i0 = std::max(i0, 0); j0 = std::max(j0, 0); i1 = std::min(i1, W - 1); j1 = std::min(j1, H - 1);
    if (i0 > i1 || j0 > j1) return 0;
    auto P = [&](int i, int j) { return pre[j * (W + 1) + i]; };
    return P(i1 + 1, j1 + 1) - P(i0, j1 + 1) - P(i1 + 1, j0) + P(i0, j0);
  }
  double clearance(double x, double y) const {  // distance to nearest occupied cell surface (approx)
    int i = (int)std::floor(x / res), j = (int)std::floor(y / res);
    if (i < 0 || j < 0 || i >= W || j >= H) return 0.0;
    double d = edt[j * W + i] * res - 0.5 * res;
    return d < 0 ? 0 : d;
  }
  // Convex polygon (CCW or CW) against occupied cells (cells are closed squares; touching = free).
  bool poly_hits(const double* vx, const double* vy, int n) const {
    double minx = 1e18, maxx = -1e18, miny = 1e18, maxy = -1e18;
    for (int k = 0; k < n; k++) { minx = std::min(minx, vx[k]); maxx = std::max(maxx, vx[k]); miny = std::min(miny, vy[k]); maxy = std::max(maxy, vy[k]); }
    int i0 = (int)std::floor(minx / res), i1 = (int)std::floor(maxx / res);
    int j0 = (int)std::floor(miny / res), j1 = (int)std::floor(maxy / res);
    if (i0 < 0 || j0 < 0 || i1 >= W || j1 >= H) return true;  // outside the map = blocked
    if (cnt(i0, j0, i1, j1) == 0) return false;
    double nx[16], ny[16], lo[16], hi[16];
    for (int k = 0; k < n; k++) {
      int k2 = (k + 1) % n; double ex = vx[k2] - vx[k], ey = vy[k2] - vy[k];
      double l = std::hypot(ex, ey); if (l < 1e-12) { nx[k] = 1; ny[k] = 0; } else { nx[k] = -ey / l; ny[k] = ex / l; }
      double mn = 1e18, mx = -1e18;
      for (int m = 0; m < n; m++) { double d = nx[k] * vx[m] + ny[k] * vy[m]; mn = std::min(mn, d); mx = std::max(mx, d); }
      lo[k] = mn; hi[k] = mx;
    }
    for (int j = j0; j <= j1; j++) for (int i = i0; i <= i1; i++) {
      if (!occ[j * W + i]) continue;
      double cx0 = i * res, cx1 = (i + 1) * res, cy0 = j * res, cy1 = (j + 1) * res;
      bool sep = false;
      for (int k = 0; k < n && !sep; k++) {  // polygon edge normals vs cell projection
        double c = nx[k] * (cx0 + cx1) * 0.5 + ny[k] * (cy0 + cy1) * 0.5;
        double r = (std::fabs(nx[k]) + std::fabs(ny[k])) * res * 0.5;
        if (c + r <= lo[k] || c - r >= hi[k]) sep = true;
      }
      if (!sep) return true;  // cell axes already satisfied by bbox pre-selection
    }
    return false;
  }
  bool rect_hits(double x, double y, double th) { checks++;
    double ca = std::cos(th), sa = std::sin(th), A = a + margin, B = b + margin;
    double vx[4], vy[4]; const double sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, -1, 1};
    for (int k = 0; k < 4; k++) { vx[k] = x + sx[k] * A * ca - sy[k] * B * sa; vy[k] = y + sx[k] * A * sa + sy[k] * B * ca; }
    return poly_hits(vx, vy, 4);
  }
  // Exact swept volume of the rectangle translating from (x0,y0) to (x1,y1) at heading th.
  bool sweep_hits(double x0, double y0, double x1, double y1, double th) {
    double L = std::hypot(x1 - x0, y1 - y0);
    int pieces = std::max(1, (int)std::ceil(L / 0.5));
    for (int p = 0; p < pieces; p++) {
      double t0 = (double)p / pieces, t1 = (double)(p + 1) / pieces;
      double ax = x0 + (x1 - x0) * t0, ay = y0 + (y1 - y0) * t0, bx = x0 + (x1 - x0) * t1, by = y0 + (y1 - y0) * t1;
      checks++;
      double ca = std::cos(th), sa = std::sin(th), A = a + margin, B = b + margin;
      double px[8], py[8]; int m = 0; const double sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, -1, 1};
      for (int k = 0; k < 4; k++) { double ox = sx[k] * A * ca - sy[k] * B * sa, oy = sx[k] * A * sa + sy[k] * B * ca;
        px[m] = ax + ox; py[m++] = ay + oy; px[m] = bx + ox; py[m++] = by + oy; }
      // convex hull (monotone chain)
      std::vector<int> id(m); for (int k = 0; k < m; k++) id[k] = k;
      std::sort(id.begin(), id.end(), [&](int u, int v) { return px[u] < px[v] || (px[u] == px[v] && py[u] < py[v]); });
      auto cr = [&](int o, int u, int v) { return (px[u] - px[o]) * (py[v] - py[o]) - (py[u] - py[o]) * (px[v] - px[o]); };
      std::vector<int> h(2 * m); int k = 0;
      for (int q = 0; q < m; q++) { while (k >= 2 && cr(h[k - 2], h[k - 1], id[q]) <= 1e-12) k--; h[k++] = id[q]; }
      for (int q = m - 2, t = k + 1; q >= 0; q--) { while (k >= t && cr(h[k - 2], h[k - 1], id[q]) <= 1e-12) k--; h[k++] = id[q]; }
      k--; double hx[16], hy[16]; for (int q = 0; q < k && q < 16; q++) { hx[q] = px[h[q]]; hy[q] = py[h[q]]; }
      if (poly_hits(hx, hy, std::min(k, 16))) return true;
    }
    return false;
  }
  // In-place rotation th0 -> th1 going in direction dir (+1 ccw / -1 cw), poses every dstep rad.
  bool rot_hits(double x, double y, double th0, double th1, int dir, double dstep) {
    double d = th1 - th0; if (std::fabs(wrap(d)) < 1e-9) d = 0;
    else if (dir > 0) { while (d < 0) d += 2 * PI; while (d > 2 * PI) d -= 2 * PI; }
    else { while (d > 0) d -= 2 * PI; while (d < -2 * PI) d += 2 * PI; }
    if (clearance(x, y) >= std::hypot(a, b) + margin + res) return false;  // circumscribed circle fits
    int n = std::max(1, (int)std::ceil(std::fabs(d) / dstep));
    for (int k = 0; k <= n; k++) if (rect_hits(x, y, th0 + d * k / n)) return true;
    return false;
  }
  // returns 0 if no collision-free direction, else +1 / -1 (shorter arc preferred)
  int rot_dir(double x, double y, double th0, double th1, double dstep) {
    double d = wrap(th1 - th0); if (std::fabs(d) < 1e-9) return rect_hits(x, y, th0) ? 0 : 1;
    int first = d > 0 ? 1 : -1;
    if (!rot_hits(x, y, th0, th1, first, dstep)) return first;
    if (!rot_hits(x, y, th0, th1, -first, dstep)) return -first;
    return 0;
  }
};

void build_edt(World& w);  // exact Euclidean distance transform (Felzenszwalb); fills w.edt

}  // namespace fpp
