// world.cpp: distance transform for the occupancy grid.
#include "fpp/world.hpp"

namespace fpp {
void build_edt(World& w) {  // exact Euclidean distance transform (Felzenszwalb), squared cell units
  const int W = w.W, H = w.H; const float INF = 1e12f;
  std::vector<float> f((size_t)W * H);
  for (int k = 0; k < W * H; k++) f[k] = w.occ[k] ? 0.f : INF;
  auto dt1 = [&](std::vector<float>& d, int n, std::vector<float>& out) {
    std::vector<int> v(n); std::vector<float> z(n + 1); int k = 0; v[0] = 0; z[0] = -INF; z[1] = INF;
    for (int q = 1; q < n; q++) {
      float s;
      while (true) { int p = v[k]; s = ((d[q] + (float)q * q) - (d[p] + (float)p * p)) / (2.f * q - 2.f * p);
        if (s <= z[k]) { k--; if (k < 0) { k = 0; break; } } else break; }
      k++; v[k] = q; z[k] = s; z[k + 1] = INF;
    }
    k = 0; for (int q = 0; q < n; q++) { while (z[k + 1] < q) k++; out[q] = (float)(q - v[k]) * (q - v[k]) + d[v[k]]; }
  };
  std::vector<float> col(H), colo(H), row(W), rowo(W);
  for (int x = 0; x < W; x++) { for (int y = 0; y < H; y++) col[y] = f[y * W + x]; dt1(col, H, colo); for (int y = 0; y < H; y++) f[y * W + x] = colo[y]; }
  for (int y = 0; y < H; y++) { for (int x = 0; x < W; x++) row[x] = f[y * W + x]; dt1(row, W, rowo); for (int x = 0; x < W; x++) f[y * W + x] = rowo[x]; }
  w.edt.resize((size_t)W * H);
  for (int k = 0; k < W * H; k++) w.edt[k] = std::sqrt(std::min(f[k], 1e10f));
}

}  // namespace fpp
