// dynamic.cpp: people tracker and speed governor (port of py/dyn.py).
#include <algorithm>
#include "fpp/dynamic.hpp"

namespace fpp {

double rect_circle_dist(double x, double y, double th, double a, double b, double cx, double cy) {
  double c = std::cos(th), s = std::sin(th), dx = cx - x, dy = cy - y;
  double u = dx * c + dy * s, v = -dx * s + dy * c;
  return std::hypot(std::max(std::fabs(u) - a, 0.0), std::max(std::fabs(v) - b, 0.0));
}

// ---------------- governor ----------------
double Governor::obj_radius(const Track& t, double tt, double keepout) const {
  double r = t.r + keepout; if (t.age < 4) r += std::min(1.0, 0.5 * tt); return r;   // velocity not known yet: grow the uncertainty instead
}
void Governor::predict(const Track& t, double tt, double& cx, double& cy) const {
  if (t.age < 4) { cx = t.x; cy = t.y; return; }
  cx = t.x + t.vx * tt; cy = t.y + t.vy * tt;
}
double Governor::free_distance(const std::vector<Seg>& segs, int k, double x, double y, double th, double v, const std::vector<Track>& tr, double keepout, double look) const {
  const double a = p_.a + p_.margin, b = p_.b + p_.margin; const double vref = std::max(v, 0.4); double acc = 0.0, tt = 0.0;
  auto hit = [&](double px, double py, double pth, double t) {
    for (const Track& q : tr) { double cx, cy; predict(q, t, cx, cy); if (rect_circle_dist(px, py, pth, a, b, cx, cy) < obj_radius(q, t, keepout)) return true; }
    return false; };
  if (hit(x, y, th, 0.0)) return 0.0;
  for (int q = k; q < (int)segs.size(); q++) {
    const Seg& sg = segs[q];
    if (sg.rot) {
      int n = std::max(1, (int)(std::fabs(sg.d) / 0.087));
      double rem = (q > k) ? sg.d : wrap(sg.th1 - th);
      for (int m = 1; m <= n; m++) {
        double hth = (q == k) ? th + rem * m / n : sg.th0 + sg.d * m / n;
        if (hit(sg.x0, sg.y0, hth, tt + std::fabs(rem) * m / n / p_.w_max)) return acc;
      }
      tt += std::fabs(rem) / p_.w_max; th = sg.th1;
    } else {
      double lx = (q == k) ? x : sg.x0, ly = (q == k) ? y : sg.y0; double L = std::hypot(sg.x1 - lx, sg.y1 - ly); int n = std::max(1, (int)(L / 0.1));
      for (int m = 1; m <= n; m++) {
        double px = lx + (sg.x1 - lx) * m / n, py = ly + (sg.y1 - ly) * m / n;
        if (hit(px, py, sg.th, tt + (acc + L * m / n) / vref)) return acc + L * (m - 1) / n;
      }
      acc += L;
    }
    if (acc > look) break;
  }
  return INF;
}
double Governor::speed_limit(double d_free) const {
  if (d_free == INF) return p_.v_max;
  double d = d_free - p_.buf; if (d <= 0) return 0.0;
  double a = p_.a_brake, tr = p_.t_react; return std::min(p_.v_max, a * (-tr + std::sqrt(tr * tr + 2 * d / a)));
}
double Governor::proximity_limit(double x, double y, double th, const std::vector<Track>& tr) const {
  double dmin = 9.0; for (const Track& t : tr) dmin = std::min(dmin, rect_circle_dist(x, y, th, p_.a, p_.b, t.x, t.y) - t.r);
  return p_.v_max * (0.3 + 0.7 * std::min(1.0, std::max(0.0, (dmin - 0.4) / 1.6)));
}
double Governor::allowed_speed(const std::vector<Seg>& segs, int k, double x, double y, double th, double v, const std::vector<Track>& tr, double* d_free) const {
  double d1 = free_distance(segs, k, x, y, th, v, tr, p_.keepout); double lim = speed_limit(d1);
  if (d1 != INF) {
    double d2 = free_distance(segs, k, x, y, th, v, tr, p_.keepout_min);
    lim = std::max(lim, std::min(p_.creep, speed_limit(d2)));
    if (lim > 0 && d1 < p_.buf + 0.05) d1 = p_.buf + 0.06;
  }
  if (d_free) *d_free = d1; return lim;
}

// ---------------- tracker ----------------
Tracker::Tracker(const uint8_t* occ, int W, int H, double res, double rng) : W_(W), H_(H), res_(res), rng_(rng), occd_(occ, occ + (size_t)W * H) {
  std::vector<uint8_t> src(occd_);   // 5 x 5 dilation: the map explains hits within 10 cm of a wall
  for (int j = 0; j < H; j++) for (int i = 0; i < W; i++) { uint8_t o = 0;
    for (int dj = -2; dj <= 2 && !o; dj++) for (int di = -2; di <= 2; di++) { int ii = (i + di + W) % W, jj = (j + dj + H) % H; if (src[(size_t)jj * W + ii]) { o = 1; break; } }   // wraps like numpy roll
    occd_[(size_t)j * W + i] = o; }
}
std::vector<Track> Tracker::update(double x, double y, const double* rr, const double* A, int n, double dt) {
  std::vector<double> px(n), py(n); std::vector<int> idx;
  for (int i = 0; i < n; i++) {
    px[i] = x + rr[i] * std::cos(A[i]); py[i] = y + rr[i] * std::sin(A[i]);
    int ci = (int)std::floor(px[i] / res_), cj = (int)std::floor(py[i] / res_);
    bool ok = rr[i] < rng_ - 0.3, inb = ci >= 0 && cj >= 0 && ci < W_ && cj < H_;
    if (ok && inb && occd_[(size_t)cj * W_ + ci] == 0) idx.push_back(i);
  }
  std::vector<std::vector<int>> clusters;
  if (!idx.empty()) {
    std::vector<int> cur{idx[0]};
    for (size_t q = 1; q < idx.size(); q++) { int kk = idx[q];
      if (kk == cur.back() + 1 && std::hypot(px[kk] - px[cur.back()], py[kk] - py[cur.back()]) < 0.25) cur.push_back(kk);
      else { clusters.push_back(cur); cur = {kk}; } }
    clusters.push_back(cur);
  }
  struct Det { double x, y, r; }; std::vector<Det> dets;
  for (auto& c : clusters) {
    if (c.size() < 3) continue;
    double cx = 0, cy = 0, mnx = 1e18, mxx = -1e18, mny = 1e18, mxy = -1e18;
    for (int i : c) { cx += px[i]; cy += py[i]; mnx = std::min(mnx, px[i]); mxx = std::max(mxx, px[i]); mny = std::min(mny, py[i]); mxy = std::max(mxy, py[i]); }
    cx /= c.size(); cy /= c.size(); double br = std::atan2(cy - y, cx - x); cx += 0.18 * std::cos(br); cy += 0.18 * std::sin(br);   // visible surface -> body centre
    double ext = std::hypot(mxx - mnx, mxy - mny); dets.push_back({cx, cy, std::max(PED_R, 0.5 * ext + 0.05)});
  }
  std::vector<char> used(dets.size(), 0);
  for (Track& t : tracks_) {
    t.miss++; int best = -1; double bd = 0.9;
    for (size_t k = 0; k < dets.size(); k++) { if (used[k]) continue;
      double d = std::hypot(dets[k].x - (t.x + t.vx * dt), dets[k].y - (t.y + t.vy * dt)); if (d < bd) { best = (int)k; bd = d; } }
    if (best >= 0) { used[best] = 1; const Det& d = dets[best]; double vx = (d.x - t.x) / dt, vy = (d.y - t.y) / dt, al = 0.15;
      t.vx = (1 - al) * t.vx + al * vx; t.vy = (1 - al) * t.vy + al * vy; t.x = d.x; t.y = d.y; t.r = d.r; t.age++; t.miss = 0; }
  }
  for (size_t k = 0; k < dets.size(); k++) if (!used[k]) { Track t; t.x = dets[k].x; t.y = dets[k].y; t.r = dets[k].r; t.id = nid_++; tracks_.push_back(t); }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [](const Track& t) { return t.miss > 10; }), tracks_.end());
  std::vector<Track> out; for (const Track& t : tracks_) if (t.miss <= 2 && t.age >= 2) out.push_back(t);
  return out;
}
}  // namespace fpp
