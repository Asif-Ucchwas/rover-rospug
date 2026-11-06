// path.cpp: build_poses and tighten (shortcut pass).
#include <limits>
#include <random>
#include "fpp/path.hpp"

namespace fpp {

// Waypoints (xy) -> pose path with in-place rotations (<= dstep) between segments.
bool build_poses(World& w, const Waypoints& wp, double th_start, double th_goal_or_nan,
                 std::vector<P3>& out, double dstep) {
  out.clear(); double th = th_start; double cx = wp[0].first, cy = wp[0].second; out.push_back({cx, cy, th});
  auto rotate = [&](double th1) -> bool {
    int dir = w.rot_dir(cx, cy, th, th1, CHK); if (!dir) return false;
    double d = th1 - th; if (std::fabs(wrap(d)) < 1e-9) { th = wrap(th1); return true; }
    if (dir > 0) { while (d < 0) d += 2 * PI; } else { while (d > 0) d -= 2 * PI; }
    int n = std::max(1, (int)std::ceil(std::fabs(d) / dstep));
    for (int k = 1; k <= n; k++) out.push_back({cx, cy, wrap(th + d * k / n)});
    th = wrap(th1); return true;
  };
  for (size_t i = 1; i < wp.size(); i++) {
    double dx = wp[i].first - cx, dy = wp[i].second - cy; if (std::hypot(dx, dy) < 1e-9) continue;
    double seg = std::atan2(dy, dx);
    // rectangle is symmetric: either end may lead, so pick the smaller rotation (<= 90 deg)
    double d = wrap(seg - th); double segh = seg; if (std::fabs(d) > PI / 2) segh = wrap(seg + PI);
    if (!rotate(segh)) return false;
    cx = wp[i].first; cy = wp[i].second; out.push_back({cx, cy, th});
  }
  if (!std::isnan(th_goal_or_nan)) { if (!rotate(th_goal_or_nan)) return false; }
  return true;
}

// Greedy shortcut with the same exact rule; heading_in tracks required rotations.
Waypoints tighten(World& w, const Waypoints& wp, double th_start) {
  if (wp.size() < 3) return wp;
  Waypoints out{wp[0]}; size_t i = 0; double th_in = th_start;
  while (i + 1 < wp.size()) {
    size_t best = i + 1;
    for (size_t j = wp.size() - 1; j > i + 1; j--) {
      double dx = wp[j].first - wp[i].first, dy = wp[j].second - wp[i].second; double seg = std::atan2(dy, dx);
      double d = wrap(seg - th_in); double segh = std::fabs(d) > PI / 2 ? wrap(seg + PI) : seg;
      if (w.sweep_hits(wp[i].first, wp[i].second, wp[j].first, wp[j].second, segh)) continue;
      if (!w.rot_dir(wp[i].first, wp[i].second, th_in, segh, CHK)) continue;
      if (w.wc > 0 && !w.cost.empty()) {   // keep the shortcut only if it is not costlier than the chain it replaces
        double old_c = 0; for (size_t q = i; q < j; q++) old_c += w.seg_cost(wp[q].first, wp[q].second, wp[q + 1].first, wp[q + 1].second);
        if (w.seg_cost(wp[i].first, wp[i].second, wp[j].first, wp[j].second) > old_c + 1e-6) continue; }
      best = j; break;
    }
    double dx = wp[best].first - wp[i].first, dy = wp[best].second - wp[i].second; double seg = std::atan2(dy, dx);
    double d = wrap(seg - th_in); th_in = std::fabs(d) > PI / 2 ? wrap(seg + PI) : seg;
    out.push_back(wp[best]); i = best;
  }
  return out;
}

}  // namespace fpp
