#pragma once
// fpp/dynamic.hpp: moving-people layer. Tracker (lidar -> tracks) and Governor (tracks + path -> speed limit).
// ROS-free; mirrors py/dyn.py, and tests/test_dynamic.cpp plus py/test_dyn_cpp.py check that the two agree.
#include <limits>
#include "fpp/common.hpp"

namespace fpp {
constexpr double PED_R = 0.25;
constexpr double INF = std::numeric_limits<double>::infinity();

struct Track { double x = 0, y = 0, vx = 0, vy = 0, r = 0; int age = 1, miss = 0, id = 0; };

// One piece of the path: a straight drive at fixed heading, or an in-place turn.
struct Seg {
  int rot = 0;                         // 0 = line, 1 = turn in place
  double x0 = 0, y0 = 0, x1 = 0, y1 = 0, th = 0;   // line: from (x0,y0) to (x1,y1) at heading th
  double th0 = 0, th1 = 0, d = 0;                  // turn at (x0,y0): from th0 to th1, signed total angle d
};

struct GovParams {
  double a = 0.56, b = 0.405;          // footprint half length / half width (m)
  double v_max = 0.8, w_max = 0.8, a_brake = 1.0, t_react = 0.25;
  double margin = 0.10, keepout = 0.30, buf = 0.25, creep = 0.25, keepout_min = 0.12;
};

double rect_circle_dist(double x, double y, double th, double a, double b, double cx, double cy);

class Governor {
 public:
  explicit Governor(const GovParams& p) : p_(p) {}
  // free distance along the path before the footprint would meet a predicted person (INF if clear)
  double free_distance(const std::vector<Seg>& segs, int k, double x, double y, double th, double v, const std::vector<Track>& tr, double keepout, double look = 7.0) const;
  double speed_limit(double d_free) const;            // highest speed from which it can still stop with `buf` to spare
  double proximity_limit(double x, double y, double th, const std::vector<Track>& tr) const;   // slows near people even when not in the way
  // main entry: returns the speed limit, writes the comfortable-keep-out free distance
  double allowed_speed(const std::vector<Seg>& segs, int k, double x, double y, double th, double v, const std::vector<Track>& tr, double* d_free) const;
  const GovParams& params() const { return p_; }
 private:
  GovParams p_;
  void predict(const Track& t, double tt, double& cx, double& cy) const;
  double obj_radius(const Track& t, double tt, double keepout) const;
};

class Tracker {
 public:
  Tracker(const uint8_t* occ, int W, int H, double res, double rng = 12.0);
  // one lidar scan: ranges rr[i] at world angles A[i] from (x, y). Returns confirmed, recently seen tracks.
  std::vector<Track> update(double x, double y, const double* rr, const double* A, int n, double dt);
 private:
  int W_, H_; double res_, rng_; std::vector<uint8_t> occd_; std::vector<Track> tracks_; int nid_ = 0;
};
}  // namespace fpp
