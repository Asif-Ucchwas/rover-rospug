#include <gtest/gtest.h>
#include "fpp/dynamic.hpp"
using namespace fpp;

namespace {
Seg line(double x0, double y0, double x1, double y1, double th) { Seg s; s.rot = 0; s.x0 = x0; s.y0 = y0; s.x1 = x1; s.y1 = y1; s.th = th; return s; }
Track person(double x, double y, double vx = 0, double vy = 0) { Track t; t.x = x; t.y = y; t.vx = vx; t.vy = vy; t.r = 0.3; t.age = 10; return t; }
}

TEST(Governor, ClearPathAllowsFullSpeed) {
  Governor g{GovParams{}}; std::vector<Seg> segs{line(0, 0, 20, 0, 0)}; double d;
  EXPECT_DOUBLE_EQ(g.allowed_speed(segs, 0, 0, 0, 0, 0.5, {}, &d), 0.8); EXPECT_EQ(d, INF);
}
TEST(Governor, PersonBesideThePathDoesNotStopTheRover) {
  Governor g{GovParams{}}; std::vector<Seg> segs{line(0, 0, 20, 0, 0)}; double d;
  EXPECT_GT(g.allowed_speed(segs, 0, 0, 0, 0, 0.5, {person(5, 3.0)}, &d), 0.7);
}
TEST(Governor, StoppingDistanceAlwaysFitsInTheFreeDistance) {   // the safety property: from the allowed speed it can stop before the keep-out
  GovParams p; Governor g(p); std::vector<Seg> segs{line(0, 0, 20, 0, 0)};
  for (double px = 1.6; px < 9.0; px += 0.37) {
    double d; double v = g.allowed_speed(segs, 0, 0, 0, 0, 0.0, {person(px, 0.0)}, &d);
    if (d == INF) continue;
    if (v <= p.creep + 1e-9) continue;     // creeping past a person is the separate, tighter-keep-out case (tested below)
    double stop = v * p.t_react + v * v / (2 * p.a_brake);
    EXPECT_LE(stop, d - p.buf + 1e-9) << "person at x=" << px;
  }
}
TEST(Governor, PersonRightInFrontMeansStop) {
  Governor g{GovParams{}}; std::vector<Seg> segs{line(0, 0, 20, 0, 0)}; double d;
  EXPECT_EQ(g.allowed_speed(segs, 0, 0, 0, 0, 0.8, {person(0.9, 0.0)}, &d), 0.0);
}
TEST(Governor, WalkerCrossingTheRouteIsPredicted) {   // not in the way now, but will be when we arrive
  Governor g{GovParams{}}; std::vector<Seg> segs{line(0, 0, 20, 0, 0)}; double d;
  double v_cross = g.allowed_speed(segs, 0, 0, 0, 0, 0.8, {person(4.0, 4.4, 0.0, -1.0)}, &d);
  EXPECT_NE(d, INF); EXPECT_GT(d, 2.5); EXPECT_LT(d, 4.5);      // conflict predicted about 3.5 m ahead (where the walker will be), not yet close enough to slow down
  EXPECT_DOUBLE_EQ(v_cross, 0.8);
  double d2; EXPECT_LT(g.allowed_speed(segs, 0, 2.6, 0, 0, 0.8, {person(4.0, 1.9, 0.0, -1.0)}, &d2), 0.8);   // closer: now it must slow
}
TEST(Governor, CreepsPastAPersonWhenComfortKeepoutIsNotFree) {
  GovParams p; Governor g(p); std::vector<Seg> segs{line(0, 0, 20, 0, 0)}; double d;
  // person 1.0 m to the side: 0.5 m from the padded footprint, inside the comfort keep-out (0.6 m), outside the minimum one (0.42 m)
  double v = g.allowed_speed(segs, 0, 4.0, 0, 0, 0.0, {person(5, 1.0)}, &d);   // robot 1 m behind the person, level with it a moment later
  EXPECT_LE(v, p.creep + 1e-9); EXPECT_GT(v, 0.0);
}
TEST(Governor, TurnInPlaceIsBlockedByAPersonNearby) {
  Governor g{GovParams{}}; Seg r; r.rot = 1; r.x0 = 0; r.y0 = 0; r.th0 = 0; r.th1 = 1.57; r.d = 1.57;
  double d; EXPECT_EQ(g.allowed_speed({r}, 0, 0, 0, 0, 0.0, {person(0.9, 0.5)}, &d), 0.0);
}
TEST(Tracker, FindsAStandingPersonInAnEmptyHall) {
  const int W = 200, H = 100; std::vector<uint8_t> occ((size_t)W * H, 0);
  for (int i = 0; i < W; i++) { occ[i] = 1; occ[(size_t)(H - 1) * W + i] = 1; }
  Tracker tr(occ.data(), W, H, 0.05); const int n = 720; std::vector<double> rr(n, 12.0), A(n);
  double rx = 2.0, ry = 2.5, px = 6.0, py = 2.5;
  for (int i = 0; i < n; i++) { A[i] = -PI + 2 * PI * i / n; double c = std::cos(A[i]), s = std::sin(A[i]);
    double dx = px - rx, dy = py - ry, proj = dx * c + dy * s, perp2 = dx * dx + dy * dy - proj * proj;
    if (proj > 0 && perp2 < PED_R * PED_R) rr[i] = proj - std::sqrt(PED_R * PED_R - perp2); }
  std::vector<Track> out; for (int k = 0; k < 5; k++) out = tr.update(rx, ry, rr.data(), A.data(), n, 0.05);
  ASSERT_EQ(out.size(), 1u); EXPECT_NEAR(out[0].x, px, 0.15); EXPECT_NEAR(out[0].y, py, 0.15); EXPECT_LT(std::hypot(out[0].vx, out[0].vy), 0.1);
}
TEST(Tracker, WallsAloneAreNotPeople) {
  const int W = 200, H = 100; std::vector<uint8_t> occ((size_t)W * H, 0);
  for (int i = 0; i < W; i++) { occ[i] = 1; occ[(size_t)(H - 1) * W + i] = 1; }
  Tracker tr(occ.data(), W, H, 0.05); const int n = 720; std::vector<double> rr(n), A(n);
  for (int i = 0; i < n; i++) { A[i] = -PI + 2 * PI * i / n; double s = std::sin(A[i]); rr[i] = std::fabs(s) < 1e-3 ? 12.0 : std::min(12.0, (s > 0 ? 4.95 - 2.5 : 2.5 - 0.05) / std::fabs(s)); }
  for (int k = 0; k < 5; k++) EXPECT_TRUE(tr.update(2.0, 2.5, rr.data(), A.data(), n, 0.05).empty());
}
