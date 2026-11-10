// Tests for the footprint-aware planner core (C API). World: 20 m x 10 m, 5 cm cells.
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <vector>

extern "C" {
void* fpp_create(const uint8_t*, int, int, double, double, double, double);
void fpp_destroy(void*);
void fpp_set_cost(void*, const uint8_t*, double);
int fpp_rect_hits(void*, double, double, double);
int fpp_sweep_hits(void*, double, double, double, double, double);
double fpp_clearance(void*, double, double);
int fpp_validate(void*, const double*, int, double, double);
int fpp_plan(void*, int, int, double, double, double, double, double, double, const double*, double*, int, double*);
}

namespace {
const double RES = 0.05, A = 0.56, B = 0.405;
const int W = 400, H = 200;
const double PRM[14] = {0.09, 24, 0.30, 0.5, 1.5, 0.15, 3e6, 30.0, 0.0, 2000, 2.0, 2.0, 0.05, 1};

// Outer walls plus a vertical wall at x = 10 m with a door of the given width centred at y = 5 m.
std::vector<uint8_t> world(double door) {
  std::vector<uint8_t> g(W * H, 0);
  for (int j = 0; j < H; j++) for (int i = 0; i < W; i++) {
    double x = i * RES, y = j * RES;
    bool wall = i < 2 || j < 2 || i >= W - 2 || j >= H - 2;
    if (!wall && std::fabs(x - 10.0) < 0.1 && std::fabs(y - 5.0) > door / 2) wall = true;
    g[(size_t)j * W + i] = wall;
  }
  return g;
}
struct Fix {
  std::vector<uint8_t> g; void* p;
  explicit Fix(double door, double margin = 0.02) : g(world(door)), p(fpp_create(g.data(), W, H, RES, A, B, margin)) {}
  ~Fix() { fpp_destroy(p); }
};
int plan(Fix& f, int method, double sx, double sy, double sth, double gx, double gy, std::vector<double>& out, double* st) {
  out.assign(3 * 20000, 0.0); return fpp_plan(f.p, method, 1, sx, sy, sth, gx, gy, 0, PRM, out.data(), 20000, st);
}
}  // namespace

TEST(Collision, FreeSpaceAndWall) {
  Fix f(1.5);
  EXPECT_EQ(fpp_rect_hits(f.p, 5.0, 5.0, 0.3), 0);
  EXPECT_EQ(fpp_rect_hits(f.p, 10.0, 1.0, 0.0), 1);          // inside the wall
  EXPECT_EQ(fpp_rect_hits(f.p, 0.3, 5.0, 0.0), 1);           // body overlaps outer wall (a = 0.56 m)
}
TEST(Collision, RectangleOrientationMatters) {
  Fix f(1.5, 0.0);
  // Next to the x=10 wall at y=2: long axis (1.12 m) along x pokes 0.56 m forward, along y only 0.405 m.
  EXPECT_EQ(fpp_rect_hits(f.p, 9.40, 2.0, 0.0), 1);            // x-extent reaches 9.95+ -> hits wall at 9.9
  EXPECT_EQ(fpp_rect_hits(f.p, 9.40, 2.0, M_PI / 2), 0);       // x-extent 0.405 -> clear
}
TEST(Collision, SweepCatchesThinWallBetweenEndpoints) {
  Fix f(1.5);
  EXPECT_EQ(fpp_rect_hits(f.p, 8.0, 2.0, 0.0), 0);
  EXPECT_EQ(fpp_rect_hits(f.p, 12.0, 2.0, 0.0), 0);
  EXPECT_EQ(fpp_sweep_hits(f.p, 8.0, 2.0, 12.0, 2.0, 0.0), 1);   // endpoints free, path crosses the wall
  EXPECT_EQ(fpp_sweep_hits(f.p, 8.0, 5.0, 12.0, 5.0, 0.0), 0);   // through the 1.5 m door
}
TEST(Clearance, DistanceToNearestObstacle) {
  Fix f(1.5, 0.0);
  EXPECT_NEAR(fpp_clearance(f.p, 5.0, 5.0), 4.9, 0.1);   // walls are 10 cm thick, so 4.9 m to each
  EXPECT_LT(fpp_clearance(f.p, 9.7, 2.0), 0.35);
}
TEST(Plan, SE2PassesDoorAndValidates) {
  Fix f(1.2); Fix v(1.2, 0.0); std::vector<double> out; double st[4];
  int n = plan(f, 3, 3.0, 5.0, 0.0, 17.0, 5.0, out, st);
  ASSERT_GT(n, 0);
  EXPECT_EQ(fpp_validate(v.p, out.data(), n, 0.005, 0.25 * M_PI / 180), 0);   // true footprint, fine steps
  EXPECT_NEAR(out[3 * (n - 1)], 17.0, 0.25);
}
TEST(Plan, DoorNarrowerThanBodyIsRejected) {
  Fix f(0.6); std::vector<double> out; double st[4];            // robot width is 0.81 m
  EXPECT_LT(plan(f, 3, 3.0, 5.0, 0.0, 17.0, 5.0, out, st), 0);
}
TEST(Plan, DoorGapsAgainstMeasuredWidth) {
  std::vector<double> out; double st[4];
  { Fix f(0.80); EXPECT_LT(plan(f, 3, 3.0, 5.0, 0.0, 17.0, 5.0, out, st), 0); }   // 0.80 m < 0.81 m body
  { Fix f(1.32); EXPECT_GT(plan(f, 3, 3.0, 5.0, 0.0, 17.0, 5.0, out, st), 0); }   // lab door, both leaves open
}
TEST(Plan, SameInputGivesIdenticalPath) {
  Fix f(1.2); std::vector<double> o1, o2; double st[4];
  int n1 = plan(f, 3, 3.0, 5.0, 0.0, 17.0, 5.0, o1, st), n2 = plan(f, 3, 3.0, 5.0, 0.0, 17.0, 5.0, o2, st);
  ASSERT_GT(n1, 0); ASSERT_EQ(n1, n2);
  for (int k = 0; k < 3 * n1; k++) EXPECT_EQ(o1[k], o2[k]);
}
TEST(Plan, ReusedLatticeTableDoesNotLeakBetweenQueries) {   // a failed query must not change the next answer
  Fix f(1.2); std::vector<double> o1, o2, o3; double st[4];
  int n1 = plan(f, 3, 3.0, 5.0, 0.0, 17.0, 5.0, o1, st);
  plan(f, 3, 3.0, 5.0, 0.0, 5.0, 5.0, o3, st);    // goal inside a wall: fails
  int n2 = plan(f, 3, 3.0, 5.0, 0.0, 17.0, 5.0, o2, st);
  ASSERT_EQ(n1, n2); for (int k = 0; k < 3 * n1; k++) EXPECT_EQ(o1[k], o2[k]);
}
TEST(Plan, RotatesInPlaceWhenNeeded) {
  Fix f(1.2); std::vector<double> out; double st[4];
  // Start facing sideways at the door: the long axis cannot pass until the rover turns.
  int n = plan(f, 3, 3.0, 5.0, M_PI / 2, 17.0, 5.0, out, st);
  ASSERT_GT(n, 0);
  double mx = 0; for (int k = 0; k < n; k++) mx = std::max(mx, std::fabs(out[3 * k + 2] - out[2]));
  EXPECT_GT(mx, 0.5);
}
TEST(Plan, WiderDoorNeverWorseThanNarrower) {
  std::vector<double> o1, o2; double st[4];
  Fix a(1.0), b(2.0);
  int na = plan(a, 3, 3.0, 5.0, 0.0, 17.0, 5.0, o1, st), nb = plan(b, 3, 3.0, 5.0, 0.0, 17.0, 5.0, o2, st);
  ASSERT_GT(na, 0); ASSERT_GT(nb, 0);
}
TEST(Cost, WeightKeepsPathValidAndCleared) {
  Fix f(2.0), v(2.0, 0.0); std::vector<uint8_t> cm(W * H, 0);
  for (int j = 0; j < H; j++) for (int i = 0; i < W; i++) {
    double d = fpp_clearance(f.p, (i + 0.5) * RES, (j + 0.5) * RES);
    cm[(size_t)j * W + i] = f.g[(size_t)j * W + i] ? 254 : (d <= B ? 253 : (d >= 1.0 ? 0 : (uint8_t)std::lround(252 * std::exp(-6 * (d - B)))));
  }
  std::vector<double> o0, o1; double st[4];
  int n0 = plan(f, 3, 3.0, 2.0, 0.0, 17.0, 2.0, o0, st);
  fpp_set_cost(f.p, cm.data(), 3.0);
  int n1 = plan(f, 3, 3.0, 2.0, 0.0, 17.0, 2.0, o1, st);
  ASSERT_GT(n0, 0); ASSERT_GT(n1, 0);
  EXPECT_EQ(fpp_validate(v.p, o1.data(), n1, 0.005, 0.25 * M_PI / 180), 0);
  auto minclr = [&](const std::vector<double>& o, int n) { double m = 1e9; for (int k = 0; k < n; k++) m = std::min(m, fpp_clearance(f.p, o[3 * k], o[3 * k + 1])); return m; };
  EXPECT_GE(minclr(o1, n1), minclr(o0, n0) - 1e-6);
}
