#pragma once
// fpp/common.hpp: shared constants, small types and timing helpers.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace fpp {
constexpr double PI = 3.14159265358979323846;
constexpr double CHK = 0.5 * 3.14159265358979323846 / 180.0;  // rotation collision-check step (0.5 deg: rectangle corner moves < 6 mm)
struct P3 { double x, y, th; };
using Waypoints = std::vector<std::pair<double, double>>;
inline double wrap(double a) { while (a > PI) a -= 2 * PI; while (a <= -PI) a += 2 * PI; return a; }
struct Stats { double expanded = 0, secs = 0, checks = 0; };
using Clock = std::chrono::steady_clock;
inline double since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }
}  // namespace fpp
