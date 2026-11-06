#pragma once
// fpp/path.hpp: waypoints -> pose path with in-place rotations, and the shortcut ("tighten") pass.
#include "fpp/world.hpp"

namespace fpp {
bool build_poses(World& w, const Waypoints& wp, double th_start, double th_goal_or_nan, std::vector<P3>& out,
                 double dstep = 2.0 * PI / 180);
Waypoints tighten(World& w, const Waypoints& wp, double th_start);
}  // namespace fpp
