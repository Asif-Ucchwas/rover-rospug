#pragma once
// fpp/planners.hpp: the planners. Each returns waypoints (x, y); path.hpp turns them into poses.
#include "fpp/world.hpp"

namespace fpp {
// 0/1: point A* on a grid inflated by `radius` (circumscribed / inscribed radius).
bool plan_point(World& w, double sx, double sy, double gx, double gy, double radius, Waypoints& wp, Stats& st);
// 2: Theta* with rectangle line of sight and rotate-in-place feasibility.
bool plan_theta(World& w, double sx, double sy, double sth, double gx, double gy, const double* prm, Waypoints& wp, Stats& st);
// 3: SE(2) lattice A* (forward, reverse, rotate in place), holonomic obstacle-aware heuristic.
bool plan_se2(World& w, double sx, double sy, double sth, double gx, double gy, const double* prm, Waypoints& wp, Stats& st);
// 4/5: RRT*, plain or restricted to a corridor around a seed path.
bool plan_rrt(World& w, double sx, double sy, double gx, double gy, const double* prm, const Waypoints* seed, Waypoints& wp, Stats& st);
}  // namespace fpp
