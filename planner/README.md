# fpp: footprint-aware planning core (simulation only, stage 3 of the Rover + ROSPug syllabus)

ROS-free C++17 library (plain C API) plus a Python harness. Robot: 1100 x 770 mm rectangle (MEGA).

Build: `g++ -O2 -std=c++17 -shared -fPIC src/fpp.cpp -o libfpp.so`

Run (from this folder):
- `python3 py/test_collision.py 0.004`   task 3.3 check: swept-rectangle collision vs brute force
- `python3 py/exp_gap.py`                door-gap sweep (results_gap.json)
- `python3 py/exp_corridor.py`           80 queries in the corridor world (results_corridor.json, about 10 min)
- `python3 py/figs.py` and `python3 py/anim.py`   figures and the driving video

Planners (`py/fpp.py`, method names): `point_circ` point A* on a map inflated by the circumscribed radius (stock-style),
`point_insc` inflated by the inscribed radius, `theta_rect` Theta* with exact rectangle line of sight and rotate-in-place checks,
`se2` SE(2) lattice A* (forward, reverse, rotate in place) with the same checks. `tight=True` adds a shortcut pass under the same rule.

## What is measured so far
Collision checker (task 3.3): 10,000 random pose queries and 1,500 sweep queries against dense sampling, 0 missed collisions
(a handful of sub-millimetre contacts the sampler cannot see). Door gaps: stock-style circle planning needs about 1.35 m;
inscribed-circle planning returns paths that clip the wall for gaps of 0.80 to 0.85 m; rectangle-aware planners find collision-free
paths down to about 0.80 m (SE(2)) and 0.90 m (Theta*). Corridor world, 80 queries, 60 of them reachable by at least one planner
(plan margin 2 cm, validated on the true footprint at 5 mm / 0.25 deg): point_circ 1/60, point_insc 60 returned but 19 collision-free,
theta_rect 34/60 (all valid), se2 51/60 (all valid), se2 with a finer lattice 56/60. Median planning time 0.33 to 0.41 s (not yet optimised).

Decision 3.2 (draft, see docs/DECISION_3.2.md): plain and corridor RRT* with the exact rectangle rule solve 19 to 24 of the 60 trips; corridor RRT* with a fallback to the lattice path matches the lattice planner (51 of 60) and is at most 2.5% shorter. Run `python3 py/exp_rrt.py`.

## Not done yet (honest list)
costmap-aware cost terms (3.4); GoogleTest suite, Nav2 plugin, latency work (3.5 to 3.9);
the world uses placeholder dimensions until the real corridor is surveyed; Theta* is incomplete under rotation constraints (34/60 vs 51/60 for SE(2)).

## 3.4 Costmap-aware costs (done, simulation)
Nav2-style inflation costmap (radius 1.0 m, scaling 6) feeds the SE(2) step cost and the shortcut step (`Planner.set_cost`). 50 queries solved by all three settings, every path valid at margin 0:

| setting | valid | mean length (m) | mean min clearance (m) | mean path cost | planning time (s) |
|---|---|---|---|---|---|
| no cost | 50/50 | 17.83 | 0.465 | 107 | 0.28 |
| weight 1 | 50/50 | 18.33 | 0.543 | 47 | 0.68 |
| weight 3 | 50/50 | 18.86 | 0.589 | 29 | 0.86 |

Trade-off: about 3–6% longer, 0.08–0.12 m more clearance, 2.5–3x slower (latency work is task 3.9). Script: `py/exp_cost.py`, data: `results_cost.json`.

## 3.6 Tests and build (done)
CMake builds the library and a GoogleTest suite (`tests/test_fpp.cpp`, 9 tests: collision, swept volume, clearance, door passing, in-place rotation, narrow-door rejection, costmap). GoogleTest is fetched from GitHub on first configure.
```
cmake -S . -B build && cmake --build build -j && ctest --test-dir build --output-on-failure
```

## Code map (one file per piece)
| File | What it holds |
|---|---|
| `include/fpp/common.hpp` | constants, `P3` pose, angle wrap, timing |
| `include/fpp/world.hpp` | occupancy grid, exact rectangle collision (SAT), swept volume, in-place rotation check, costmap |
| `src/world.cpp` | exact distance transform (Felzenszwalb) |
| `include/fpp/path.hpp`, `src/path.cpp` | waypoints to pose path, shortcut ("tighten") pass |
| `include/fpp/planners.hpp` | the four planner entry points |
| `src/point_astar.cpp` | point A* baselines (circumscribed / inscribed inflation) |
| `src/theta_star.cpp` | Theta* with rectangle line of sight |
| `src/se2_lattice.cpp` | SE(2) lattice A* (main planner) |
| `src/rrt_star.cpp` | RRT* and corridor RRT* |
| `src/c_api.cpp` | plain C API (`fpp_plan`, `fpp_validate`, ...) used by Python and tests |
| `tests/test_fpp.cpp` | GoogleTest suite |
| `py/fpp.py` | Python wrapper (loads `build/libfpp.so`) |
| `py/worlds.py`, `py/exp_*.py`, `py/figs.py`, `py/viz.py`, `py/anim.py`, `py/mkvideo.py` | test worlds, experiments, figures, videos |

Build: `cmake -S . -B build && cmake --build build -j` (older one-line `g++` build no longer applies).
