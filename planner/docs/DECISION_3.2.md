# Decision 3.2 (draft): which planner core goes into the Nav2 plugin

Setup: placeholder corridor world (36 x 14 m, 5 cm cells), 80 random trips, 60 of them solved by at least one planner. Planning margin 2 cm; every route is
re-checked on the true 1100 x 770 mm footprint at 5 mm / 0.25 deg sampling. Plain and corridor RRT* use exact swept-rectangle edges, 2 m corridor half-width,
2 m rewire radius, 5% goal bias, one fixed seed per query.

| Planner | Collision-free routes (of 60) | Median time |
|---|---|---|
| Point A*, circumscribed circle | 1 | under 1 ms |
| Point A*, inscribed circle | 19 (60 returned) | under 1 ms |
| Theta* + rectangle | 34 | about 0.4 s |
| SE(2) lattice A* + rectangle, with shortcut | 51 | about 0.4 s |
| Plain RRT*, 2000 / 4000 iterations | 19 / 23 | 4 / 15 ms |
| Corridor RRT* (seeded by the lattice path), 2000 / 4000 | 24 / 23 | 0.3 to 0.4 s |
| Corridor RRT* with fallback to the seed, 2000 / 4000 | 51 / 51 (fallback used in 28 / 29) | 0.35 to 0.4 s |

Reading: RRT* works in open hallways and fails at narrow doors, because an edge has to cross a 0.85 to 1.1 m gap at almost right angles to a wall and random samples
rarely line up. When corridor RRT* does succeed its routes are about as long as the lattice route (median 0.7% longer, shorter in 5 of 23 by 1.8% median, 2.5% at most).
With the fallback, its success equals the lattice planner's, because the fallback is the lattice path.

Draft decision: the plugin core is the SE(2) lattice planner plus the rectangle shortcut. Corridor RRT* is not worth carrying unless a later test shows a meaningful length gain
(the data here show at most 2.5%). This matches Paper A, where the thesis planner did not beat Theta* with tightening.

Limits: one seeded world, placeholder dimensions, one RRT* setting each, no door-aware sampling, no costmap terms. Re-run on the surveyed Cherry ground floor before the decision is final.
