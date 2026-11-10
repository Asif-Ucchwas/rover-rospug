# Planner specification (task 3.1), draft 4, November 8, 2026

Purpose: one page that says what the planner must do, so tests can say pass or fail. Targets marked "proposed" are my starting values. Values marked "(find out)" need a measurement or a fact from you.

## 1. Robot
| Item | Value | Source |
|---|---|---|
| Footprint | rectangle 1120 x 810 mm (measured), centered on the planning point | your notes |
| Drive | skid-steer, can rotate in place | (find out: confirm on the real rover) |
| Max speed, max turn rate | planning placeholder 0.8 m/s and 0.8 rad/s (no MEGA datasheet number found; read it off the rover manual or measure in task 2.2) | placeholder |
| Where the planning point sits on the body | footprint center | you confirmed |

## 2. Inputs and outputs
- Input: a 2D occupancy grid (5 cm cells; costmap values later, task 3.4), a start pose (x, y, heading), a goal position (heading optional).
- Output: a list of poses. Between two consecutive poses the robot either drives straight at fixed heading or turns in place. This is the format the Nav2 plugin will convert to.
- Failure: the planner returns no path and says why (no route, time limit, start or goal blocked). It never returns a path it has not checked.

## 3. Collision rule
- The footprint plus a safety margin must not overlap any occupied cell, at any point along the path, including during in-place turns.
- Straight segments are checked exactly (swept rectangle against each cell). In-place turns are checked every 0.5 degrees (the corner moves under 6 mm per step).
- Safety margin: proposed 2 cm for planning. Validation uses the true footprint with no margin.
- Cells outside the map count as blocked.

## 4. Acceptance tests (pass or fail)
| Test | Pass condition | Status |
|---|---|---|
| Collision checker vs brute force, 10,000 poses | 0 missed collisions | passed (simulation) |
| Every returned path, validated on the true footprint at 5 mm and 0.25 degrees | 0 collisions | passed on 80 corridor trips |
| Door gaps | finds a path through gaps of 0.85 m and wider (rover width 0.81 m); gaps of 0.80 m and narrower are correctly blocked, 0.85 m passes with the 2 cm margin | passed (gap sweep, 0.70 to 1.50 m) |
| Success rate on reachable trips (corridor world, 0.81 m width) | proposed at least 85% | 51 of 56 = 91% |
| Latency, 80 corridor queries | proposed under 100 ms median on the rover computer | cloud machine: median 101 ms (was 150 ms), slowest 5 s on unreachable goals; rover computer not yet measured |
| Same input, same output | identical path on repeat runs | passed (unit test, also after a failed query in between) |

## 4b. Pass/fail against the stock planners (the numbers task 3.10 must meet)
Stock planners = NavFn, SmacPlanner2D, SmacPlannerHybrid, ThetaStar, run with the same 1.12 x 0.81 m footprint and the same maps. Paths from a point or circle planner are checked on the true rectangle, so a path that clips a wall counts as a failure.

| Axis | Our planner passes if | Why this number |
|---|---|---|
| Valid paths (0 collisions on the true footprint) | 100% of returned paths | a returned path must be safe, no exceptions |
| Success on reachable trips | at least the best stock planner's rate, and at least 85% | stock circle planners refuse or clip at doors |
| Doors 0.85 to 1.32 m | passes every one; blocks every gap of 0.80 m and narrower | rover is 0.81 m wide, margin 2 cm |
| Path length | within 10% of the shortest valid stock path | no long detours for safety |
| Minimum clearance | at least 2 cm on every path | the planning margin |
| Planning time | median under 100 ms and 95th percentile under 500 ms on the rover computer | replanning around people needs fast answers |
| Repeatability | identical output on identical input | testing and debugging |

Decision gate (from the syllabus): if we beat no stock planner on any axis, the write-up says so and reports the benchmark and plugin as the contribution.

## 4c. Status of the spec itself
Written after the first design experiment (3.2), not before it, because this project started the planner in simulation first. The numbers above were set from the measured footprint and door survey, not from our own results.

## 5. Motion
Forward, reverse and turning in place. Reverse has a cost penalty so the planner prefers forward when it is about equal. Other motion models (for example car-like vehicles) are for later and are not part of this spec.

## 6. Not in scope yet
Doors that open and close, the real speed and acceleration limits (moving people and speed control: see DYNAMIC.md), a costmap cost beyond blocked or free (task 3.4), the Nav2 plugin interface (3.7).

## 7. Open questions for you
1. (answered: footprint center.)
2. What are its max speed and turn rate?
3. Is the 2 cm margin enough for the real localization error? (We learn this in stage 4.)
4. (answered: forward and reverse allowed.)
