# Moving people, speed control and stop/resume (simulation), Nov 8, 2026

Code: `py/dyn.py` (sensing, tracking, speed governor, replanning, 2D simulator), `py/exp_dynamic.py` (experiments). Python for now; a C++ port comes once the behavior is settled.

## What the robot does
1. **Sees people.** A simulated 360 degree lidar (720 beams, 12 m, 2 cm noise). Hits that the static map does not explain are moving or new objects; they are clustered and tracked, and each track gets a velocity. The governor never reads the simulator's walker positions.
2. **Predicts.** Constant velocity. A track younger than 4 scans has no trusted velocity, so its radius grows with time instead.
3. **Governs speed.** Walks the remaining path in 0.1 m steps; the first point where the footprint (+10 cm) would meet a predicted person (+30 cm keep-out) gives the free distance `d`. Speed limit = the highest speed from which it can still stop with 25 cm to spare, given a 0.25 s reaction time and the braking deceleration. Also slows near people even when not in the way (down to 30 percent of top speed at 0.4 m). When the comfortable 30 cm is not free it may still creep at 0.25 m/s as long as 12 cm is free.
4. **Brakes, stops, resumes.** Normal braking at `a_brake`; emergency braking at `a_emerg` when a person appears inside the stopping distance. Speed ramps back up by itself once the path is clear.
5. **Waits, then replans, then reports.** After 4 s without progress it replans around the people it sees (and only accepts a detour the governor itself would drive). After 40 s without progress it stops and reports "blocked".

All speeds and decelerations are placeholders (rover: 0.8 m/s, 1.0 / 2.0 m/s^2 brake / emergency; dog: 0.2 m/s) until measured on the real machines.

## Results (simulation; one person at a time unless noted)
| Test | Result |
|---|---|
| Person steps out of a doorway, 0.8 to 5 m ahead, 1.4 m/s, rover at 0.8 m/s | 0 collisions in all 8; closest gap 0.25 m at 0.8 m ahead; arrives at the goal each time |
| Same with the dog (0.2 m/s) | 0 collisions in all 8; closest gap 0.29 m |
| Person standing in the path, 4.5 m corridor | rover and dog both go around it and arrive |
| Person standing in the path, 3.0 m corridor | rover waits, tries, then reports "blocked" (not enough room to pass at a safe distance); dog passes |
| No people, 14 corridor trips | 14 of 14 arrive, 0 phantom detections, 0 false stops |
| 14 trips with 2 to 6 walkers on the whole floor | 0 collisions caused by the robot moving. In 8 of the 28 runs with walkers a simulated walker stepped into the stopped robot (flaw in the walker model, see below). Many runs end "blocked": the floor was packed with up to 6 walkers |

## Known limits
- The simulated walkers are crude: they stop when the robot is close, sidestep after 1.5 s, and then can step into a robot that is standing still. Real people behave differently, so this is a first sanity check, not proof of safety.
- Dense crowds in narrow corridors make the cautious robot wait a lot. That is the safe failure mode, but the wait times need tuning once we have real corridor widths.
- Lidar only sees one plane. Low or overhanging obstacles, glass and black surfaces need the RealSense (not used yet).
- Constant-velocity prediction. People who stop or turn suddenly are handled by the stopping distance, not by prediction.
- The stack is Python in a 2D kinematic simulator: no wheel slip, no latency jitter. Gazebo and then the real rover come later.

## C++ port (Nov 8)
`include/fpp/dynamic.hpp`, `src/dynamic.cpp`, `src/dynamic_c_api.cpp`: the Tracker and the Governor, ROS-free, same logic as `py/dyn.py`.
- `tests/test_dynamic.cpp`: 9 unit tests (clear path, person beside the path, stopping distance always fits inside the free distance, person right in front, crossing walker is predicted, creep past a person, turn blocked by a person, tracker finds a standing person, walls alone are not people). Runs in CI.
- `py/test_dyn_cpp.py`: 3000 random scenes (worst difference between C++ and Python 2e-16) and 4 full simulated drives with walkers (same outcome, same time, same stops). Run by hand: `python3 py/test_dyn_cpp.py` (needs `results_corridor.json` from `py/exp_corridor.py`).
- Governor decision: about 0.08 ms in Python, 0.012 ms through the C++ library. Either is far below one control step (50 ms); the C++ version is what the Nav2 plugin will call.
