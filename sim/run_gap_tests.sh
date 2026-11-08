#!/usr/bin/env bash
# Runs the door-gap test in Gazebo, headless, one world at a time. Needs: colcon build done and install/setup.bash sourced.
# Rover is 0.77 m wide: a 0.70 m gap must block it, 0.90 m and 1.20 m must let it through.
set -u
declare -A EXPECT=( [gap_0p70]=block [gap_0p80]=any [gap_0p90]=pass [gap_1p20]=pass )
fail=0
for w in gap_0p70 gap_0p80 gap_0p90 gap_1p20; do
  echo "=== $w (expect ${EXPECT[$w]}) ==="
  ros2 launch rover_sim sim.launch.py world:=$w headless:=true > /tmp/rover_sim_$w.log 2>&1 &
  LP=$!; sleep 12
  ros2 run rover_sim check_sim.py --distance 12 --expect "${EXPECT[$w]}" || fail=1
  kill -INT $LP 2>/dev/null; sleep 2; pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; wait $LP 2>/dev/null; sleep 2
done
[ $fail -eq 0 ] && echo "ALL GAP TESTS OK" || echo "SOME GAP TESTS FAILED (logs in /tmp/rover_sim_*.log)"
exit $fail
