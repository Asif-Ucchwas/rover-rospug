# sim: Gazebo twin (Stage 1)

ROS 2 Jazzy + Gazebo Harmonic. The worlds are generated from the planner's own occupancy grids (`planner/py/worlds.py`), so the physics world and the planner map are the same map. When the Cherry floor is measured, it becomes a grid and goes through the same script.

## Build
```bash
cd ~/rover-rospug
source /opt/ros/jazzy/setup.bash
colcon build --base-paths sim --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch rover_sim sim.launch.py world:=gap_1p20            # with the Gazebo window
ros2 launch rover_sim sim.launch.py world:=corridor            # the engineering-floor placeholder
ros2 launch rover_sim sim.launch.py world:=gap_0p90 headless:=true
# in a second terminal: drive and check
ros2 run rover_sim check_sim.py --distance 12 --expect pass
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"   # manual drive
```

## Automatic door test
```bash
./sim/run_gap_tests.sh
```
The rover is 0.81 m wide: 0.70 m and 0.80 m gaps must block it, 0.90 m and 1.20 m must let it through; the lab door with both leaves open (1.32 m) must pass.

## Regenerate worlds
```bash
python3 sim/rover_sim/scripts/grid_to_sdf.py
```

## Rover model
1120 x 810 mm (measured) collision box (the real footprint) with an orange six-wheel rover look, a snack bin on top and a lidar mast; two drive wheels plus two casters, 2D lidar (720 beams, 20 m, mounted 0.70 m above ground), diff-drive odometry. Mass and speed limits are placeholders until measured.
Topics: `/cmd_vel`, `/odom`, `/tf`, `/scan`, `/joint_states`, `/clock`.
