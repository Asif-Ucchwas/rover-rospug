# Rover + ROSPug

An autonomous snack delivery system for the Cherry building (Lamar University, Electrical Engineering).
A Rover Robotics MEGA drives the corridors with a footprint-aware path planner, and a ROSPug robot dog
delivers to the offices the rover cannot enter.

## Layout
- `planner/`: footprint-aware path planner (C++ core, tests, simulated experiments). See `planner/README.md`.
- `sim/`: Gazebo Harmonic twin of the building and a Rover MEGA model (ROS 2 Jazzy). See `sim/README.md`.
- More folders will be added as the project grows (Nav2 plugin, dog delivery).
