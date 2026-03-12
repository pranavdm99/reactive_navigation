# ROS 2 Reactive Navigation

This project implements a robot navigation system in Gazebo using ROS 2 Humble. It features a robust **Bug2 Algorithm** implementation capable of navigating complex environments defined by custom STL meshes.

## Key Features
- **STL Mesh Support**: Navigation within complex 3D environments (e.g., `bug_world.stl`).
- **Refined Bug2 Algorithm**:
    - **Side-Tracking**: Reliable M-line intersection detection (prevents high-speed overshooting).
    - **Wall-Side Configuration**: Choose between Left-Hand or Right-Hand wall following.
    - **Progress Guarantee**: Mathematical convergence via $d_{leave} < d_{hit}$ logic.
- **Dynamic Configuration**: All navigation parameters (speed, tolerance, safety distance) are externalized in `parameters.yaml`.
- **Navigation Analytics**: Automatic summary of distance, time, and positioning accuracy upon goal arrival.
- **Dynamic Spawning**: Decoupled robot description using official TurtleBot3 patterns and URDF injection.

## Project Structure
- `src/reactive_nav/reactive_nav/`: Python source code (Autonomous Nav, Teleop).
- `src/reactive_nav/worlds/`: Gazebo world files and STL meshes.
- `src/reactive_nav/config/`: YAML parameter configurations.
- `src/reactive_nav/launch/`: Integrated ROS 2 launch scripts.

## Quick Start (Interactive)

1. **Environmental Setup**:
   ```bash
   xhost +local:docker
   docker compose up -d --build
   ```

2. **Launch Integrated Simulation**:
   ```bash
   docker compose exec ros_env bash
   # Inside the container:
   colcon build --packages-select reactive_nav
   source install/setup.bash
   ros2 launch reactive_nav bug_behavior.launch.py
   ```

3. **Navigate**:
   - Open RViz: `ros2 run rviz2 rviz2`.
   - Use **2D Nav Goal** to set destinations.
   - Watch the terminal for real-time state logs and final navigation summaries.

## Configuration
Navigation behavior is configured via `src/reactive_nav/config/parameters.yaml`.

| Parameter         | Default   | Description                                              |
| :---------------- | :-------- | :------------------------------------------------------- |
| `max_speed`       | `0.22`    | Maximum linear velocity (m/s).                           |
| `turning_speed`   | `0.5`     | Angular velocity (rad/s) for rotations.                  |
| `safety_dist`     | `0.4`     | Distance (m) to detect walls and trigger Wall-Following. |
| `wall_side`       | `"right"` | Which side to keep the wall on (`"left"` or `"right"`).  |
| `goal_tolerance`  | `0.01`    | Stop distance (m) from the target goal coordinates.      |
| `mline_tolerance` | `0.1`     | Sensitivity (m) for detecting the Goal-Line (M-Line).    |
| `progress_delta`  | `0.1`     | Required distance (m) closer to goal to leave a wall.    |
