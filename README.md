# ROS 2 Reactive Navigation

This project implements a robot navigation system in Gazebo using ROS 2 Humble. It includes custom teleoperation with real-time feedback and an autonomous navigation system using the **Bug2 Algorithm**.

## Features
- **Containerized Environment**: One-command setup using Docker and Docker Compose.
- **Dynamic Teleoperation**: Keyboard control with smooth acceleration, auto-braking, and real-time velocity step configuration.
- **Bug2 Autonomous Navigation**:
    - Obstacle avoidance using LIDAR.
    - Goal seeking via RViz mouse clicks.
    - State machine for switching between goal-seeking and wall-following.
- **Visual Feedback**: Real-time status updates in the terminal console.

## Setup

1. **Build the container**:
   ```bash
   docker compose build
   ```
2. **Allow X11 connections** (for Gazebo/RViz):
   ```bash
   xhost +local:docker
   ```
3. **Start the environment**:
   ```bash
   docker compose up -d
   ```

## Usage

### 1. Launch Simulation
Inside the container:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Teleoperation
```bash
ros2 run reactive_nav teleop_node
```
- `w/x`: Move forward/backward.
- `a/d`: Turn left/right.
- `t/g` & `y/h`: Adjust step sizes.

### 3. Autonomous Navigation
```bash
ros2 run reactive_nav autonomous_nav_node
```
- Open RViz: `ros2 run rviz2 rviz2`.
- Set a goal using the **2D Nav Goal** tool.

## Configuration
Tunable parameters for the Bug2 node:
- `max_speed`: Maximum linear velocity.
- `safety_dist`: Distance to trigger wall-following.
- `turning_speed`: Speed of rotation during avoidance.
