# Robotics_Assignment_Navigation

## Path Tracker – ROS2 Trajectory Tracking System

A ROS2 package implementing path smoothing, trajectory generation, and trajectory tracking for differential drive robots using cubic spline interpolation and pure pursuit control.

---

## Reference Links
Documenation: https://docs.google.com/document/d/1qV9PhmaOmRzm_XL4lg4zYXtQ7TjaCCGXxUShRvJjeLY/edit?usp=sharing
Demo Videos : https://drive.google.com/drive/folders/1ayvLRf3VbKC25sDm4E86IiKSdpCom8Wt?usp=drive_link

## Overview

This package provides an end-to-end navigation pipeline comprising:

* **Path Smoothing:** Converts discrete waypoints into continuous, differentiable paths using natural cubic splines.
* **Trajectory Generation:** Produces time-parameterized trajectories with trapezoidal velocity profiles, considering acceleration and curvature constraints.
* **Trajectory Tracking:** Implements a pure pursuit control algorithm for precise path following under kinematic constraints.

### Key Features

* Natural cubic spline path smoothing
* Trapezoidal velocity profile generation
* Pure pursuit trajectory tracking
* Compatible with both simulated and physical robots (TurtleBot3 Gazebo)
* Real-time visualization in RViz
* Built-in tracking error computation and performance logging

---

## Dependencies

* ROS2 Humble (or later)
* C++17 compiler
* TurtleBot3 packages (for Gazebo simulation)

---

## Installation & Usage

### 1. Clone the repository

```bash
cd ~/ros2_ws/src
git clone path_tracker
```

### 2. Install dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select path_tracker
source install/setup.bash
```

### Simulation Mode (Internal Kinematics)

```bash
ros2 launch path_tracker trajectory_tracking.launch.py waypoints_file:=waypoints1.yaml
```

### Gazebo Simulation Mode (TurtleBot3)

```bash
ros2 launch path_tracker gazebo_trajectory_tracking.launch.py waypoints_file:=waypoints3.yaml
ros2 launch path_tracker view_graphs.launch.py
```

This configuration launches:

* Gazebo world with TurtleBot3
* Path Smoother node
* Trajectory Generator node
* Trajectory Tracker node (using Gazebo odometry)
* RViz visualization

---

## System Architecture

```
Path Smoother → Trajectory Generator → Trajectory Tracker → Robot
↓                  ↓                      ↓
/smoothed_path     /trajectory            /cmd_vel
```

---

## Node Descriptions

### 1. Path Smoother Node (`path_smoother_node`)

**Algorithm:** Natural Cubic Splines

**Design Details:**

* Implements parametric cubic splines (independent for x and y coordinates)
* Chord-length parameterization for stable curve spacing
* Thomas algorithm for efficient tridiagonal matrix solutions
* Handles duplicate and collinear input points

**Published Topics:**

* `/original_path` (`nav_msgs/Path`) – Original coarse waypoint sequence
* `/smoothed_path` (`nav_msgs/Path`) – Smoothed continuous path

**Implementation Example:**

```cpp
// Natural boundary conditions: c[0] = c[n-1] = 0
// Ensures smooth start and end without introducing artificial acceleration.
```

---

### 2. Trajectory Generator Node (`trajectory_generator_node`)

**Algorithm:** Trapezoidal Velocity Profile

**Design Details:**

* Forward pass applies acceleration limits
* Backward pass ensures deceleration at the terminal state
* Curvature-based velocity limiting ensures safe cornering
* Time stamps computed using average segment velocities

**Subscribed Topics:**

* `/smoothed_path` (`nav_msgs/Path`)

**Published Topics:**

* `/trajectory` (`nav_msgs/Path`) – Time-parameterized trajectory
* `/trajectory_odom` (`nav_msgs/Odometry`) – Debugging data
---

### 3. Trajectory Tracker Node (`trajectory_tracker_node`)

**Algorithm:** Pure Pursuit Controller

**Design Details:**

* Adaptive lookahead distance for smooth control response
* Velocity modulation based on remaining distance to goal
* Window-based nearest-point search for computational efficiency
* Compatible with both simulated and real odometry inputs

**Parameters:**

| Parameter          | Description                     | Default   |
| ------------------ | ------------------------------- | --------- |
| lookahead_distance | Pure pursuit lookahead distance | 0.5 m     |
| max_linear_vel     | Maximum linear velocity         | 0.5 m/s   |
| max_angular_vel    | Maximum angular velocity        | 1.0 rad/s |
| position_tolerance | Goal position tolerance         | 0.15 m    |
| control_frequency  | Control loop rate               | 50 Hz     |
| k_v                | Linear velocity gain            | 1.0       |
| k_omega            | Angular velocity gain           | 2.0       |
| use_gazebo_odom    | Use Gazebo odometry input       | false     |

**Subscribed Topics:**

* `/trajectory` (`nav_msgs/Path`)
* `/odom` (`nav_msgs/Odometry`) *(when use_gazebo_odom=true)*

**Published Topics:**

* `/cmd_vel` (`geometry_msgs/Twist`) – Velocity command output
* `/robot_odom` (`nav_msgs/Odometry`) – Internal odometry feedback
* `/tracking_marker` (`visualization_msgs/Marker`) – Visualization marker
* `/lookahead_target` (`geometry_msgs/PoseStamped`) – Target waypoint

---

## Performance Evaluation

The node logs key performance metrics during operation:

* Average tracking error
* Maximum tracking error
* Total execution time

**Example Output:**

```markdown
[trajectory_tracker_node]: Trajectory tracking complete.
[trajectory_tracker_node]: Average tracking error: 0.042 m
[trajectory_tracker_node]: Maximum tracking error: 0.089 m
[trajectory_tracker_node]: Total time: 24.50 s
```

---
