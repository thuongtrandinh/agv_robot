# AGV Fuzzy Trajectory Controller

This package implements a fuzzy logic trajectory tracking controller for differential drive mobile robots based on the Omrane2016 paper.

## Overview

The controller uses Mamdani fuzzy inference to compute wheel velocities (VL, VR) based on:
- **Distance** to the next trajectory point (0-500mm)
- **Angle** error to the next trajectory point (-180° to 180°)

## Implementation Details

### Fuzzy Logic Components

#### Input Variables

1. **Distance (d)** - Range: [0, 500] mm
   - VS (Very Small): [0, 50, 100]
   - S (Small): [50, 100, 150]
   - M (Medium): [100, 200, 300]
   - B (Big): [200, 350, 500]
   - VB (Very Big): [350, 500, 500]

2. **Angle (θ)** - Range: [-180, 180] degrees
   - NB (Negative Big): [-180, -180, -120]
   - NM (Negative Medium): [-180, -90, 0]
   - NS (Negative Small): [-90, -45, 0]
   - Z (Zero): [-45, 0, 45]
   - PS (Positive Small): [0, 45, 90]
   - PM (Positive Medium): [0, 90, 180]
   - PB (Positive Big): [120, 180, 180]

#### Output Variables

**Velocity Singletons (mm/s):**
- Z (Zero): 0 mm/s
- S (Small): 15 mm/s *(added per user request)*
- F (Few): 30 mm/s
- M (Medium): 40 mm/s
- B (Big): 50 mm/s
- VB (Very Big): 70 mm/s

### Fuzzy Rules

The controller implements 35 fuzzy rules from Omrane2016 Table 1:

| Distance/Angle | NB  | NM  | NS  | Z   | PS  | PM  | PB  |
|----------------|-----|-----|-----|-----|-----|-----|-----|
| VS             | Z,VB| S,B | S,M | F,F | S,M | S,B | Z,VB|
| S              | Z,VB| F,B | F,M | M,M | F,M | F,B | Z,VB|
| M              | S,VB| M,B | M,M | B,B | M,M | M,B | S,VB|
| B              | F,VB| B,B | B,M | VB,VB| B,M| B,B | F,VB|
| VB             | M,VB| VB,B| VB,M| VB,VB| VB,M| VB,B| M,VB|

Format: (VL, VR) for each combination

### Differential Drive Kinematics

The controller converts fuzzy output (VL, VR) to robot velocities:
- Linear velocity: `v = (VL + VR) / 2`
- Angular velocity: `ω = (VR - VL) / wheel_base`

## ROS2 Interface

### Subscribed Topics

- `/odom` (nav_msgs/Odometry): Robot odometry for current pose
- `/trajectory` (nav_msgs/Path): Desired trajectory to follow

### Published Topics

- `/cmd_vel` (geometry_msgs/TwistStamped): Velocity commands for the robot

### Parameters

- `wheel_base` (double, default: 0.35): Distance between wheels [m]
- `max_linear_vel` (double, default: 0.5): Maximum linear velocity [m/s]
- `max_angular_vel` (double, default: 1.0): Maximum angular velocity [rad/s]
- `control_frequency` (double, default: 20.0): Control loop frequency [Hz]
- `goal_tolerance` (double, default: 0.1): Goal reached tolerance [m]

## Usage

### Launch the Controller

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch with default parameters
ros2 launch agv_fuzzy_trajectory fuzzy_trajectory.launch.py

# Launch with custom parameters
ros2 launch agv_fuzzy_trajectory fuzzy_trajectory.launch.py \
  wheel_base:=0.40 \
  max_linear_vel:=0.7 \
  control_frequency:=30.0
```

### Test with Sample Trajectory

Open a new terminal and run the test trajectory publisher:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run agv_fuzzy_trajectory test_trajectory_publisher.py
```

This publishes a 2m x 2m square trajectory for testing.

### Monitor the Controller

```bash
# View velocity commands
ros2 topic echo /cmd_vel

# View trajectory
ros2 topic echo /trajectory

# Check node info
ros2 node info /fuzzy_trajectory_controller
```

## Integration with Simulation

To use with the AGV simulation:

1. Launch the simulation:
```bash
ros2 launch mobile_robot launch_sim.launch.py
```

2. In a new terminal, launch the fuzzy controller:
```bash
ros2 launch agv_fuzzy_trajectory fuzzy_trajectory.launch.py
```

3. Publish a trajectory using Nav2 or a custom path planner

## Algorithm Details

### Fuzzification

- Triangular and trapezoidal membership functions
- Membership degree computed for each input variable

### Inference

- Mamdani min-max inference
- Rule firing strength: minimum of input membership degrees
- Output aggregation: maximum of all rule outputs

### Defuzzification

- Center of Gravity (CoG) method
- Formula: `output = Σ(μᵢ × singletonᵢ) / Σ(μᵢ)`

## Dependencies

- rclpy: ROS2 Python client library
- geometry_msgs: For Twist messages
- nav_msgs: For Path and Odometry messages
- tf2_ros: For transform handling
- tf_transformations: For quaternion conversions
- numpy: For numerical computations

## Reference

Based on: Omrane, H., et al. (2016). "Fuzzy Logic Based Control for Autonomous Mobile Robot Navigation"

## Notes

- The 'S' (Small) output level at 15mm/s was added to provide finer control at low speeds
- The controller automatically stops when the goal is reached (within goal_tolerance)
- All angles are internally normalized to [-180, 180] degrees range
