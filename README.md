# Autonomous Drone Project with PX4 and ROS2

This project demonstrates autonomous drone flight using PX4 autopilot, ROS2, and Gazebo simulation.

## Features

- ✈️ PX4 autopilot integration
- 🤖 ROS2 offboard control
- 📡 Custom ROS2 nodes in Python
- 🗺️ Autonomous grid search pattern
- 📍 Letter tracing mission
- 🎮 Gazebo simulation

## Project Structure
first_flight_code/
├── src/
│ ├── px4_msgs/ # PX4 message definitions
│ ├── my_takeoff_node/ # Basic takeoff node
│ ├── autonomous_flight/ # Letter tracing mission
│ └── grid_search/ # Grid search pattern mission
├── build/ # CMake build (auto-generated)
├── install/ # Installation files (auto-generated)
└── README.md

text

## Installation

### Prerequisites
- Ubuntu 24.04 (Jammy)
- ROS2 Jazzy
- PX4 Autopilot
- Gazebo
- QGroundControl

### Setup

```bash
# Clone repository
git clone <your-repo-url>
cd first_flight_code

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build packages
colcon build --symlink-install

# Source install
source install/setup.bash
Usage
Run Grid Search Mission
bash
# Terminal 1: Grid search node
ros2 run grid_search grid_search

# Terminal 2: Visualizer (optional)
ros2 run grid_search grid_visualizer

# Terminal 3: RViz visualization (optional)
rviz2
Run Autonomous Flight (Letter Tracing)
bash
# Terminal 1: Mission node
ros2 run autonomous_flight mission

# Terminal 2: Path visualizer (optional)
ros2 run autonomous_flight visualizer

# Terminal 3: RViz
rviz2
Nodes
grid_search
Autonomous grid search pattern for surveillance/mapping

Takes off to 5m altitude
Flies grid pattern (10m x 6m with 2m line spacing)
Returns to origin and lands
autonomous_flight
Letter tracing mission

Takes off
Traces letter 'L' shape
Returns to origin
Lands
my_takeoff_node
Basic takeoff demonstration

ROS2 Topics
Published
/fmu/in/offboard_control_mode - Control mode setpoint
/fmu/in/trajectory_setpoint - Position setpoint
/fmu/in/vehicle_command - Vehicle commands (arm, disarm, etc.)
Subscribed
/fmu/out/vehicle_status - Drone status
/fmu/out/vehicle_local_position_v1 - Current position
/fmu/out/vehicle_attitude - Drone attitude
Future Improvements
 YOLO object detection integration
 Real drone hardware testing
 Advanced path planning
 Sensor fusion
 Multi-drone coordination
License
Apache License 2.0

References
PX4 Documentation
ROS2 Documentation
Gazebo Documentation
