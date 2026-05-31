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
    ```

# Source ROS2
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```

# Build packages
    ```bash
    colcon build --symlink-install
    ```

# Source install
    ```bash
    source install/setup.bash
    ```

## Usage

Before Running Any Mission
Start PX4 Gazebo simulation in a terminal:
    ```bash
    cd ~/PX4-Autopilot
    make px4_sitl gazebo
    ```

# Terminal 1: Grid search node
    ```bash
    source /opt/ros/jazzy/setup.bash
    cd ~/Drone_Project/first_flight_code
    source install/setup.bash
    ros2 run grid_search grid_search
    ```

# Terminal 2: Visualizer (optional)
    ```bash
    source /opt/ros/jazzy/setup.bash
    cd ~/Drone_Project/first_flight_code
    source install/setup.bash
    ros2 run grid_search grid_visualizer
    ```

# Terminal 3: RViz visualization (optional)
    ```bash
    source /opt/ros/jazzy/setup.bash
    rviz2
    ```

`

Nodes
grid_search
Autonomous grid search pattern for surveillance/mapping

Takes off to 5m altitude
Flies grid pattern (10m x 6m with 2m line spacing)
Returns to origin and lands
Real-time waypoint tracking in terminal
Run:
    ```bash
    ros2 run grid_search grid_search
    ```


Takes off to 5m altitude
Flies grid pattern (10m x 6m with 2m line spacing)
Returns to origin and lands
autonomous_flight

Future Improvements
 YOLO object detection integration
 Real drone hardware testing (Pixhawk)
 Advanced path planning algorithms
 Obstacle avoidance with LiDAR
 Sensor fusion (IMU + vision)
 Multi-drone coordination
 Real-time video streaming
 Improved landing detection

References
PX4 Documentation
ROS2 Documentation
Gazebo Documentation
