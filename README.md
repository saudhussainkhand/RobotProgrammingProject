# RobotProgrammingProject
Final project for the course of Robot Programming 

# Simple Planner: A* Based Path Planner with Obstacle Avoidance

This ROS2 package implements a simple A* based path planner with obstacle avoidance capabilities. The planner uses laser scans to detect obstacles and works alongside the AMCL localization and the Nav2 stack.

## Project Structure

The project is organized as follows:


simple_planner/ │ ├── config/ │ ├── amcl.yaml # AMCL configuration file │ ├── all_params.yaml # Nav2 parameters for planner, controller, and costmaps │ └── bt_navigator.yaml # Behavior Tree navigator configuration │ ├── launch/ │ └── launch_all.xml # Main launch file to run the entire system │ ├── src/ │ ├── simple_planner.cpp # A* planner with obstacle avoidance capabilities │ ├── search.cpp # A* search algorithm implementation │ ├── distance_map.cpp # Functions for creating a distance map from the occupancy grid │ └── utils.cpp # Utility functions for the planner │ ├── include/ │ ├── search.hpp # Header file for A* search algorithm │ ├── distance_map.hpp # Header file for distance map computations │ └── utils.hpp # Header file for utility functions │ ├── worlds/ │ └── cappero_laser_odom_diag.world # World file for use with Stage 2 simulator │ └── cappero_laser_odom_diag.yaml # Map YAML file for loading the environment in ROS2 │ ├── CMakeLists.txt # Build system configuration file └── package.xml # Package metadata file



### Folder Descriptions

- **config/**: Contains the configuration files for AMCL, the Nav2 stack (planner, controller, costmaps), and the behavior tree navigator.
  
- **launch/**: Contains the main launch file (`launch_all.xml`) that starts the entire system, including RViz, AMCL, Nav2 planner and controller, and the A* planner.

- **src/**: Source code for the A* based planner (`simple_planner.cpp`), distance map generation, and utility functions.

- **include/**: Header files for source code, including A* algorithm implementation and utilities.

- **worlds/**: Contains the world and map files for use with Stage 2 simulator.

## How to Run the Project

### 1. Clone the Repository

```bash
git clone <repository-url>
cd your_ros_workspace


### 2. Build the Package
colcon build

### 3. Source the Workspace
source install/setup.bash

### 4. Launch the Project
ros2 launch simple_planner launch_project.xml



### Instructions Included in the README:

- **How to Run the Project**: Provides step-by-step instructions on cloning the repository, building it with `colcon`, sourcing the workspace, and launching the entire system with `ros2 launch`.
- **Visualization**: Instructions on how to visualize laser scans, paths, and the 2D Pose Estimate tool in RViz.
- **Optional Stage 2 Simulation**: Instructions on how to use the world file with the Stage 2 simulator by uncommenting the relevant lines in the launch file.
- **Configuration Files Overview**: A section explaining the purpose of key configuration files (`amcl.yaml`, `all_params.yaml`, and `bt_navigator.yaml`).
- **Future Enhancements**: Suggestions for future work on improving obstacle avoidance and integrating SLAM.
- **Maintainer Information**: Your contact information for support.



