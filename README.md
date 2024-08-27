# RobotProgrammingProject

Final project for the course of Robot Programming

## Simple Planner: A* Based Path Planner with Obstacle Avoidance

This ROS2 package implements a simple A* based path planner with obstacle avoidance capabilities. The planner uses laser scans to detect obstacles and works alongside the AMCL localization and the Nav2 stack.

## Project Structure

The project is organized as follows:

### Directory Structure:

- **simple_planner/**
  - **config/**
    - `amcl.yaml` - AMCL configuration file
    - `params.yaml` - Nav2 parameters for planner, controller, and costmaps
    - `bt_navigator.yaml` - Behavior Tree navigator configuration
  - **launch/**
    - `planner_launch.xml` - Main launch file to run the entire system
  - **src/**
    - `simple_planner.cpp` - A* planner with obstacle avoidance capabilities
    - `search.cpp` - A* search algorithm implementation
    - `distance_map.cpp` - Functions for creating a distance map from the occupancy grid
    - `utils.cpp` - Utility functions for the planner
  - **include/**
    - `search.hpp` - Header file for A* search algorithm
    - `distance_map.hpp` - Header file for distance map computations
    - `utils.hpp` - Header file for utility functions
- **worlds/**
    - `cappero_laser_odom_diag.world` - World file for use with Stage 2 simulator
    - `cappero_laser_odom_diag.yaml` - Map YAML file for loading the environment in ROS2

### Stage and Stage ROS2

- **Stage:**
    - Stage is a 2D multi-robot simulator used for testing and developing robot navigation strategies. It provides a virtual world in which robots can move, sense, and interact with each other and the environment. Stage can simulate various sensors like laser scanners, cameras, and sonar.
    - In this project, Stage is used to simulate the robot's environment and the obstacles within it, providing data for the planner to navigate and avoid obstacles.

- **Stage ROS2:**
    - Stage ROS2 is the ROS2 interface for Stage, allowing for integration with ROS2 topics, services, and messages. It connects Stage to the ROS2 ecosystem, enabling the simulation data to be processed in real-time by ROS2-based applications.
    - This project utilizes Stage ROS2 to simulate the environment where the robot moves, and the robot's sensor data is published to ROS2 topics such as `/base_scan`, `/odom`, and `/tf`.


### How to Run the Project:

1. **Clone the repository:**
    ```bash
    git clone https://github.com/saudhussainkhand/RobotProgrammingProject.git
    ```

2. **Build the project using colcon at your ros2_ws:**
    ```bash
    cd ros2_ws/
    colcon build
    ```

3. **Source the workspace:**
    ```bash
    source install/setup.bash
    ```

4. **Run the entire system by launching the `planner_launch.xml` file:**
    ```bash
    ros2 launch simple_planner planner_launch.xml
    ```

### Features:
- **A* Path Planning:** Implements the A* search algorithm to compute an optimal path from the start to the goal.
- **Obstacle Avoidance:** Uses laser scans to avoid obstacles in the environment.
- **AMCL Localization:** Integrated with the AMCL package for robot localization in a known map.
- **Nav2 Integration:** Utilizes Nav2's costmaps, controllers, and behavior trees for navigation.

### Dependencies:
- **ROS 2 Humble**
- **Nav2 Stack**
- **OpenCV**
- **tf2 and tf2_ros**

