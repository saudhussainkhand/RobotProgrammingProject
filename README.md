# RobotProgrammingProject

Final project for the course of Robot Programming

## Simple Planner: A* Based Path Planner with Obstacle Avoidance

This Project implements a simple A* based path planner with obstacle avoidance capabilities. The planner uses laser scans to detect obstacles and works alongside the AMCL localization and the Nav2 stack.

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
2. **Move the simple_planner folder in your workspace src:**
3. **Build the project using colcon at your ros2_ws:**
    ```bash
    cd ros2_ws/
    colcon build
    ```

4. **Source the workspace:**
    ```bash
    source install/setup.bash
    ```

5. **Run the entire system by launching the `planner_launch.xml` file:**
    ```bash
    ros2 launch simple_planner planner_launch.xml
    ```

### How Simple Planner Implements Key Features

1. **Write a program that computes the path to a user-selected goal from the current location of the robot, received as the transform map->base_link.**
   - The planner computes a path from the robot’s current position (derived from `map -> base_link` transform) to a user-selected goal using the A* algorithm.

2. **The program listens to a grid_map, extracts the obstacles, and the traversable surface (cells “brighter” than a value).**
   - The `distance_map` function processes the occupancy grid and creates a map highlighting obstacles and traversable areas. This map is used for the path computation.

3. **The cost of being in a location depends on the distance to the closest obstacle (the smaller, the higher).**
   - The planner assigns a cost to each traversable cell based on its proximity to the nearest obstacle. Cells closer to obstacles have a higher cost, guiding the robot to safer paths.

4. **Using this cost function, the program computes the path (if existing) to the goal by using your favorite search algorithm.**
   - The planner utilizes the A* search algorithm, considering the cost map, to compute an optimal path to the goal if one exists.

5. **The goal pose is received from the /move_base/goal message.**
   - In this project, the goal pose is set using the RViz tool (by publishing to `/goal_pose`), and the planner listens to the goal pose updates to begin the pathfinding process.



### Dependencies:
- **ROS 2 Humble**
- **Nav2 Stack**
- **OpenCV**
- **tf2 and tf2_ros**

