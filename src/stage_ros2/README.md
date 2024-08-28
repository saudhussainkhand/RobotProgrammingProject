
## Stage ROS2
**Stage ROS2:**
    - Stage ROS2 is the ROS2 interface for Stage, allowing for integration with ROS2 topics, services, and messages. It connects Stage to the ROS2 ecosystem, enabling the simulation data to be processed in real-time by ROS2-based applications.
    - This project utilizes Stage ROS2 to simulate the environment where the robot moves, and the robot's sensor data is published to ROS2 topics such as `/base_scan`, `/odom`, and `/tf`.

### How to add it
1. **Clone the repository:**
    ```bash
    git clone https://github.com/tuw-robotics/stage_ros2.git
    ```
2. **Move the stage_ros2 folder in your workspace src:**
