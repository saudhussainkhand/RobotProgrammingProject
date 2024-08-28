cmake_minimum_required(VERSION 3.8)
project(simple_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # Already added
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)  # Already added

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add executable
add_executable(simple_planner src/simple_planner.cpp src/search.cpp src/distance_map.cpp src/utils.cpp)

# Link dependencies to the executable
ament_target_dependencies(simple_planner 
  rclcpp 
  geometry_msgs 
  std_msgs 
  nav_msgs 
  tf2_ros 
  tf2_geometry_msgs 
  visualization_msgs 
  sensor_msgs  # Already added
  OpenCV
)

# Link OpenCV libraries
target_link_libraries(simple_planner ${OpenCV_LIBRARIES})

# Install the target
install(TARGETS simple_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files (new part)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_auto_FIND_DEPENDENCIES)
endif()

ament_package()

