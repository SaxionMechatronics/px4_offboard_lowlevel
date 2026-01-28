cmake_minimum_required(VERSION 3.8)
project(px4_offboard_lowlevel)

# Ensure Python is found
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # <-- Add this for Python node support
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)



# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories for C++
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Add executables (C++ nodes)
add_executable(offboard_controller_node 
  src/controller.cpp
  src/controller_node.cpp
)

add_executable(circle_trajectory_node 
  src/TrajectoryPublishers/circle_trajectory_node.cpp
)

# Add dependencies for the C++ executables
ament_target_dependencies(offboard_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
  px4_msgs
  trajectory_msgs
)

ament_target_dependencies(circle_trajectory_node
  rclcpp
  geometry_msgs
)

# Install the C++ executables
install(TARGETS offboard_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS circle_trajectory_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts (including your visualizer.py)
install(PROGRAMS
  scripts/identification.py  # Existing Python script
  scripts/visualizer.py      # <-- Add visualizer.py here for installation
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY 
  resource
  DESTINATION share/${PROJECT_NAME})


# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Optionally add testing (unchanged)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

