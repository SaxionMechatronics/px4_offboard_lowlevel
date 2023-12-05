cmake_minimum_required(VERSION 3.8)
project(px4_offboard_lowlevel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}  # Use EIGEN3_INCLUDE_DIR instead of EIGEN3_DIR
)

add_executable(offboard_controller_node 
  src/controller.cpp
  src/controller_node.cpp
)

add_executable(circle_trajectory_node 
  src/TrajectoryPublishers/circle_trajectory_node.cpp
)

# Add dependencies for the executables
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

install(TARGETS offboard_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS circle_trajectory_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/identification.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
