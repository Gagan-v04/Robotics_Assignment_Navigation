cmake_minimum_required(VERSION 3.8)
project(path_tracker)

# ========================
# Compiler options
# ========================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# Dependencies
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ========================
# Include directories
# ========================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ========================
# Executables
# ========================

# Path Smoother Node
add_executable(path_smoother_node src/path_smoother_node.cpp)
ament_target_dependencies(path_smoother_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)
target_include_directories(path_smoother_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

# Trajectory Generator Node
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
ament_target_dependencies(trajectory_generator_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Trajectory Tracker Node
add_executable(trajectory_tracker_node src/trajectory_tracker_node.cpp)
ament_target_dependencies(trajectory_tracker_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

# ========================
# Installation
# ========================
install(TARGETS
  path_smoother_node
  trajectory_generator_node
  trajectory_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# ========================
# Testing / Linting
# ========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ========================
# Ament package declaration
# ========================
ament_package()