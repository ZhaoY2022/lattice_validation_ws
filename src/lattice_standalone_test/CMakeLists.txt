cmake_minimum_required(VERSION 3.8)
project(lattice_standalone_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
# Use conda's yaml-cpp (system 0.6 has old ABI)
find_package(yaml-cpp REQUIRED PATHS /home/zy/miniconda3 NO_DEFAULT_PATH)

# Force yaml-cpp include from conda, not /usr/local
include_directories(BEFORE /home/zy/miniconda3/include)

include_directories(
  include
  include/Lattice_planner
  include/Frenet
  include/Polynomial
  include/Spline
  include/path
  include/ReferenceLine
  include/common
  include/Obstacle
  ${EIGEN3_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${builtin_interfaces_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

# Lattice core library (no ROS deps)
set(LATTICE_CORE_SRCS
  # Lattice_planner
  src/Lattice_planner/lattice_planner.cpp
  src/Lattice_planner/trajectory_evaluator.cpp
  src/Lattice_planner/trajectory1d_generator.cpp
  src/Lattice_planner/trajectory_combiner.cpp
  src/Lattice_planner/collision_checker.cpp
  src/Lattice_planner/constraint_checker.cpp
  src/Lattice_planner/end_condition_sampler.cpp
  src/Lattice_planner/feasible_region.cpp
  src/Lattice_planner/lattice_trajectory1d.cpp
  src/Lattice_planner/path_time_graph.cpp
  src/Lattice_planner/piecewise_acceleration_trajectory1d.cpp
  src/Lattice_planner/piecewise_braking_trajectory_generator.cpp
  src/Lattice_planner/prediction_querier.cpp
  src/Lattice_planner/PlanningTarget.cpp
  # Frenet
  src/Frenet/cartesian_frenet_conversion.cpp
  # Polynomial
  src/Polynomial/polynomial_xd.cpp
  src/Polynomial/QuarticPolynomial.cpp
  src/Polynomial/quartic_polynomial_curve1d.cpp
  src/Polynomial/QuinticPolynomial.cpp
  src/Polynomial/quintic_polynomial_curve1d.cpp
  # Spline
  src/Spline/CubicSpline1D.cpp
  src/Spline/CubicSpline2D.cpp
  # path
  src/path/path_matcher.cpp
  src/path/path_points.cpp
  src/path/trajectoryPoint.cpp
  src/path/FrenetPath.cpp
  # ReferenceLine
  src/ReferenceLine/reference_point.cpp
  # common
  src/common/Config.cpp
  src/common/vec2d.cpp
  src/common/box2d.cpp
  src/common/line_segment2d.cpp
  src/common/polygon2d.cpp
  src/common/math_utils.cpp
  src/common/angle.cpp
  src/common/auxiliary_function.cpp
  src/common/sin_table.cpp
  # Obstacle
  src/Obstacle/Obstacle.cpp
  src/Obstacle/boundarys.cpp
  src/Obstacle/Ob_prediction_trajectory.cpp
  src/Obstacle/Obstacle_avoid.cpp
)

add_library(lattice_core STATIC ${LATTICE_CORE_SRCS})

# Test node executable
add_executable(lattice_test_node
  src/lattice_test_node.cpp
)
# Use conda's yaml-cpp (system version has ABI incompatibility)
find_library(YAML_CPP_LIB_PATH yaml-cpp PATHS /home/zy/miniconda3/lib NO_DEFAULT_PATH REQUIRED)
target_link_libraries(lattice_test_node
  lattice_core
  ${YAML_CPP_LIB_PATH}
)
ament_target_dependencies(lattice_test_node
  rclcpp std_msgs nav_msgs visualization_msgs
  geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

# Install
install(TARGETS lattice_core
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS lattice_test_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
