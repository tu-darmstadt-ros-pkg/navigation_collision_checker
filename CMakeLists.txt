cmake_minimum_required(VERSION 2.8.3)
project(navigation_collision_checker)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  geometry_msgs
  moveit_core
  moveit_ros_planning
  roscpp
  sensor_msgs
  octomap_msgs
  visualization_msgs
  tf
  dynamic_reconfigure
  grid_map_core
  grid_map_msgs
  grid_map_ros
  grid_map_proc
  vigir_worldmodel_server
)

generate_dynamic_reconfigure_options(cfg/NavCollisionChecker.cfg)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigation_collision_checker_node src/navigation_collision_checker_node.cpp)
target_link_libraries(navigation_collision_checker_node ${catkin_LIBRARIES})
add_dependencies(navigation_collision_checker_node ${${PROJECT_NAME}_EXPORTED_TARGETS})


