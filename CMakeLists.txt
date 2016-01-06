cmake_minimum_required(VERSION 2.8.3)
project(navigation_collision_checker)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
)

generate_dynamic_reconfigure_options(cfg/NavCollisionChecker.cfg)


find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})


add_executable(navigation_collision_checker_node src/navigation_collision_checker_node.cpp)
add_dependencies(navigation_collision_checker_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(navigation_collision_checker_node
   ${catkin_LIBRARIES}
)

catkin_package(
INCLUDE_DIRS
  include
LIBRARIES
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
CATKIN_DEPENDS eigen_conversions geometry_msgs moveit_core roscpp sensor_msgs tf
#DEPENDS system_lib
)