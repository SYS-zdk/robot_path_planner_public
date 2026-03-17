cmake_minimum_required(VERSION 3.0.2)
project(ilqr_controller)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  distance_layer
  geometry_msgs
  nav_core
  nav_msgs
  visualization_msgs
  navfn
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
  base_local_planner
  controller
  common
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller distance_layer common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ilqr_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
