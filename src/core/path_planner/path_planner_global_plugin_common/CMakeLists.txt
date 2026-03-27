cmake_minimum_required(VERSION 3.0.2)
project(path_planner_global_plugin_common)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  nav_core
  roscpp
  path_planner
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    costmap_2d
    nav_core
    roscpp
    path_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
