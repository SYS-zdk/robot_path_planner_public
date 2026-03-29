cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

 SET(CMAKE_BUILD_TYPE "Debug")
 SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
 SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  distance_layer
  geometry_msgs
  nav_msgs
  navfn
  tf2_geometry_msgs
  tf2_ros
  trajectory_planner
  common
  std_msgs
  visualization_msgs
  voronoi_layer
  globalreachability_layer
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
   angles
   roscpp
   costmap_2d
   geometry_msgs
   nav_msgs
   tf2_geometry_msgs
   tf2_ros
   common
   trajectory_planner
   std_msgs
   visualization_msgs
   voronoi_layer
   globalreachability_layer
   distance_layer
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(path_planner_src_files
  src/path_planner.cpp
  # NOTE: Implements PathPlannerEngine (shared by global planner plugins).
  src/path_planner_engine.cpp
  src/path_planner_ros_io.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${path_planner_src_files}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  trajectory_optimization
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


