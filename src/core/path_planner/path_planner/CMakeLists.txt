cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

 SET(CMAKE_BUILD_TYPE "Debug")
 SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
 SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  tf2_geometry_msgs
  tf2_ros
  trajectory_planner
  common
  path_processor
  std_msgs
  voronoi_layer
  globalreachability_layer
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS common path_processor trajectory_planner std_msgs voronoi_layer globalreachability_layer
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB path_planner_src_files CONFIGURE_DEPENDS src/*.cpp)
file(GLOB graph_path_planner_src_files CONFIGURE_DEPENDS src/graph_planner/*.cpp)
set(graph_path_planner_hybrid_astar_src_files
  src/graph_planner/hybrid_astar_planner/hybrid_astar_planner_impl.cpp
  src/graph_planner/hybrid_astar_planner/astar_framework.cpp
  src/graph_planner/hybrid_astar_planner/analytic_expansion.cpp
  src/graph_planner/hybrid_astar_planner/node_hybrid.cpp
  src/graph_planner/hybrid_astar_planner/node_wrapper.cpp
)
file(GLOB sample_path_planner_src_files CONFIGURE_DEPENDS src/sample_planner/*.cpp)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${path_planner_src_files}
  ${graph_path_planner_src_files}
  ${graph_path_planner_hybrid_astar_src_files}
  ${sample_path_planner_src_files}
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

install(FILES path_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
