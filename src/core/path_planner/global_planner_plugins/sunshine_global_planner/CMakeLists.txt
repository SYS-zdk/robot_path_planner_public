cmake_minimum_required(VERSION 3.0.2)
project(sunshine_global_planner)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  path_planner
  path_planner_global_plugin_common
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    path_planner
    path_planner_global_plugin_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sunshine_global_planner.cpp
  src/sunshine_planner.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
