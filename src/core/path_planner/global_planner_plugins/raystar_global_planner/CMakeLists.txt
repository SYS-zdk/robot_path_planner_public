cmake_minimum_required(VERSION 3.0.2)
project(raystar_global_planner)

find_package(catkin REQUIRED COMPONENTS
  common
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
    common
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

# Optional CGAL support (upstream-equivalent Ray* polymap/visibility)
find_package(CGAL QUIET)

set(RAYSTAR_CGAL_SOURCES "")
set(RAYSTAR_CGAL_LIBS "")

if(CGAL_FOUND)
  message(STATUS "[raystar_global_planner] CGAL found: enabling implementation=upstream_cgal")
  set(RAYSTAR_CGAL_SOURCES src/raystar_cgal_polymap.cpp)
  if(TARGET CGAL::CGAL)
    set(RAYSTAR_CGAL_LIBS CGAL::CGAL)
  else()
    # Fallback for older FindCGAL modules
    include_directories(${CGAL_INCLUDE_DIRS})
    set(RAYSTAR_CGAL_LIBS ${CGAL_LIBRARIES})
  endif()
endif()

add_library(${PROJECT_NAME}
  src/raystar_global_planner.cpp
  src/raystar_planner.cpp
  ${RAYSTAR_CGAL_SOURCES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${RAYSTAR_CGAL_LIBS}
)

if(CGAL_FOUND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC RPP_RAYSTAR_WITH_CGAL=1)
else()
  target_compile_definitions(${PROJECT_NAME} PUBLIC RPP_RAYSTAR_WITH_CGAL=0)
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
