cmake_minimum_required(VERSION 3.0.2)
project(social_layer)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  pedsim_msgs
  nav_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES social_layer
  CATKIN_DEPENDS costmap_2d pedsim_msgs nav_msgs roscpp 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/social_layer.cpp
)

# Ensure message headers from dependencies (e.g., pedsim_msgs) are generated
# before compiling this package (important for clean builds).
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(social_layer_local_debug_node
  src/social_layer_local_debug_node.cpp
)

add_dependencies(social_layer_local_debug_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(social_layer_local_debug_node
  ${catkin_LIBRARIES}
)

add_executable(social_layer_global_debug_node
  src/social_layer_global_debug_node.cpp
)

add_dependencies(social_layer_global_debug_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(social_layer_global_debug_node
  ${catkin_LIBRARIES}
)
