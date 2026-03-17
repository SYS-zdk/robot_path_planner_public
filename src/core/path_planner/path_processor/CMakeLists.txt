cmake_minimum_required(VERSION 3.0.2)
project(path_processor)

find_package(catkin REQUIRED COMPONENTS
  common
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_processor
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB path_processor_src_fils src/path_processor/*.cpp)
file(GLOB path_prune_src_fils src/path_prune/*.cpp)
file(GLOB path_smooth_src_fils src/path_smooth/*.cpp)

add_library(${PROJECT_NAME}
  ${path_prune_src_fils}
  ${path_processor_src_fils}
  ${path_smooth_src_fils}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

set(BUILDING_WITH_TEST Off)
if (${BUILDING_WITH_TEST})
  add_subdirectory(test)
endif()