cmake_minimum_required(VERSION 3.14)
project(robot_path_planner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(path_planner_lib
    src/grid_map.cpp
    src/astar.cpp
    src/path_planner.cpp
)
target_include_directories(path_planner_lib PUBLIC include)

# ---------------------------------------------------------------------------
# Example executable
# ---------------------------------------------------------------------------
add_executable(path_planner_example examples/main.cpp)
target_link_libraries(path_planner_example PRIVATE path_planner_lib)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
enable_testing()
add_subdirectory(test)
