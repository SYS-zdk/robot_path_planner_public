cmake_minimum_required(VERSION 3.0.2)
project(hlpmpccorridor_local_planner)

get_filename_component(PROJECT_ROOT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
include(${PROJECT_ROOT_DIR}/3rd/conanbuildinfo.cmake)
conan_basic_setup()

add_compile_options(-std=c++17)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            base_local_planner
            cmake_modules
            costmap_2d
            dwa_local_planner
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            sensor_msgs
            roscpp
            tf2
            tf2_geometry_msgs
            tf2_ros
            common
            pedsim_msgs
            social_layer
        )

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CONAN_INCLUDE_DIRS_OSQP}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hlpmpccorridor_local_planner
    CATKIN_DEPENDS
        base_local_planner
        dwa_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        roscpp
        tf2
        tf2_ros
        common
)

add_library(hlpmpccorridor_local_planner src/hybrid_planner_ros.cpp src/hybrid_planner.cpp src/Node.cpp src/Edge.cpp src/Helper.cpp src/AStar_Node.cpp src/dynamic_obstacle_cost.cpp src/social_cost_function.cpp)
add_dependencies(hlpmpccorridor_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hlpmpccorridor_local_planner ${catkin_LIBRARIES} ${CONAN_LIBS_OSQP})

add_executable(pedestrian_markers_node src/pedestrian_markers_node.cpp)
target_link_libraries(pedestrian_markers_node ${catkin_LIBRARIES})

install(TARGETS hlpmpccorridor_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES hlpmpccorridor_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
