cmake_minimum_required(VERSION 3.0.2)
project(localreachability_layer)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	costmap_2d
	dynamic_reconfigure
	nav_msgs
	sensor_msgs
	pluginlib
	roscpp
	distance_layer
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES localreachability_layer
	CATKIN_DEPENDS costmap_2d dynamic_reconfigure nav_msgs sensor_msgs pluginlib roscpp distance_layer
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/localreachability_layer.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES localreachability_layer_costmap_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
