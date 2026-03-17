set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")
find_package(Python3 COMPONENTS Development NumPy)

catkin_add_executable_with_gtest(test_convex_safety_corridor test_convex_safety_corridor.cpp)
if(TARGET test_convex_safety_corridor)
  target_link_libraries(test_convex_safety_corridor
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    Python3::Python
    Python3::NumPy
    common
  )
endif()
