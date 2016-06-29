cmake_minimum_required(VERSION 2.8.3)
project(controlplugin)
find_package(catkin)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(v_repExtQuadControlPlugin src/controlplugin.cpp src/v_repLib.cpp)

target_link_libraries(v_repExtQuadControlPlugin boost_system  ${catkin_LIBRARIES})
