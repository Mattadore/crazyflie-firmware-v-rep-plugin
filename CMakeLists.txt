cmake_minimum_required(VERSION 2.8.3)

project(controlplugin)
find_package(catkin REQUIRED COMPONENTS quad_simulation)
catkin_package(CATKIN_DEPENDS quad_simulation)
add_definitions(-DCOMPILE_AS_SIM_)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(v_repExtQuadControlPlugin src/controlplugin.cpp src/v_repLib.cpp)

target_link_libraries(v_repExtQuadControlPlugin ${quad_simulation_LIBRARY} boost_system  ${catkin_LIBRARIES})
