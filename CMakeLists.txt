cmake_minimum_required(VERSION 2.8.3)
project(robot_kinematic_simulation)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  urdf
  rosgraph_msgs
  sensor_msgs
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp urdf sensor_msgs rosgraph_msgs
  INCLUDE_DIRS include
  LIBRARIES kinematic_simulation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(kinematic_simulation src/kinematic_simulation.cpp)
target_link_libraries(kinematic_simulation ${catkin_LIBRARIES})
add_dependencies(kinematic_simulation ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(kinematic_simulation_node src/node_sim.cpp)
target_link_libraries(kinematic_simulation_node kinematic_simulation ${catkin_LIBRARIES})
add_dependencies(kinematic_simulation_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
