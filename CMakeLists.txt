cmake_minimum_required(VERSION 2.8.3)
project(robot_kinematic_simulation)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  urdf
  sensor_msgs
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp urdf sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kinematic_simulation src/kinematic_simulation.cpp)
target_link_libraries(kinematic_simulation ${catkin_LIBRARIES})
add_dependencies(kinematic_simulation ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
