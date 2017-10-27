cmake_minimum_required(VERSION 2.8.3)
project(simclock_singleton)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_ros rospack rostime cmake_modules rosgraph_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

orocos_library(simclock_singleton 
  src/simclock_singleton.cpp
  src/simclock_singleton_interface.cpp)
target_link_libraries(simclock_singleton ${catkin_LIBRARIES})


orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rostime
)

orocos_install_headers(
  DIRECTORY include/${PROJECT_NAME}/
)
