cmake_minimum_required(VERSION 2.8.3)

project(irp6_regulator)

find_package(catkin REQUIRED COMPONENTS common_headers rtt_ros cmake_modules)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS})

orocos_component(${PROJECT_NAME} src/IRp6Regulator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_generate_package()

