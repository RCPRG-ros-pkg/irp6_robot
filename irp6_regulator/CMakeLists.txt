cmake_minimum_required(VERSION 2.8.3)

project(irp6_regulator)

find_package(catkin REQUIRED COMPONENTS rtt_ros)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  DEPENDS rtt 
  CATKIN_DEPENDS rtt_ros 
)

orocos_component(${PROJECT_NAME} src/IRp6Regulator.cpp src/Regulator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_generate_package()

