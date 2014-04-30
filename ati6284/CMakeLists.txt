cmake_minimum_required(VERSION 2.8.3)

project(ati6284)

find_package(catkin REQUIRED COMPONENTS rtt_ros)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_component(${PROJECT_NAME} src/ATI6284.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} comedi)

orocos_generate_package()

