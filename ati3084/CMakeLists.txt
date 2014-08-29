cmake_minimum_required(VERSION 2.8.3)

project(ati3084)

find_package(catkin REQUIRED COMPONENTS rtt_ros force_sensor cmake_modules)
find_package(Eigen REQUIRED)
find_package(OROCOS-RTT REQUIRED)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(${PROJECT_NAME} src/ATI3084.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} comedi)

orocos_generate_package()
