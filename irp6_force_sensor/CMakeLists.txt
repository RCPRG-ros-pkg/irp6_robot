cmake_minimum_required(VERSION 2.8.3)

project(irp6_force_sensor)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs)

find_package(Eigen3 REQUIRED)

find_package(OROCOS-RTT REQUIRED)

catkin_package(
LIBRARIES ${PROJECT_NAME}
)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

orocos_component(ATI3084 src/ATI3084.cpp src/ForceSensor.cpp)
target_link_libraries(ATI3084 ${catkin_LIBRARIES} comedi)

orocos_component(ATI6284 src/ATI6284.cpp src/ForceSensor.cpp)
target_link_libraries(ATI6284 ${catkin_LIBRARIES} comedi)

orocos_generate_package()