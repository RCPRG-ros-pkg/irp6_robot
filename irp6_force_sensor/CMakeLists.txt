cmake_minimum_required(VERSION 2.8.3)

project(irp6_force_sensor)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs cmake_modules)

find_package(Eigen REQUIRED)

find_package(OROCOS-RTT REQUIRED)

catkin_package(
LIBRARIES ${PROJECT_NAME}
)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(ati3084 src/ATI3084.cpp src/ForceSensor.cpp)
target_link_libraries(ati3084 ${catkin_LIBRARIES} comedi)

orocos_component(ati6284 src/ATI6284.cpp src/ForceSensor.cpp)
target_link_libraries(ati6284 ${catkin_LIBRARIES} comedi)

orocos_generate_package()