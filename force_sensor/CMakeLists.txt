cmake_minimum_required(VERSION 2.8.3)

project(force_sensor)

find_package(catkin REQUIRED COMPONENTS rtt_ros cmake_modules)

find_package(Eigen REQUIRED)

find_package(OROCOS-RTT REQUIRED)

catkin_package(
LIBRARIES ${PROJECT_NAME}
)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/ForceSensor.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

