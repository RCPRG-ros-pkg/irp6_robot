cmake_minimum_required(VERSION 2.8.3)

project(data_export)

find_package(catkin REQUIRED COMPONENTS hi_msgs rtt_ros cmake_modules)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
 )
  
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${USE_OROCOS_LIBRARY_DIRS})

catkin_package(
  DEPENDS rtt 
  CATKIN_DEPENDS rtt_ros 
)

orocos_component(${PROJECT_NAME} src/DataExport.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_generate_package()

