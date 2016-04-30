cmake_minimum_required(VERSION 2.8.3)
project(irp6_supervisor)

find_package(catkin REQUIRED COMPONENTS common_headers rtt_ros cmake_modules)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(Eigen REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS})
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${USE_OROCOS_LIBRARY_DIRS})

orocos_component(irp6ot_supervisor src/Irp6otSupervisor.cpp)
target_link_libraries(irp6ot_supervisor ${catkin_LIBRARIES})

orocos_component(irp6p_supervisor src/Irp6pSupervisor.cpp)
target_link_libraries(irp6p_supervisor ${catkin_LIBRARIES})

orocos_component(conv_supervisor src/ConvSupervisor.cpp)
target_link_libraries(conv_supervisor ${catkin_LIBRARIES})

orocos_generate_package()

