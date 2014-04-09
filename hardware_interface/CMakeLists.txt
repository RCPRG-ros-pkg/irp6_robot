cmake_minimum_required(VERSION 2.8.6)
project(hardware_interface)

find_package(catkin REQUIRED)

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

orocos_component(${PROJECT_NAME} src/HardwareInterface.cpp src/hi_moxa.cpp src/nf/nfv2.c src/serialcomm/serialcomm.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_component(irp6p_transmission src/irp6p_transmission.cpp)
target_link_libraries(irp6p_transmission ${catkin_LIBRARIES})

orocos_component(irp6p_transmission_inv src/irp6p_transmission_inv.cpp)
target_link_libraries(irp6p_transmission ${catkin_LIBRARIES})

orocos_generate_package()

