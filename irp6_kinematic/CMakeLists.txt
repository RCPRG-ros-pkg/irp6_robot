cmake_minimum_required(VERSION 2.8.3)

project(irp6p_kinematic)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs eigen_conversions)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}  ${Eigen_INCLUDE_DIRS})

orocos_component(irp6p_forward_kinematic src/Irp6pForwardKinematic.cpp)
target_link_libraries(irp6p_forward_kinematic ${catkin_LIBRARIES})

orocos_component(irp6p_inverse_kinematic src/Irp6pInverseKinematic.cpp)
target_link_libraries(irp6p_inverse_kinematic ${catkin_LIBRARIES})


orocos_generate_package()
