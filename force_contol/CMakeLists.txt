cmake_minimum_required(VERSION 2.8.3)

project(force_contol)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs eigen_conversions)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}  ${Eigen_INCLUDE_DIRS})

orocos_component(force_control_law src/ForceControlLaw.cpp)
target_link_libraries(force_control_law ${catkin_LIBRARIES})



orocos_generate_package()
