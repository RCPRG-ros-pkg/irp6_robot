cmake_minimum_required(VERSION 2.8.3)

project(irp6_haptic)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs eigen_conversions kdl_conversions force_control_msgs cmake_modules)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}  ${Eigen_INCLUDE_DIRS})

orocos_component(irp6_haptic src/Irp6Haptic.cpp)
target_link_libraries(irp6_haptic ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})


orocos_generate_package()
