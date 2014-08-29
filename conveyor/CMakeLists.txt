cmake_minimum_required(VERSION 2.8.3)

project(conveyor_kinematic)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs eigen_conversions cmake_modules)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(conveyor_m2j src/ConveyorM2J.cpp)
target_link_libraries(conveyor_m2j ${catkin_LIBRARIES})

orocos_component(conveyor_j2m src/ConveyorJ2M.cpp)
target_link_libraries(conveyor_j2m ${catkin_LIBRARIES})


orocos_generate_package()
