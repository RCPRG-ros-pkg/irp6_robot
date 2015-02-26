cmake_minimum_required(VERSION 2.8.3)

project(sarkofag_kinematic)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs eigen_conversions cmake_modules)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(sarkofag_m2j src/SarkofagM2J.cpp)
target_link_libraries(sarkofag_m2j ${catkin_LIBRARIES})

orocos_component(sarkofag_j2m src/SarkofagJ2M.cpp)
target_link_libraries(sarkofag_j2m ${catkin_LIBRARIES})


orocos_generate_package()
