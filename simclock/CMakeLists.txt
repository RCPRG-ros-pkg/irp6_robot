cmake_minimum_required(VERSION 2.8.3)

project(simclock)

find_package(catkin REQUIRED COMPONENTS rtt_ros)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

orocos_component(simclock src/SimClock.cpp)
target_link_libraries(simclock ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(simclock ${catkin_EXPORTED_TARGETS})


orocos_generate_package()
