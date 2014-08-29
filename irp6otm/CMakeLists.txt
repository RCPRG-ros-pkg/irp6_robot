cmake_minimum_required(VERSION 2.8.3)

project(irp6otm_kinematic)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs eigen_conversions cmake_modules)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(irp6otm_m2j src/Irp6otmM2J.cpp)
target_link_libraries(irp6otm_m2j ${catkin_LIBRARIES})

orocos_component(irp6otm_j2m src/Irp6otmJ2M.cpp)
target_link_libraries(irp6otm_j2m ${catkin_LIBRARIES})


orocos_component(irp6otm_forward_kinematic src/Irp6otmForwardKinematic.cpp)
target_link_libraries(irp6otm_forward_kinematic ${catkin_LIBRARIES})

orocos_component(irp6otm_inverse_kinematic src/Irp6otmInverseKinematic.cpp)
target_link_libraries(irp6otm_inverse_kinematic ${catkin_LIBRARIES})


orocos_generate_package()
