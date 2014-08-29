cmake_minimum_required(VERSION 2.8.3)

project(irp6pm_kinematic)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs eigen_conversions cmake_modules)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(irp6pm_m2j src/Irp6pmM2J.cpp)
target_link_libraries(irp6pm_m2j ${catkin_LIBRARIES})

orocos_component(irp6pm_j2m src/Irp6pmJ2M.cpp)
target_link_libraries(irp6pm_j2m ${catkin_LIBRARIES})


orocos_component(irp6pm_forward_kinematic src/Irp6pmForwardKinematic.cpp)
target_link_libraries(irp6pm_forward_kinematic ${catkin_LIBRARIES})

orocos_component(irp6pm_inverse_kinematic src/Irp6pmInverseKinematic.cpp)
target_link_libraries(irp6pm_inverse_kinematic ${catkin_LIBRARIES})


orocos_generate_package()
