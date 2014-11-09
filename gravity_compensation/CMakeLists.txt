cmake_minimum_required(VERSION 2.8.3)

project(gravity_compensation)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs eigen_conversions kdl_conversions cmake_modules)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}  ${Eigen_INCLUDE_DIRS})

orocos_component(gravity_joint_torque src/GravityJointTorque.cpp)
target_link_libraries(gravity_joint_torque ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

orocos_component(gravity_motor_current src/GravityMotorCurrent.cpp)
target_link_libraries(gravity_motor_current ${catkin_LIBRARIES})

orocos_generate_package()
