cmake_minimum_required(VERSION 2.8.3)

project(impedance_joint_control)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs eigen_conversions kdl_conversions cmake_modules)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}  ${Eigen_INCLUDE_DIRS})

orocos_component(impedance_joint_torque src/ImpedanceJointTorque.cpp)
target_link_libraries(impedance_joint_torque ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

orocos_component(impedance_motor_current src/ImpedanceMotorCurrent.cpp)
target_link_libraries(impedance_motor_current ${catkin_LIBRARIES})

orocos_generate_package()
