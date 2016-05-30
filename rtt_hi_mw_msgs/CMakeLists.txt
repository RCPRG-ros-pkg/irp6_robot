cmake_minimum_required(VERSION 2.8.3)
project(rtt_hi_mw_msgs)

find_package(catkin REQUIRED COMPONENTS hi_mw_msgs rtt_roscomm rtt_std_msgs rtt_rosgraph_msgs)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS})

ros_generate_rtt_typekit(hi_mw_msgs)
ros_generate_rtt_service_proxies(hi_mw_msgs)

orocos_generate_package(
  DEPENDS hi_mw_msgs
  DEPENDS_TARGETS rtt_roscomm  rtt_std_msgs
 rtt_rosgraph_msgs

)
