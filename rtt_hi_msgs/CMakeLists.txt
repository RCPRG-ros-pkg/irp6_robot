cmake_minimum_required(VERSION 2.8.3)
project(rtt_hi_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(hi_msgs)
ros_generate_rtt_service_proxies(hi_msgs)

orocos_generate_package(
  DEPENDS hi_msgs
  DEPENDS_TARGETS rtt_roscomm  rtt_std_msgs
 rtt_rosgraph_msgs

)
