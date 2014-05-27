#include <rtt/Component.hpp>

#include "ForceTransformation.h"
#include "eigen_conversions/eigen_msg.h"

ForceTransformation::ForceTransformation(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      force_sensor_test_mode(false),
      is_reading_ready(false),
      is_right_turn_frame(true),
      gravity_transformation(NULL),
      is_sensor_configured(false) {

  this->ports()->addPort("CurrentPose", port_current_pose_);
  this->ports()->addPort("Tool", port_tool_);

  this->ports()->addPort("CurrentWrench", port_current_wrench_);
  this->ports()->addPort("OutputWrench", port_output_wrench_);

}

ForceTransformation::~ForceTransformation() {

}

bool ForceTransformation::configureHook() {

  return true;
}

bool ForceTransformation::startHook() {
  if (port_current_pose_.read(current_pose_) == RTT::NoData) {
    return false;
  }
  return true;
}

void ForceTransformation::updateHook() {

  geometry_msgs::Wrench current_wrench;
  port_current_wrench_.read(current_wrench);
  port_output_wrench_.write(current_wrench);

}

ORO_CREATE_COMPONENT(ForceTransformation)

