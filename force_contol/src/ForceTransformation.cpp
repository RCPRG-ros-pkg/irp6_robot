#include <rtt/Component.hpp>

#include "ForceTransformation.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

ForceTransformation::ForceTransformation(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      force_sensor_test_mode(false),
      is_reading_ready(false),
      is_right_turn_frame(true),
      gravity_transformation(NULL),
      is_sensor_configured(false) {

  this->ports()->addPort("CurrentWristPose", port_current_wrist_pose_);
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
  if (port_current_wrist_pose_.read(current_wrist_pose_) == RTT::NoData) {
    return false;
  }

  is_sensor_configured = true;

  // force offset determination
  if (!force_sensor_test_mode) {
    // read current force
    geometry_msgs::Wrench current_wrench;
    if (port_current_wrench_.read(current_wrench) == RTT::NoData) {
      return false;
    }

    tf::wrenchMsgToKDL(current_wrench, force_offset);

  }

  // polozenie nadgarstka narzedzia wzgledem bazy
  if (port_current_wrist_pose_.read(current_wrist_pose_) == RTT::NoData) {
    return false;
  }

  KDL::Frame current_frame;

  tf::poseMsgToKDL(current_wrist_pose_, current_frame);

  if (!gravity_transformation)  // nie powolano jeszcze obiektu
  {
    //   irp6 postument force sensor physical parameters
    //   force_sensor_in_wrist=0.0 0.0 0.09 0.0 0.0 3.14159
    //   default_mass_center_in_wrist=0.004 0.0 0.156
    //   weight=10.8

    force_sensor_frame = KDL::Frame(KDL::Rotation::RotZ(M_PI),
                                    KDL::Vector(0.0, 0.0, 0.09));

    double weight = 10.8;

    KDL::Vector pointofgravity(0.004, 0.0, 0.156);

    tool_mass_center_translation = KDL::Frame(pointofgravity);

    gravity_transformation = new ForceTrans(current_frame, force_sensor_frame,
                                            weight, pointofgravity,
                                            is_right_turn_frame);

  } else {
    gravity_transformation->synchro(current_frame);
  }

  return true;
}

void ForceTransformation::updateHook() {

  geometry_msgs::Wrench current_wrench;
  port_current_wrench_.read(current_wrench);
  port_output_wrench_.write(current_wrench);

}

ORO_CREATE_COMPONENT(ForceTransformation)

