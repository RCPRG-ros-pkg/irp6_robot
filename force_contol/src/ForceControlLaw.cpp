#include <rtt/Component.hpp>

#include "ForceControlLaw.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

ForceControlLaw::ForceControlLaw(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("CurrentPose", port_current_pose_);
  this->ports()->addPort("OutputPose", port_output_pose_);

  this->ports()->addPort("CurrentWrench", port_current_wrench_);
  this->ports()->addPort("Tool", port_tool_);

}

ForceControlLaw::~ForceControlLaw() {

}

bool ForceControlLaw::configureHook() {

  return true;
}

bool ForceControlLaw::startHook() {
  if (port_current_pose_.read(current_pose_) == RTT::NoData) {
    return false;
  }

  tf::poseMsgToKDL(current_pose_, current_pose_kdl);

  return true;
}

void ForceControlLaw::updateHook() {

  geometry_msgs::Wrench current_wrench;
  port_current_wrench_.read(current_wrench);

  KDL::Wrench input_force;
  tf::wrenchMsgToKDL(current_wrench, input_force);

  // w pierwszej kolejnosci zwiekszam to co krok na x o wartość 0.0001 - dzialalo
  //current_pose_.position.x = current_pose_.position.x + 0.00001;
  // prosty regulator na osi z czujnika
  /*
   current_pose_.position.x = current_pose_.position.x
   + 0.00001 * current_wrench.force.z;
   */

  double kl = -0.000005;

  current_pose_kdl.p[0] = current_pose_kdl.p.x() + kl * input_force.force.x();
  current_pose_kdl.p[1] = current_pose_kdl.p.y() + kl * input_force.force.y();
  current_pose_kdl.p[2] = current_pose_kdl.p.z() + kl * input_force.force.z();

  tf::poseKDLToMsg(current_pose_kdl, current_pose_);

  port_output_pose_.write(current_pose_);

}

ORO_CREATE_COMPONENT(ForceControlLaw)

