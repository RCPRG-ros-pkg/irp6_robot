#include <rtt/Component.hpp>

#include "ForceControlLaw.h"
#include "eigen_conversions/eigen_msg.h"

ForceControlLaw::ForceControlLaw(const std::string& name)
    : RTT::TaskContext(name, PreOperational)
{

  this->ports()->addPort("CurrentPose", port_current_pose_);
  this->ports()->addPort("OutputPose", port_output_pose_);

  this->ports()->addPort("CurrentWrench", port_current_wrench_);

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
  return true;
}

void ForceControlLaw::updateHook() {

  geometry_msgs::Wrench current_wrench;
  port_current_wrench_.read(current_wrench);
  // w pierwszej kolejnosci zwiekszam to co krok na x o wartość 0.0001 - dzialalo
  //current_pose_.position.x = current_pose_.position.x + 0.00001;
  // prosty regulator na osi z czujnika
  current_pose_.position.x = current_pose_.position.x
      + 0.00001 * current_wrench.force.z;
  port_output_pose_.write(current_pose_);

}

ORO_CREATE_COMPONENT(ForceControlLaw)

