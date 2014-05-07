#include <rtt/Component.hpp>

#include "ForceControlLaw.h"
#include "eigen_conversions/eigen_msg.h"

ForceControlLaw::ForceControlLaw(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

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
  if(port_current_pose_.read(current_pose_) == RTT::NoData) {
    return false;
  }
  return true;
}

void ForceControlLaw::updateHook() {
  // w pierwszej kolejnosci zwiekszam to co krok na x o wartość 0.0001 - dzialalo
  // current_pose_.position.x = current_pose_.position.x + 0.00001;

  port_output_pose_.write(current_pose_);


/*
  if (port_joint_position_.read(joint_position_) == RTT::NewData) {

    Eigen::Affine3d trans;
    geometry_msgs::Pose pos;

    direct_kinematics_transform(joint_position_, &trans);

    tf::poseEigenToMsg(trans, pos);
    port_output_pose_.write(pos);
  }
  */
}


ORO_CREATE_COMPONENT(ForceControlLaw)

