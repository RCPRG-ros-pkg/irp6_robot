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

void ForceControlLaw::updateHook() {
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

