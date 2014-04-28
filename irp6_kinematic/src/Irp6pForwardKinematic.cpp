#include <rtt/Component.hpp>

#include "Irp6pForwardKinematic.h"
#include "eigen_conversions/eigen_msg.h"

Irp6pForwardKinematic::Irp6pForwardKinematic(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("JointPosition", port_joint_position_);
  this->ports()->addPort("OutputPose", port_output_pose_);

}

Irp6pForwardKinematic::~Irp6pForwardKinematic() {

}

bool Irp6pForwardKinematic::configureHook() {

  joint_position_.resize(6);
  return true;
}

void Irp6pForwardKinematic::updateHook() {

  if (port_joint_position_.read(joint_position_) == RTT::NewData) {

    Eigen::Affine3d trans;
    geometry_msgs::Pose pos;

    direct_kinematics_transform(joint_position_, &trans);

    tf::poseEigenToMsg(trans, pos);
    port_output_pose_.write(pos);
  }
}

void Irp6pForwardKinematic::direct_kinematics_transform(
    const Eigen::VectorXd& local_current_joints,
    Eigen::Affine3d* local_current_end_effector_frame) {

}

ORO_CREATE_COMPONENT(Irp6pForwardKinematic)

