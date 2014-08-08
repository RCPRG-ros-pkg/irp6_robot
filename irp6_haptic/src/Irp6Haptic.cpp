// Copyright WUT 2014
#include <rtt/Component.hpp>

#include "Irp6Haptic.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

Irp6Haptic::Irp6Haptic(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      step_duration_(0.002) {

  this->ports()->addPort("CurrentEndEffectorPose",
                         port_current_end_effector_pose_);
  this->ports()->addPort("OutputEndEffectorPose",
                         port_output_end_effector_pose_);
  this->ports()->addPort("CurrentEndEffectorWrench",
                         port_current_end_effector_wrench_);
  this->ports()->addPort("OutputFclParam", port_output_fcl_param_);
}

Irp6Haptic::~Irp6Haptic() {
}

bool Irp6Haptic::configureHook() {
  return true;
}

bool Irp6Haptic::startHook() {

  return true;
}

void Irp6Haptic::updateHook() {

  // current wrench determination
  geometry_msgs::Wrench current_end_effector_wrench;
  port_current_end_effector_wrench_.read(current_end_effector_wrench);

  force_control_msgs::ForceControl fcl_param;

  fcl_param.reciprocaldamping.translation.x = 0.002;
  fcl_param.inertia.translation.x = 0.0;
  fcl_param.wrench.force.x = -current_end_effector_wrench.force.x;
  fcl_param.twist.linear.x = 0.0;

  fcl_param.reciprocaldamping.translation.y = 0.002;
  fcl_param.inertia.translation.y = 0.0;
  fcl_param.wrench.force.y = -current_end_effector_wrench.force.y;
  fcl_param.twist.linear.y = 0.0;

  fcl_param.reciprocaldamping.translation.z = 0.002;
  fcl_param.inertia.translation.z = 0.0;
  fcl_param.wrench.force.z = -current_end_effector_wrench.force.z;
  fcl_param.twist.linear.z = 0.0;

  fcl_param.reciprocaldamping.rotation.x = 0.05;
  fcl_param.inertia.rotation.x = 0.0;
  fcl_param.wrench.torque.x = -current_end_effector_wrench.torque.x;
  fcl_param.twist.angular.x = 0.0;

  fcl_param.reciprocaldamping.rotation.y = 0.05;
  fcl_param.inertia.rotation.y = 0.0;
  fcl_param.wrench.torque.y = -current_end_effector_wrench.torque.y;
  fcl_param.twist.angular.y = 0.0;

  fcl_param.reciprocaldamping.rotation.z = 0.05;
  fcl_param.inertia.rotation.z = 0.0;
  fcl_param.wrench.torque.z = -current_end_effector_wrench.torque.z;
  fcl_param.twist.angular.z = 0.0;

  port_output_fcl_param_.write(fcl_param);

  geometry_msgs::Pose cl_ef_pose;
  port_current_end_effector_pose_.read(cl_ef_pose);
  port_output_end_effector_pose_.write(cl_ef_pose);

}


ORO_CREATE_COMPONENT(Irp6Haptic)

