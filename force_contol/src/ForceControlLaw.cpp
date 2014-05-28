#include <rtt/Component.hpp>

#include "ForceControlLaw.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

ForceControlLaw::ForceControlLaw(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("CurrentWristPose", port_current_wrist_pose_);
  this->ports()->addPort("OutputWristPose", port_output_wrist_pose_);

  this->ports()->addPort("CurrentWristWrench", port_current_wrist_wrench_);
  this->ports()->addPort("Tool", port_tool_);

}

ForceControlLaw::~ForceControlLaw() {

}

bool ForceControlLaw::configureHook() {

  return true;
}

bool ForceControlLaw::startHook() {
  if (port_current_wrist_pose_.read(cl_wrist_pose_) == RTT::NoData) {
    return false;
  }

  tf::poseMsgToKDL(cl_wrist_pose_, cl_wrist_pose_kdl_);

  return true;
}

void ForceControlLaw::updateHook() {

  // current wrench determination
  geometry_msgs::Wrench current_wrist_wrench;
  port_current_wrist_wrench_.read(current_wrist_wrench);
  KDL::Wrench input_force;
  tf::wrenchMsgToKDL(current_wrist_wrench, input_force);

  //tool determination
  geometry_msgs::Pose tool_msgs;
  port_tool_.read(tool_msgs);
  KDL::Frame tool_kdl;
  tf::poseMsgToKDL(tool_msgs, tool_kdl);

  // current wrist pose determination
  geometry_msgs::Pose current_wrist_pose;
  port_current_wrist_pose_.read(current_wrist_pose);
  KDL::Frame current_wrist_pose_kdl;
  tf::poseMsgToKDL(current_wrist_pose, current_wrist_pose_kdl);

  //conversion of wrist wrench to the end effector

  double kl = -0.000005;
  double kr = -0.0001;

  KDL::Twist target_vel;

  target_vel.vel[0] = kl * input_force.force.x();
  target_vel.vel[1] = kl * input_force.force.y();
  target_vel.vel[2] = kl * input_force.force.z();

  target_vel.rot[0] = kr * input_force.torque.x();
  target_vel.rot[1] = kr * input_force.torque.y();
  target_vel.rot[2] = kr * input_force.torque.z();

  cl_wrist_pose_kdl_ = KDL::addDelta(cl_wrist_pose_kdl_, target_vel, 1.0);

  tf::poseKDLToMsg(cl_wrist_pose_kdl_, cl_wrist_pose_);

  port_output_wrist_pose_.write(cl_wrist_pose_);

}

ORO_CREATE_COMPONENT(ForceControlLaw)

