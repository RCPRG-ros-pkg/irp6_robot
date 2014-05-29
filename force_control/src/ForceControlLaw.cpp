#include <rtt/Component.hpp>

#include "ForceControlLaw.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

ForceControlLaw::ForceControlLaw(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("CurrentEndEffectorPose",
                         port_current_end_effector_pose_);
  this->ports()->addPort("OutputEndEffectorPose",
                         port_output_end_effector_pose_);
  this->ports()->addPort("CurrentEndEffectorWrench",
                         port_current_end_effector_wrench_);
  this->ports()->addPort("CurrentFclParam",
                         port_current_fcl_param_);

}

ForceControlLaw::~ForceControlLaw() {

}

bool ForceControlLaw::configureHook() {

  return true;
}

bool ForceControlLaw::startHook() {

  //tool determination

  geometry_msgs::Pose cl_ef_pose;
  if (port_current_end_effector_pose_.read(cl_ef_pose) == RTT::NoData) {
    return false;
  }

  tf::poseMsgToKDL(cl_ef_pose, cl_ef_pose_kdl_);

  force_control_msgs::ForceControl fcl_param;
 //controller parameters have to be set before the component is started
  if (port_current_fcl_param_.read(fcl_param) == RTT::NoData) {
    return false;
  }


  return true;
}

void ForceControlLaw::updateHook() {

  // current wrench determination
  geometry_msgs::Wrench current_end_effector_wrench;
  port_current_end_effector_wrench_.read(current_end_effector_wrench);
  KDL::Wrench input_force;
  tf::wrenchMsgToKDL(current_end_effector_wrench, input_force);

  force_control_msgs::ForceControl fcl_param;
  port_current_fcl_param_.read(fcl_param);

  KDL::Twist target_vel;

  target_vel.vel[0] = fcl_param.reciprocaldamping.translation.x * (-input_force.force.x());
  target_vel.vel[1] = fcl_param.reciprocaldamping.translation.y * (-input_force.force.y());
  target_vel.vel[2] = fcl_param.reciprocaldamping.translation.z * (-input_force.force.z());

  target_vel.rot[0] = fcl_param.reciprocaldamping.rotation.x * (-input_force.torque.x());
  target_vel.rot[1] = fcl_param.reciprocaldamping.rotation.y * (-input_force.torque.y());
  target_vel.rot[2] = fcl_param.reciprocaldamping.rotation.z * (-input_force.torque.z());

  target_vel = cl_ef_pose_kdl_.M * target_vel;

  cl_ef_pose_kdl_ = KDL::addDelta(cl_ef_pose_kdl_, target_vel, 1.0);

  geometry_msgs::Pose cl_ef_pose;

  tf::poseKDLToMsg(cl_ef_pose_kdl_, cl_ef_pose);

  port_output_end_effector_pose_.write(cl_ef_pose);

}

ORO_CREATE_COMPONENT(ForceControlLaw)

