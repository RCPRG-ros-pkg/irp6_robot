#include <rtt/Component.hpp>

#include "ForceTransformation.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

ForceTransformation::ForceTransformation(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      gravity_transformation_(NULL) {

  this->ports()->addPort("CurrentWristPose", port_current_wrist_pose_);

  this->ports()->addPort("CurrentSensorWrench", port_current_sensor_wrench_);
  this->ports()->addPort("OutputWristWrench", port_output_wrist_wrench_);
  this->ports()->addPort("OutputEndEffectorWrench",
                         port_output_end_effector_wrench_);
  this->ports()->addPort("Tool", port_tool_);
}

ForceTransformation::~ForceTransformation() {

}

bool ForceTransformation::configureHook() {

  return true;
}

bool ForceTransformation::startHook() {
  geometry_msgs::Pose current_wrist_pose;
  if (port_current_wrist_pose_.read(current_wrist_pose) == RTT::NoData) {
    return false;
  }

  // read current force
  geometry_msgs::Wrench current_wrench;
  if (port_current_sensor_wrench_.read(current_wrench) == RTT::NoData) {
    return false;
  }

  tf::wrenchMsgToKDL(current_wrench, force_offset_);

  // polozenie nadgarstka narzedzia wzgledem bazy
  if (port_current_wrist_pose_.read(current_wrist_pose) == RTT::NoData) {
    return false;
  }

  KDL::Frame current_frame;

  tf::poseMsgToKDL(current_wrist_pose, current_frame);

  if (!gravity_transformation_)  // nie powolano jeszcze obiektu
  {

    KDL::Frame force_sensor_frame = KDL::Frame(KDL::Rotation::RotZ(M_PI),
                                               KDL::Vector(0.0, 0.0, 0.09));
    double weight = 10.8;

    KDL::Vector pointofgravity(0.004, 0.0, 0.156);

    gravity_transformation_ = new ForceTrans(current_frame, force_sensor_frame,
                                             weight, pointofgravity, true);

  } else {
    gravity_transformation_->synchro(current_frame);
  }

  return true;
}

void ForceTransformation::updateHook() {
  geometry_msgs::Pose current_wrist_pose;
  port_current_wrist_pose_.read(current_wrist_pose);
  KDL::Frame current_wrist_pose_kdl;
  tf::poseMsgToKDL(current_wrist_pose, current_wrist_pose_kdl);

  // odczyt sily
  geometry_msgs::Wrench current_wrench;
  port_current_sensor_wrench_.read(current_wrench);
  KDL::Wrench input_force;
  tf::wrenchMsgToKDL(current_wrench, input_force);

  // offset level removal
  KDL::Wrench biased_force = input_force - force_offset_;

  //sily przechowujemy w zerowej orientacji bazowej w ukladzie nadgarstka
  KDL::Wrench computed_force = gravity_transformation_->getForce(biased_force,
                                                                 current_wrist_pose_kdl);
  geometry_msgs::Wrench output_wrist_wrench;
  tf::wrenchKDLToMsg(computed_force, output_wrist_wrench);

  port_output_wrist_wrench_.write(output_wrist_wrench);

  //tool determination
  geometry_msgs::Pose tool_msgs;
  port_tool_.read(tool_msgs);
  KDL::Frame tool_kdl;
  tf::poseMsgToKDL(tool_msgs, tool_kdl);

  KDL::Wrench computed_ef_force = tool_kdl.Inverse()
      * (current_wrist_pose_kdl.M.Inverse() * computed_force);

  geometry_msgs::Wrench output_end_effector_wrench;
  tf::wrenchKDLToMsg(computed_ef_force, output_end_effector_wrench);

  port_output_end_effector_wrench_.write(output_end_effector_wrench);

}

ORO_CREATE_COMPONENT(ForceTransformation)

