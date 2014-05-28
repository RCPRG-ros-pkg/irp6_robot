#include <rtt/Component.hpp>

#include "ForceTransformation.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

ForceTransformation::ForceTransformation(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      gravity_transformation(NULL) {

  this->ports()->addPort("CurrentWristPose", port_current_wrist_pose_);

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

  // read current force
  geometry_msgs::Wrench current_wrench;
  if (port_current_wrench_.read(current_wrench) == RTT::NoData) {
    return false;
  }

  tf::wrenchMsgToKDL(current_wrench, force_offset);

  // polozenie nadgarstka narzedzia wzgledem bazy
  if (port_current_wrist_pose_.read(current_wrist_pose_) == RTT::NoData) {
    return false;
  }

  KDL::Frame current_frame;

  tf::poseMsgToKDL(current_wrist_pose_, current_frame);

  if (!gravity_transformation)  // nie powolano jeszcze obiektu
  {

    KDL::Frame force_sensor_frame = KDL::Frame(KDL::Rotation::RotZ(M_PI),
                                               KDL::Vector(0.0, 0.0, 0.09));
    double weight = 10.8;

    KDL::Vector pointofgravity(0.004, 0.0, 0.156);

    gravity_transformation = new ForceTrans(current_frame, force_sensor_frame,
                                            weight, pointofgravity, true);

  } else {
    gravity_transformation->synchro(current_frame);
  }

  return true;
}

void ForceTransformation::updateHook() {

  port_current_wrist_pose_.read(current_wrist_pose_);
  KDL::Frame current_frame;
  tf::poseMsgToKDL(current_wrist_pose_, current_frame);

  // odczyt sily
  geometry_msgs::Wrench current_wrench;
  port_current_wrench_.read(current_wrench);

  KDL::Wrench input_force;
  tf::wrenchMsgToKDL(current_wrench, input_force);

  // offset level removal
  KDL::Wrench biased_force = input_force - force_offset;

  //sily przechowujemy w zerowej orientacji bazowej w ukladzie nadgarstka
  KDL::Wrench computed_force = gravity_transformation->getForce(biased_force,
                                                                current_frame);
  geometry_msgs::Wrench output_wrench;
  tf::wrenchKDLToMsg(computed_force, output_wrench);

  port_output_wrench_.write(output_wrench);

}

ORO_CREATE_COMPONENT(ForceTransformation)

