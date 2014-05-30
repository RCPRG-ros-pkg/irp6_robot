#include <rtt/Component.hpp>

#include "ForceTransformation.h"
#include "eigen_conversions/eigen_msg.h"

ForceTransformation::ForceTransformation(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      first_run_(true) {

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

  // read current force
  geometry_msgs::Wrench current_wrench;
  if (port_current_sensor_wrench_.read(current_wrench) == RTT::NoData) {
    return false;
  }

  tf::wrenchMsgToKDL(current_wrench, force_offset_);

  // read current wrist pose
  geometry_msgs::Pose current_wrist_pose;
  if (port_current_wrist_pose_.read(current_wrist_pose) == RTT::NoData) {
    return false;
  }

  KDL::Frame current_frame;

  tf::poseMsgToKDL(current_wrist_pose, current_frame);

  if (first_run_) {

    sensor_frame_ = KDL::Frame(KDL::Rotation::RotZ(M_PI),
                               KDL::Vector(0.0, 0.0, 0.09));
    tool_weight_ = 10.8;

    gravity_arm_in_wrist_ = KDL::Vector(0.004, 0.0, 0.156);

    // ustalenie skretnosci wektora z odczytami z czujnika
    is_right_turn_frame_ = true;

    defineTool(current_frame, tool_weight_, gravity_arm_in_wrist_);
    first_run_ = false;
  } else {
    synchro(current_frame);
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
  KDL::Wrench computed_force = getForce(biased_force, current_wrist_pose_kdl);
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

void ForceTransformation::defineTool(const KDL::Frame & init_frame,
                                     const double weight,
                                     const KDL::Vector & point_of_gravity) {

  tool_weight_ = weight;
  gravity_arm_in_wrist_ = point_of_gravity;

  gravity_force_torque_in_base_ = KDL::Wrench(
      KDL::Vector(0.0, 0.0, -tool_weight_), KDL::Vector(0.0, 0.0, 0.0));

// orientacja koncowki manipulatora bez narzedzia
  KDL::Frame current_orientation(init_frame.M, KDL::Vector(0.0, 0.0, 0.0));

// sila reakcji w ukladzie czujnika z orientacja bazy
  KDL::Wrench gravity_force_torque_in_sensor = current_orientation.Inverse()
      * gravity_force_torque_in_base_;

// macierz narzedzia wzgledem nadgarstka
  tool_mass_center_translation_ = KDL::Frame(KDL::Rotation(), point_of_gravity);

// sila reakcji w ukladzie nadgarstka z orientacja bazy
  reaction_force_torque_in_wrist_ = -(tool_mass_center_translation_
      * gravity_force_torque_in_sensor);

}

// zwraca sily i momenty sil w w ukladzie z orientacja koncowki manipulatory bez narzedzia
KDL::Wrench ForceTransformation::getForce(const KDL::Wrench _inputForceTorque,
                                          const KDL::Frame curr_frame) {

  KDL::Wrench inputForceTorque = _inputForceTorque;

  if (!is_right_turn_frame_) {

    inputForceTorque[2] = -inputForceTorque[2];
    inputForceTorque[5] = -inputForceTorque[5];
  }

  // sprowadzenie wejsciowych, zmierzonych sil i momentow sil z ukladu czujnika do ukladu nadgarstka
  KDL::Wrench input_force_torque = sensor_frame_ * inputForceTorque;

  // sprowadzenie odczytow sil do ukladu czujnika przy zalozeniu ze uklad chwytaka ma te sama orientacje
  // co uklad narzedzia
  KDL::Wrench gravity_force_torque_in_sensor = (curr_frame.Inverse()).M
      * gravity_force_torque_in_base_;

  // finalne przeksztalcenie (3.30 z doktoratu TW)
  KDL::Wrench output_force_torque = input_force_torque
      - tool_mass_center_translation_ * gravity_force_torque_in_sensor
      - reaction_force_torque_in_wrist_;

  // sprowadzenie sily w ukladzie nadgarstka do orientacji ukladu bazowego
  output_force_torque = curr_frame.M * (-output_force_torque);

  return output_force_torque;

}

void ForceTransformation::synchro(const KDL::Frame & init_frame) {
  defineTool(init_frame, tool_weight_, gravity_arm_in_wrist_);
}

ORO_CREATE_COMPONENT(ForceTransformation)

