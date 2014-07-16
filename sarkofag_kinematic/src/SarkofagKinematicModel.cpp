// Copyright WUT 2014
#include <rtt/Component.hpp>

#include "SarkofagKinematicModel.h"
#include "eigen_conversions/eigen_msg.h"

SarkofagKinematicModel::SarkofagKinematicModel(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      a1(0.0) {

  this->ports()->addPort("JointPosition", port_joint_position_);
  this->ports()->addPort("Tool", port_tool_);

  this->ports()->addPort("WristOutputPose", port_output_wrist_pose_);
  this->ports()->addPort("EndEffectorOutputPose",
                         port_output_end_effector_pose_);

}

SarkofagKinematicModel::~SarkofagKinematicModel() {

}

bool SarkofagKinematicModel::configureHook() {

  /* -----------------------------------------------------------------------
   Dlugosci czlonow robota [m].
   ------------------------------------------------------------------------- */
  a1 = a1_const;

  joint_position_.resize(1);
  local_current_joints_tmp_.resize(1);

  if (port_tool_.read(tool_msgs_) == RTT::NewData) {

    return false;

  }

  return true;
}

void SarkofagKinematicModel::updateHook() {

  if (port_joint_position_.read(joint_position_) == RTT::NewData) {

    port_tool_.read(tool_msgs_);

    Eigen::Affine3d tool;
    Eigen::Affine3d end_effector_pose;
    Eigen::Affine3d wrist_pose;
    geometry_msgs::Pose wrist_pose_msgs;
    geometry_msgs::Pose end_effector_pose_msgs;

    tf::poseMsgToEigen(tool_msgs_, tool);

    direct_kinematics_transform(joint_position_, &wrist_pose);

    end_effector_pose = wrist_pose * tool;

    tf::poseEigenToMsg(wrist_pose, wrist_pose_msgs);
    tf::poseEigenToMsg(end_effector_pose, end_effector_pose_msgs);

    port_output_wrist_pose_.write(wrist_pose_msgs);
    port_output_end_effector_pose_.write(end_effector_pose_msgs);
  }
}

void SarkofagKinematicModel::direct_kinematics_transform(
    const Eigen::VectorXd& local_current_joints,
    Eigen::Affine3d* local_current_end_effector_frame) {

  // poprawka w celu uwzglednienia konwencji DH
  local_current_joints_tmp_ = local_current_joints;

  // Parametry pomocnicze - przeliczenie zmiennych.
  const double s1 = sin((double)local_current_joints_tmp_[0]);
  const double c1 = cos((double)local_current_joints_tmp_[0]);

  // Proste zadanie kinematyki.
  (*local_current_end_effector_frame)(0, 0) = (c1  + s1) + c1;  //NX
  (*local_current_end_effector_frame)(0, 1) = -(c1 + s1) + c1;  //OX
  (*local_current_end_effector_frame)(0, 2) = c1  - s1;  //AX
  (*local_current_end_effector_frame)(0, 3) = c1;  //PX
  (*local_current_end_effector_frame)(1, 0) = (s1 - c1) + s1;  //NY2
  (*local_current_end_effector_frame)(1, 1) = -(s1 - c1) + s1;  //OY
  (*local_current_end_effector_frame)(1, 2) = s1 + c1;  //AY
  (*local_current_end_effector_frame)(1, 3) = s1;  //PY
  (*local_current_end_effector_frame)(2, 0) = 0;  //NZ
  (*local_current_end_effector_frame)(2, 1) = 0;  //OZ
  (*local_current_end_effector_frame)(2, 2) = 0;  //AZ
  (*local_current_end_effector_frame)(2, 3) = z_offset_const;  //PZ

}

ORO_CREATE_COMPONENT(SarkofagKinematicModel)

