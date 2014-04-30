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

  /* -----------------------------------------------------------------------
   Dlugosci czlonow robota [m].
   ------------------------------------------------------------------------- */
  a2 = 0.455;
  a3 = 0.67;
  d5 = 0.19;
  d6 = 0.095;
  d7 = 0.20;

  joint_position_.resize(6);
  local_current_joints_tmp_.resize(6);
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

  // poprawka w celu uwzglednienia konwencji DH
  local_current_joints_tmp_ = local_current_joints;

  local_current_joints_tmp_[2] += local_current_joints_tmp_[1] + M_PI_2;
  local_current_joints_tmp_[3] += local_current_joints_tmp_[2];

  // Parametry pomocnicze - przeliczenie zmiennych.
  const double s1 = sin(local_current_joints_tmp_[0]);
  const double c1 = cos(local_current_joints_tmp_[0]);
  const double s2 = sin(local_current_joints_tmp_[1]);
  const double c2 = cos(local_current_joints_tmp_[1]);
  const double s3 = sin(local_current_joints_tmp_[2]);
  const double c3 = cos(local_current_joints_tmp_[2]);
  const double s4 = sin(local_current_joints_tmp_[3]);
  const double c4 = cos(local_current_joints_tmp_[3]);
  const double s5 = sin(local_current_joints_tmp_[4]);
  const double c5 = cos(local_current_joints_tmp_[4]);
  const double s6 = sin(local_current_joints_tmp_[5]);
  const double c6 = cos(local_current_joints_tmp_[5]);

  // Proste zadanie kinematyki.
  (*local_current_end_effector_frame)(0, 0) = (c1 * s4 * c5 + s1 * s5) * c6
      + c1 * c4 * s6;  //NX
  (*local_current_end_effector_frame)(0, 1) = -(c1 * s4 * c5 + s1 * s5) * s6
      + c1 * c4 * c6;  //OX
  (*local_current_end_effector_frame)(0, 2) = c1 * s4 * s5 - s1 * c5;  //AX
  (*local_current_end_effector_frame)(0, 3) = c1 * (a2 * c2 + a3 * c3 + d5 * c4);  //PX
  (*local_current_end_effector_frame)(1, 0) = (s1 * s4 * c5 - c1 * s5) * c6
      + s1 * c4 * s6;  //NY2
  (*local_current_end_effector_frame)(1, 1) = -(s1 * s4 * c5 - c1 * s5) * s6
      + s1 * c4 * c6;  //OY
  (*local_current_end_effector_frame)(1, 2) = s1 * s4 * s5 + c1 * c5;  //AY
  (*local_current_end_effector_frame)(1, 3) = s1 * (a2 * c2 + a3 * c3 + d5 * c4);  //PY
  (*local_current_end_effector_frame)(2, 0) = c4 * c5 * c6 - s4 * s6;  //NZ
  (*local_current_end_effector_frame)(2, 1) = -c4 * c5 * s6 - s4 * c6;  //OZ
  (*local_current_end_effector_frame)(2, 2) = c4 * s5;  //AZ
  (*local_current_end_effector_frame)(2, 3) = -a2 * s2 - a3 * s3 - d5 * s4;  //PZ

}

ORO_CREATE_COMPONENT(Irp6pForwardKinematic)

