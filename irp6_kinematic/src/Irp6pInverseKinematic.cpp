#include <rtt/Component.hpp>

#include "Irp6pInverseKinematic.h"
#include "eigen_conversions/eigen_msg.h"

Irp6pInverseKinematic::Irp6pInverseKinematic(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("CurrentJointPosition", port_current_joint_position_);
  this->ports()->addPort("OutputJointPosition", port_output_joint_position_);
  this->ports()->addPort("InputPose", port_input_pose_);

}

Irp6pInverseKinematic::~Irp6pInverseKinematic() {

}

bool Irp6pInverseKinematic::configureHook() {

  /* -----------------------------------------------------------------------
   Dlugosci czlonow robota [m].
   ------------------------------------------------------------------------- */
  a2 = 0.455;
  a3 = 0.67;
  d5 = 0.19;
  d6 = 0.095;
  d7 = 0.20;

  local_desired_joints_.resize(6);
  local_current_joints_.resize(6);

  /*
   joint_position_.resize(6);
   local_current_joints_tmp_.resize(6);
   */

  return true;
}

void Irp6pInverseKinematic::updateHook() {

  if (port_input_pose_.read(pos) == RTT::NewData) {

    Eigen::Affine3d trans;
    tf::poseEigenToMsg(trans, pos);

    port_current_joint_position_.read(local_current_joints_);

    /*

     geometry_msgs::Pose pos;

     direct_kinematics_transform(joint_position_, &trans);

     tf::poseEigenToMsg(trans, pos);
     */

    inverse_kinematics_single_iteration(local_current_joints_, trans,
                                        &local_desired_joints_);

    port_output_joint_position_.write(local_desired_joints_);

  }

}

void Irp6pInverseKinematic::inverse_kinematics_single_iteration(
    const Eigen::VectorXd& local_current_joints,
    const Eigen::Affine3d& local_desired_end_effector_frame,
    Eigen::VectorXd* local_desired_joints) {

}

/*
 void Irp6pInverseKinematic::direct_kinematics_transform(
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
 */

ORO_CREATE_COMPONENT(Irp6pInverseKinematic)

