// Copyright WUT 2014
#ifndef Irp6pInverseKinematic_H_
#define Irp6pInverseKinematic_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>

#include <Eigen/Dense>
#include "Irp6pKinematic.h"

class Irp6pInverseKinematic : public RTT::TaskContext {
 public:
  Irp6pInverseKinematic(const std::string& name);
  virtual ~Irp6pInverseKinematic();

  bool configureHook();
  void updateHook();

 private:
  Eigen::VectorXd local_desired_joints_;
  Eigen::VectorXd local_current_joints_;
  Eigen::VectorXd local_current_joints_tmp_;

  void inverse_kinematics_single_iteration(
      const Eigen::VectorXd& local_current_joints,
      const Eigen::Affine3d& local_desired_end_effector_frame,
      Eigen::VectorXd* local_desired_joints);

  RTT::InputPort<geometry_msgs::Pose> port_tool_;
  RTT::InputPort<Eigen::VectorXd> port_current_joint_position_;
  RTT::OutputPort<Eigen::VectorXd> port_output_joint_position_;

  RTT::InputPort<geometry_msgs::Pose> port_input_wrist_pose_;
  RTT::InputPort<geometry_msgs::Pose> port_input_end_effector_pose_;

  geometry_msgs::Pose wrist_pose_;
  geometry_msgs::Pose end_effector_pose_;

  geometry_msgs::Pose tool_msgs_;

  //! D-H kinematic parameters - length of 2nd segment.
  double a2;

  //! D-H kinematic parameters - length of 3rd segment.
  double a3;

  //! D-H kinematic parameters - length of 4th segment.
  double d5;

  //! D-H kinematic parameters - length of 5th segment.
  double d6;

  //! D-H kinematic parameters - length of 6th segment.
  double d7;

};

#endif /* Irp6pInverseKinematic */
