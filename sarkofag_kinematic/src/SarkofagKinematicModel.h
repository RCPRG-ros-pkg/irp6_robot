// Copyright WUT 2014
#ifndef SarkofagKinematicModel_H_
#define SarkofagKinematicModel_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>

#include <Eigen/Dense>
#include "SarkofagKinematic.h"

class SarkofagKinematicModel : public RTT::TaskContext {
 public:
  SarkofagKinematicModel(const std::string& name);
  virtual ~SarkofagKinematicModel();

  bool configureHook();
  void updateHook();
 private:
  void direct_kinematics_transform(
      const Eigen::VectorXd& local_current_joints,
      Eigen::Affine3d* local_current_end_effector_frame);

  RTT::InputPort<Eigen::VectorXd> port_joint_position_;

  RTT::InputPort<geometry_msgs::Pose> port_tool_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_wrist_pose_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_end_effector_pose_;

  Eigen::VectorXd joint_position_;
  Eigen::VectorXd local_current_joints_tmp_;

  geometry_msgs::Pose tool_msgs_;

  //! D-H kinematic parameters - length of 2nd segment.
  double a1;

};

#endif /* SarkofagKinematicModel_H_ */
