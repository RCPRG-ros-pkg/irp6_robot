#ifndef Irp6pForwardKinematic_H_
#define Irp6pForwardKinematic_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>

#include <Eigen/Dense>

class Irp6pForwardKinematic : public RTT::TaskContext {
 public:
  Irp6pForwardKinematic(const std::string& name);
  virtual ~Irp6pForwardKinematic();

  bool configureHook();
  void updateHook();
 private:
  void direct_kinematics_transform(
      const Eigen::VectorXd& local_current_joints,
      Eigen::Affine3d* local_current_end_effector_frame);

  RTT::InputPort<Eigen::VectorXd> port_joint_position_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_pose_;

  Eigen::VectorXd joint_position_;
};

#endif /* Irp6pForwardKinematic */
