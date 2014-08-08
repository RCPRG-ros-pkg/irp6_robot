// Copyright WUT 2014
#ifndef IRP6HAPTIC_H_
#define IRP6HAPTIC_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <force_control_msgs/ForceControl.h>

class Irp6Haptic : public RTT::TaskContext {
 public:
  explicit Irp6Haptic(const std::string& name);
  virtual ~Irp6Haptic();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

 private:
  RTT::InputPort<geometry_msgs::Pose> port_current_end_effector_pose_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_end_effector_pose_;

  RTT::InputPort<geometry_msgs::Wrench> port_current_end_effector_wrench_;

  RTT::OutputPort<force_control_msgs::ForceControl> port_output_fcl_param_;

  KDL::Frame cl_ef_pose_kdl_;
  KDL::Twist p_vel_;
  double step_duration_;
};

#endif  // IRP6HAPTIC_H_
