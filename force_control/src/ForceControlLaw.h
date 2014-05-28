#ifndef ForceControlLaw_H_
#define ForceControlLaw_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Wrench.h"

#include <Eigen/Dense>
#include "kdl/frames.hpp"

class ForceControlLaw : public RTT::TaskContext {
 public:
  ForceControlLaw(const std::string& name);
  virtual ~ForceControlLaw();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

 private:

  RTT::InputPort<geometry_msgs::Pose> port_current_wrist_pose_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_end_effector_pose_;

  RTT::InputPort<geometry_msgs::Wrench> port_current_end_effector_wrench_;
  RTT::InputPort<geometry_msgs::Pose> port_tool_;

  KDL::Frame cl_ef_pose_kdl_;

};

#endif /* ForceControlLaw */
