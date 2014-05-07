#ifndef ForceControlLaw_H_
#define ForceControlLaw_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Wrench.h"

#include <Eigen/Dense>

class ForceControlLaw : public RTT::TaskContext {
 public:
  ForceControlLaw(const std::string& name);
  virtual ~ForceControlLaw();

  bool configureHook();
  void updateHook();

 private:


  RTT::InputPort<geometry_msgs::Pose> port_current_pose_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_pose_;

  RTT::InputPort<geometry_msgs::Wrench> port_current_wrench_;

};

#endif /* ForceControlLaw */
