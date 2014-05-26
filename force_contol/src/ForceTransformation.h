#ifndef ForceTransformation_H_
#define ForceTransformation_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Wrench.h"

#include <Eigen/Dense>

class ForceTransformation : public RTT::TaskContext {
 public:
  ForceTransformation(const std::string& name);
  virtual ~ForceTransformation();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

 private:

  RTT::InputPort<geometry_msgs::Pose> port_current_pose_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_pose_;

  RTT::InputPort<geometry_msgs::Wrench> port_current_wrench_;

  geometry_msgs::Pose current_pose_;

};

#endif /* ForceTransformation_H_ */
