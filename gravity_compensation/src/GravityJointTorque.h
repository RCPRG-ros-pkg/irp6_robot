// Copyright WUT 2014
#ifndef GRAVITYJOINTTORQUE_H_
#define GRAVITYJOINTTORQUE_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include <Eigen/Dense>
#include <kdl/frames.hpp>

class GravityJointTorque : public RTT::TaskContext {
 public:
  explicit GravityJointTorque(const std::string& name);
  virtual ~GravityJointTorque();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();


};

#endif  // GRAVITYJOINTTORQUE_H_
