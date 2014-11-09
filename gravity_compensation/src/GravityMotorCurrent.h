// Copyright WUT 2014
#ifndef GRAVITYMOTORCURRENT_H_
#define GRAVITYMOTORCURRENT_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Wrench.h"
#include "kdl_conversions/kdl_msg.h"
#include <Eigen/Dense>

class GravityMotorCurrent : public RTT::TaskContext {
 public:
  explicit GravityMotorCurrent(const std::string& name);
  virtual ~GravityMotorCurrent();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

};

#endif  // GRAVITYMOTORCURRENT_H_
