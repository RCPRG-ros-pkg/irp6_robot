// Copyright WUT 2014
#ifndef ImpedanceMotorCurrent_H_
#define ImpedanceMotorCurrent_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Wrench.h"
#include "kdl_conversions/kdl_msg.h"
#include <Eigen/Dense>

class ImpedanceMotorCurrent : public RTT::TaskContext {
 public:
  explicit ImpedanceMotorCurrent(const std::string& name);
  virtual ~ImpedanceMotorCurrent();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

};

#endif  // ImpedanceMotorCurrent_H_
