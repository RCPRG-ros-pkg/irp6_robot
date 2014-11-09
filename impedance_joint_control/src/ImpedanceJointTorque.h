// Copyright WUT 2014
#ifndef ImpedanceJointTorque_H_
#define ImpedanceJointTorque_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include <Eigen/Dense>
#include <kdl/frames.hpp>

class ImpedanceJointTorque : public RTT::TaskContext {
 public:
  explicit ImpedanceJointTorque(const std::string& name);
  virtual ~ImpedanceJointTorque();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

};

#endif  // ImpedanceJointTorque_H_
