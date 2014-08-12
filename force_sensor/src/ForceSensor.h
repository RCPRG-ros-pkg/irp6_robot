#ifndef FORCESENSOR_H
#define FORCESENSOR_H

#include <string>
#include <comedilib.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"

class ForceSensor : public RTT::TaskContext {

 public:
  ForceSensor(const std::string &name);

  void WrenchKDLToMsg(const KDL::Wrench &in, geometry_msgs::Wrench &out);

};

#endif // FORCESENSOR_H
