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

 protected:
  RTT::OutputPort<geometry_msgs::Wrench> wrench_port_;
  RTT::Property<KDL::Wrench> offset_prop_;

protected:
  void WrenchKDLToMsg(const KDL::Wrench &in, geometry_msgs::Wrench &out);
  comedi_t *device_;

};

#endif // FORCESENSOR_H
