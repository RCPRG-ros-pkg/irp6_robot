#include <ocl/Component.hpp>

#include "ForceSensor.h"

#define MUX0 0
#define MUX1 1
#define MUX2 2

#define DOSD 3

void ForceSensor::WrenchKDLToMsg(const KDL::Wrench &in,
                                 geometry_msgs::Wrench &out) {
  out.force.x = in[0];
  out.force.y = in[1];
  out.force.z = in[2];

  out.torque.x = in[3];
  out.torque.y = in[4];
  out.torque.z = in[5];
}

ForceSensor::ForceSensor(const std::string &name)
    : RTT::TaskContext(name, PreOperational),
      wrench_port_("Wrench"),
      offset_prop_("offset", "sensor zero offset", KDL::Wrench::Zero()),
      device_(NULL) {

}

