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

  this->addPort(wrench_port_);
  this->addProperty(offset_prop_);

}

bool ForceSensor::startHook() {
  readData();

  bias_ = voltage_ADC_;

  return true;
}


void ForceSensor::updateHook() {
  geometry_msgs::Wrench wrenchMsg;

  readData();
  voltage2FT();

//wrench_ -= offset_prop_.value();

  WrenchKDLToMsg(wrench_, wrenchMsg);

// sprawdzenie ograniczen na sile
  bool overforce = false;
  for (int i = 0; i < 6; i++) {
    if ((fabs(wrench_[i]) > FORCE_CONSTRAINTS[i])
        || (!(std::isfinite(wrench_[i])))) {
      overforce = true;
    }
  }

  if (!overforce) {
    wrench_port_.write(wrenchMsg);
  }
}

void ForceSensor::voltage2FT() {
  SetToZero(wrench_);

  Vector6d result_voltage = voltage_ADC_ - bias_;

  Vector6d force = conversion_matrix * result_voltage;
  force = force.array() * conversion_scale.array();

  for (int i = 0; i < 6; i++) {
    wrench_[i] = force(i);
  }


}

