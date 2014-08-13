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
      raw_wrench_output_port_("OutputRawWrench"),
      fast_filtered_wrench_output_port_("OutputFastFilteredWrench"),
      slow_filtered_wrench_output_port_("OutputSlowFilteredWrench"),
      offset_prop_("offset", "sensor zero offset", KDL::Wrench::Zero()),
      device_(NULL) {

  this->addPort(raw_wrench_output_port_);
  this->addPort(fast_filtered_wrench_output_port_);
  this->addPort(slow_filtered_wrench_output_port_);

  this->addProperty(offset_prop_);
  this->addProperty("force_limits", force_limits_);
}

bool ForceSensor::startHook() {
  readData();

  bias_ = voltage_ADC_;

  return true;
}

bool ForceSensor::configureHook() {

  if (force_limits_.size() != 6) {
    RTT::Logger::log(RTT::Logger::Error) << "Force limits not loaded"
                                         << RTT::endlog();
    return false;
  }

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
    if ((fabs(wrench_[i]) > force_limits_[i])
        || (!(std::isfinite(wrench_[i])))) {
      overforce = true;
    }
  }

  if (!overforce) {
    raw_wrench_output_port_.write(wrenchMsg);
    fast_filtered_wrench_output_port_.write(wrenchMsg);
    slow_filtered_wrench_output_port_.write(wrenchMsg);
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

