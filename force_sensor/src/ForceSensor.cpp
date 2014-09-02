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
      device_(NULL),
      slow_buffer_size_(100),
      fast_buffer_size_(2),
      slow_buffer_index_(0),
      fast_buffer_index_(0),
      test_mode_(0) {

  this->addPort(raw_wrench_output_port_);
  this->addPort(fast_filtered_wrench_output_port_);
  this->addPort(slow_filtered_wrench_output_port_);
  this->addProperty("test_mode", test_mode_).doc("");

  this->addProperty(offset_prop_);
  this->addProperty("force_limits", force_limits_);
  this->addProperty("slow_buffer_size", slow_buffer_size_);
  this->addProperty("fast_buffer_size", fast_buffer_size_);
}

bool ForceSensor::startHook() {
  if (!test_mode_) {
    readData();
  } else {
    for (int i = 0; i < 6; i++) {
      voltage_ADC_(i) = 0.0;
    }
  }

  bias_ = voltage_ADC_;

  return true;
}

bool ForceSensor::configureHook() {

  if (force_limits_.size() != 6) {
    RTT::Logger::log(RTT::Logger::Error) << "Force limits not loaded"
                                         << RTT::endlog();
    return false;
  }

  slow_buffer_.resize(slow_buffer_size_);
  fast_buffer_.resize(fast_buffer_size_);

  if (!test_mode_) {
    return configureParticularSensorHook();
  } else {
    return true;
  }

}

void ForceSensor::updateHook() {
  geometry_msgs::Wrench RawWrenchMsg;
  geometry_msgs::Wrench SlowFilteredWrenchMsg;
  geometry_msgs::Wrench FastFilteredWrenchMsg;

  if (!test_mode_) {
    readData();
  } else {
    for (int i = 0; i < 6; i++) {
      voltage_ADC_(i) = 0.0;
    }
  }

  voltage2FT();

//wrench_ -= offset_prop_.value();

// sprawdzenie ograniczen na sile
  bool overforce = false;
  for (int i = 0; i < 6; i++) {
    if ((fabs(wrench_[i]) > force_limits_[i])
        || (!(std::isfinite(wrench_[i])))) {
      overforce = true;
    }
  }

  if (!overforce) {
    valid_wrench_ = wrench_;
  }

  WrenchKDLToMsg(valid_wrench_, RawWrenchMsg);
  raw_wrench_output_port_.write(RawWrenchMsg);

  // tu i ponizej dla fast moze być problem ze stabilnoscią numeryczną
  // po jej stwierdzeniu zamienic na proste liczenie podzielonej sumy wszystkich elementow

  slow_filtered_wrench_ = slow_filtered_wrench_
      + valid_wrench_ / slow_buffer_size_
      - slow_buffer_[slow_buffer_index_] / slow_buffer_size_;

  slow_buffer_[slow_buffer_index_] = valid_wrench_;
  if ((++slow_buffer_index_) == slow_buffer_size_) {
    slow_buffer_index_ = 0;
  }

  WrenchKDLToMsg(slow_filtered_wrench_, SlowFilteredWrenchMsg);
  slow_filtered_wrench_output_port_.write(SlowFilteredWrenchMsg);

  fast_filtered_wrench_ = fast_filtered_wrench_
      + valid_wrench_ / fast_buffer_size_
      - fast_buffer_[fast_buffer_index_] / fast_buffer_size_;

  fast_buffer_[fast_buffer_index_] = valid_wrench_;
  if ((++fast_buffer_index_) == fast_buffer_size_) {
    fast_buffer_index_ = 0;
  }

  WrenchKDLToMsg(fast_filtered_wrench_, FastFilteredWrenchMsg);
  fast_filtered_wrench_output_port_.write(FastFilteredWrenchMsg);

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

