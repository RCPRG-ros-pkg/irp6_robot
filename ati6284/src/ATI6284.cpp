#include <ocl/Component.hpp>

#include "ATI6284.h"

ATI6284::ATI6284(const std::string &name)
    : ForceSensor(name),
      device_prop_("device", "DAQ device to use", "/dev/comedi1") {
  this->addPort(wrench_port_);
  this->addProperty(device_prop_);
  this->addProperty(offset_prop_);
}

bool ATI6284::configureHook() {
  return initSensor();
}

bool ATI6284::startHook() {
  readData();

  for (int i = 0; i < 6; i++)
    bias_[i] = voltage_ADC_[i];

  return true;
}

void ATI6284::updateHook() {
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

void ATI6284::stopHook() {

}

bool ATI6284::initSensor() {
  device_ = comedi_open(device_prop_.value().c_str());
  if (!device_) {
    RTT::log(RTT::Error) << "Unable to open device [" << device_prop_.value()
                         << "]" << RTT::endlog();
    return false;
  }
  if (comedi_apply_calibration(device_, 0, 0, 0, 0, NULL) != 0) {
    RTT::log(RTT::Error) << "Unable to set calibration" << RTT::endlog();
    //return false;
  }

  comedi_get_hardcal_converter(device_, 0, 0, 0, COMEDI_TO_PHYSICAL,
                               &calib_ADC_);
  return true;
}

void ATI6284::readData() {
  for (int i = 0; i < 6; i++) {
    comedi_data_read(device_, 0, i, 0, AREF_DIFF, &raw_ADC_[i]);
  }

  for (int i = 0; i < 6; i++) {
    voltage_ADC_[i] = comedi_to_physical(raw_ADC_[i], &calib_ADC_);
  }

  //for(int i = 0; i < 6; i++)
  //  voltage_ADC_[i] /= 5;
}

void ATI6284::voltage2FT() {
  SetToZero(wrench_);

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      wrench_[i] += (voltage_ADC_[j] - bias_[j]) * conversion_matrix[i][j];
    }

    wrench_[i] /= conversion_scale[i];
  }
}

ORO_CREATE_COMPONENT(ATI6284)
