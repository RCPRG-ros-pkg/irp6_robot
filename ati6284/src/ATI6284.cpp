#include <ocl/Component.hpp>

#include "ATI6284.h"

ATI6284::ATI6284(const std::string &name)
    : ForceSensor(name),
      device_prop_("device", "DAQ device to use", "/dev/comedi0") {

  this->addProperty(device_prop_);

  // Initialize conversion matrix
  conversion_matrix << -0.40709, -0.27318, 0.34868, -33.58156, -0.32609, 33.54162, 0.35472, 38.22730, -0.41173, -19.49156, 0.49550, -19.15271, 18.72635, -0.59676, 19.27843, -0.56931, 18.69352, -0.67633, -0.40836, -0.95908, -33.37957, 1.38537, 32.52522, -0.51156, 37.13715, -1.02875, -20.00474, -0.27959, -19.34135, 1.42577, -0.15775, -18.16831, -0.00133, -18.78961, 0.31895, -18.38586;

  conversion_scale << 0.219722406, 0.219722406, 0.707994418, 0.011780738, 0.011780738, 0.012353731;

}

bool ATI6284::configureParticularSensorHook() {

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
    voltage_ADC_(i) = comedi_to_physical(raw_ADC_[i], &calib_ADC_);
  }

//for(int i = 0; i < 6; i++)
//  voltage_ADC_[i] /= 5;
}

ORO_CREATE_COMPONENT(ATI6284)
