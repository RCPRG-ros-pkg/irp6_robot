/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rtt/Component.hpp>
#include <string>
#include "ATI6284.h"

ATI6284::ATI6284(const std::string &name)
    : ForceSensor(name),
      device_prop_("device", "DAQ device to use", "/dev/comedi1") {
  this->addProperty(device_prop_);

  // Initialize conversion matrix
  conversion_matrix << -0.40709, -0.27318, 0.34868, -33.58156, -0.32609, 33.54162, 0.35472, 38.22730, -0.41173, -19.49156, 0.49550, -19.15271, 18.72635, -0.59676, 19.27843, -0.56931, 18.69352, -0.67633, -0.40836, -0.95908, -33.37957, 1.38537, 32.52522, -0.51156, 37.13715, -1.02875, -20.00474, -0.27959, -19.34135, 1.42577, -0.15775, -18.16831, -0.00133, -18.78961, 0.31895, -18.38586; // NOLINT
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
    // return false;
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

// for(int i = 0; i < 6; i++)
//  voltage_ADC_[i] /= 5;
}

ORO_CREATE_COMPONENT(ATI6284)
