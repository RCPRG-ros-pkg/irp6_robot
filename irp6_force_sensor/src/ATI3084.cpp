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

#include "ATI3084.h"

#define MUX0 0
#define MUX1 1
#define MUX2 2

#define DOSD 3

ATI3084::ATI3084(const std::string &name)
    : ForceSensor(name),
      device_prop_("device", "DAQ device to use", "/dev/comedi0"),
      maxdata_(0),
      rangetype_(NULL) {

  this->addProperty(device_prop_);

  // Initialize conversion matrix
  conversion_matrix << -0.000022, 0.001325, -0.035134, 0.640126, 0.051951, -0.641909, 0.017570, -0.743414, -0.016234, 0.372558, -0.032329, 0.366082, -1.184654, -0.012028, -1.165485, -0.014266, -1.174821, 0.002540, 0.007847, -0.144965, 0.552931, 0.079813, -0.571950, 0.071877, -0.661215, -0.007048, 0.337836, -0.125610, 0.315335, 0.132327, -0.010556, 0.346443, -0.009666, 0.344562, -0.031572, 0.339944;  // NOLINT
  conversion_scale << -20.4, -20.4, -20.4, -1.23, -1.23, -1.23;
}

bool ATI3084::configureParticularSensorHook() {
  device_ = comedi_open(device_prop_.value().c_str());
  if (!device_) {
    RTT::log(RTT::Error) << "Unable to open device [" << device_prop_.value()
                         << "]" << RTT::endlog();
    return false;
  }
  // comedi_dio_config(device_, DOSD, 0, COMEDI_OUTPUT);
  // comedi_dio_config(device_, DOSD, 1, COMEDI_OUTPUT);
  // comedi_dio_config(device_, DOSD, 2, COMEDI_OUTPUT);

  maxdata_ = comedi_get_maxdata(device_, 0, 0);
  rangetype_ = comedi_get_range(device_, 0, 0, 0);

  return true;
}

void ATI3084::readData() {
  comedi_dio_write(device_, DOSD, MUX0, 0);
  comedi_dio_write(device_, DOSD, MUX1, 0);
  comedi_dio_write(device_, DOSD, MUX2, 0);

  usleep(USLEEP_MUX);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[0]);
  ////// G1

  comedi_dio_write(device_, DOSD, MUX0, 1);
  usleep(USLEEP_MUX);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[1]);
  ////// G2

  comedi_dio_write(device_, DOSD, MUX1, 1);
  usleep(USLEEP_MUX);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[2]);
  ////// G3

  comedi_dio_write(device_, DOSD, MUX0, 0);
  usleep(USLEEP_MUX);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[3]);
  ////// G4

  comedi_dio_write(device_, DOSD, MUX2, 1);
  usleep(USLEEP_MUX);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[4]);
  ////// G5

  comedi_dio_write(device_, DOSD, MUX0, 1);
  usleep(USLEEP_MUX);
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[5]);

  for (int i = 0; i < 6; i++) {
    voltage_ADC_(i) = comedi_to_phys(raw_ADC_[i], rangetype_, maxdata_);
  }

  // for(int i = 0; i < 6; i++)
  //  voltage_ADC_[i] /= 5;
}

ORO_CREATE_COMPONENT(ATI3084)
