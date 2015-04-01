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

#ifndef FORCESENSOR_H_
#define FORCESENSOR_H_

#include <comedilib.h>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

#include <string>
#include <vector>



#include "geometry_msgs/Wrench.h"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class ForceSensor : public RTT::TaskContext {
 public:
  explicit ForceSensor(const std::string &name);
  bool startHook();
  void updateHook();
  virtual bool configureHook();

 protected:
  Matrix6d conversion_matrix;  // F/T conversion matrix
  Vector6d conversion_scale;  // F/T scaling

  KDL::Wrench wrench_;
  KDL::Wrench valid_wrench_;

  RTT::OutputPort<geometry_msgs::Wrench> raw_wrench_output_port_;
  RTT::OutputPort<geometry_msgs::Wrench> fast_filtered_wrench_output_port_;
  RTT::OutputPort<geometry_msgs::Wrench> slow_filtered_wrench_output_port_;

  void WrenchKDLToMsg(const KDL::Wrench &in, geometry_msgs::Wrench *out);
  void voltage2FT();
  virtual void readData() = 0;
  virtual bool configureParticularSensorHook() = 0;

  comedi_t *device_;

  lsampl_t raw_ADC_[6];
  Vector6d voltage_ADC_;
  Vector6d bias_;

  std::vector<KDL::Wrench> slow_buffer_;
  std::vector<KDL::Wrench> fast_buffer_;

  int slow_buffer_index_;
  int fast_buffer_index_;

  KDL::Wrench slow_filtered_wrench_;
  KDL::Wrench fast_filtered_wrench_;

  // properties
  std::vector<double> force_limits_;
  RTT::Property<KDL::Wrench> offset_prop_;
  int slow_buffer_size_;
  int fast_buffer_size_;
  bool test_mode_;
};

#endif  // FORCESENSOR_H_
