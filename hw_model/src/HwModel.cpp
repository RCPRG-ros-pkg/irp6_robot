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

#include "HwModel.h"
#include "string_colors.h"

HwModel::HwModel(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      number_of_servos_(0),
      m_factor_(0) {
  this->ports()->addPort("MotorPosition", port_motor_position_);
  this->ports()->addPort("DesiredInput", port_desired_input_);

  this->addProperty("iteration_per_step", iteration_per_step_);
  this->addProperty("step_per_second", step_per_second_);
  this->addProperty("torque_constant", torque_constant_);
  this->addProperty("inertia", inertia_);
  this->addProperty("viscous_friction", viscous_friction_);
  this->addProperty("current_input", current_input_);
}

HwModel::~HwModel() {
}

bool HwModel::configureHook() {
  number_of_servos_ = torque_constant_.size();
  if ((number_of_servos_ != inertia_.size())
      || (number_of_servos_ != viscous_friction_.size())
      || (number_of_servos_ != current_input_.size())) {
    std::cout
        << std::endl
        << RED
        << "[error] hardware model "
        << getName()
        << "configuration failed: wrong properties vector length in launch file."
        << RESET << std::endl;
    return false;
  }

  motor_position_.resize(number_of_servos_);
  motor_velocity_.resize(number_of_servos_);
  motor_acceleration_.resize(number_of_servos_);
  desired_input_.resize(number_of_servos_);
  desired_torque_.resize(number_of_servos_);
  effective_torque_.resize(number_of_servos_);

  port_motor_position_.setDataSample(motor_position_);

  for (int i = 0; i < number_of_servos_; i++) {
    motor_position_(i) = 0.0;
    motor_velocity_(i) = 0.0;
    motor_acceleration_(i) = 0.0;
    desired_input_(i) = 0.0;
    desired_torque_(i) = 0.0;
    effective_torque_(i) = 0.0;
  }

  m_factor_ = step_per_second_ * iteration_per_step_;

  return true;
}

void HwModel::updateHook() {
  if (RTT::NewData == port_desired_input_.read(desired_input_)) {
//    std::cout << "HwModel updateHook" << desired_input_(1) << std::endl;
// pytanie czy to nie przychodzi w inkrementach
    for (int servo = 0; servo < number_of_servos_; servo++) {
      // PWM input do implementacji
      /*
       if (!current_input_[servo]) {  // pwm input
       motor_position_(servo) = desired_input_(servo);
       } else {  // current input
       */

      // prad jest w miliamperach
      desired_torque_(servo) = desired_input_(servo) * torque_constant_[servo];

      for (int iteration = 0; iteration < iteration_per_step_; iteration++) {
        effective_torque_(servo) = desired_torque_(servo)
            - motor_velocity_(servo) * viscous_friction_[servo];
        motor_acceleration_(servo) = effective_torque_(servo) / inertia_[servo];
        motor_velocity_(servo) += motor_acceleration_(servo) / m_factor_;
        motor_position_(servo) += motor_velocity_(servo) / m_factor_;
      }
      //}
    }
    port_motor_position_.write(motor_position_);
  }
}

ORO_CREATE_COMPONENT(HwModel)
