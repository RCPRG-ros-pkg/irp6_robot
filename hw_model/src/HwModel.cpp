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

#include "HwModel.h"

#include <rtt/Component.hpp>
#include <string>

HwModel::HwModel(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      number_of_servos_(0),
      m_factor_(0) {
  this->ports()->addPort("MotorPosition", port_motor_position_);
  this->ports()->addPort("DesiredInput", port_desired_input_);

  this->addProperty("iteration_per_step_", iteration_per_step_);
  this->addProperty("step_per_second", step_per_second_);
  this->addProperty("torque_constant", torque_constant_);
  this->addProperty("inertia", inertia_);
  this->addProperty("viscous_friction", viscous_friction_);
  this->addProperty("current_or_position_input", current_or_position_input_);

}

HwModel::~HwModel() {
}

bool HwModel::configureHook() {

  // number_of_servos_ trzeba czytac z konfiguracji pierwszego z wektorow (w ktoryms z komponentow tak to jest robione)
  // trzeba sprawdzic czy wszystkie wektory sa wlaciwej wielkosci

  motor_position_.resize(number_of_servos_);
  motor_velocity_.resize(number_of_servos_);
  motor_acceleration_.resize(number_of_servos_);
  desired_input_.resize(number_of_servos_);
  desired_torque_.resize(number_of_servos_);
  effective_torque_.resize(number_of_servos_);

// motor_position_ ustaw na zero
// motor_velocity_ ustaw na zero
  m_factor_ = step_per_second_ * iteration_per_step_;

  return true;
}

void HwModel::updateHook() {

  /*

   desired_input_ przypisz z port_desired_input_


   for number_of_servos_

   sprawdz current_or_position_input_
   if position input_
   motor_position_ = desired_input_

   else if current_input_
   desired_torque_ = desired_input_ * torque_constant_
   for iteration_per_step_
   effective_torque_ = desired_torque_ - motor_velocity_ * viscous_friction_
   motor_acceleration_ = effective_torque / inertia_
   motor_velocity_ += motor_acceleration_ / m_factor_
   motor_position_ += motor_velocity_ / m_factor_

   endfor iteration_per_step_

   endif current_input
   endfor number_of_servos_

   port_motor_position_ przypisz motor_position_
   */
}

ORO_CREATE_COMPONENT(HwModel)
