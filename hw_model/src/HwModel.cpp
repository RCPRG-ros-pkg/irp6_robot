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
#include "common_headers/string_colors.h"

HwModel::HwModel(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      number_of_servos_(0),
      m_factor_(0) {
  // this->ports()->addPort("MotorPosition", port_motor_position_);
  // this->ports()->addPort("DesiredInput", port_desired_input_);

  // ports addition

  this->ports()->addPort("EmergencyStopIn", port_emergency_stop_);
  this->ports()->addPort("DoSynchroIn", port_do_synchro_);
  this->ports()->addPort("IsSynchronised", port_is_synchronised_);
  this->ports()->addPort("IsHardwarePanic", port_is_hardware_panic_);

  this->addProperty("iteration_per_step", iteration_per_step_);
  this->addProperty("step_per_second", step_per_second_);
  this->addProperty("torque_constant", torque_constant_);
  this->addProperty("input_current_units_per_amper",
                    input_current_units_per_amper_);
  this->addProperty("inertia", inertia_);
  this->addProperty("viscous_friction", viscous_friction_);
  this->addProperty("enc_res", enc_res_);
  this->addProperty("port_labels", port_labels_).doc("");
}

HwModel::~HwModel() {
}

bool HwModel::configureHook() {
  number_of_servos_ = torque_constant_.size();
  if ((number_of_servos_ != torque_constant_.size())
      || (number_of_servos_ != input_current_units_per_amper_.size())
      || (number_of_servos_ != inertia_.size())
      || (number_of_servos_ != viscous_friction_.size())
      || (number_of_servos_ != enc_res_.size())) {
    std::cout << std::endl << RED << "[error] hardware model " << getName()
        << " configuration failed: wrong properties vector length in launch file."
        << RESET << std::endl;
    return false;
  }

  motor_position_.resize(number_of_servos_);
  motor_velocity_.resize(number_of_servos_);
  motor_acceleration_.resize(number_of_servos_);
  desired_input_.resize(number_of_servos_);
  desired_torque_.resize(number_of_servos_);
  effective_torque_.resize(number_of_servos_);
  inc_motor_position_.resize(number_of_servos_);

  // port_motor_position_.setDataSample(motor_position_);

  port_motor_position_list_.resize(number_of_servos_);
  port_desired_input_list_.resize(number_of_servos_);

  desired_position_out_list_.resize(number_of_servos_);
  port_motor_position_command_list_.resize(number_of_servos_);
  port_motor_current_list_.resize(number_of_servos_);
  port_regulator_reset_list_.resize(number_of_servos_);

  for (int i = 0; i < number_of_servos_; i++) {
    char port_motor_position_name[32];
    snprintf(port_motor_position_name, sizeof(port_motor_position_name),
             "MotorPosition_%s", port_labels_[i].c_str());
    port_motor_position_list_[i] = new typeof(*port_motor_position_list_[i]);
    this->ports()->addPort(port_motor_position_name,
                           *port_motor_position_list_[i]);

    char port_desired_input_name[32];
    snprintf(port_desired_input_name, sizeof(port_desired_input_name),
             "computedReg_in_%s", port_labels_[i].c_str());
    port_desired_input_list_[i] = new typeof(*port_desired_input_list_[i]);
    this->ports()->addPort(port_desired_input_name,
                           *port_desired_input_list_[i]);

    char DesiredPosition_out_port_name[32];
    snprintf(DesiredPosition_out_port_name,
             sizeof(DesiredPosition_out_port_name), "DesiredPosition_out_%s",
             port_labels_[i].c_str());
    desired_position_out_list_[i] = new typeof(*desired_position_out_list_[i]);
    this->ports()->addPort(DesiredPosition_out_port_name,
                           *desired_position_out_list_[i]);

    char MotorPositionCommand_port_name[32];
    snprintf(MotorPositionCommand_port_name,
             sizeof(MotorPositionCommand_port_name), "MotorPositionCommand_%s",
             port_labels_[i].c_str());
    port_motor_position_command_list_[i] =
        new typeof(*port_motor_position_command_list_[i]);
    this->ports()->addPort(MotorPositionCommand_port_name,
                           *port_motor_position_command_list_[i]);

    char MotorCurrent_port_name[32];
    snprintf(MotorCurrent_port_name, sizeof(MotorCurrent_port_name),
             "MotorCurrent_%s", port_labels_[i].c_str());
    port_motor_current_list_[i] = new typeof(*port_motor_current_list_[i]);
    this->ports()->addPort(MotorCurrent_port_name,
                           *port_motor_current_list_[i]);

    char port_regulator_reset_name[32];
    snprintf(port_regulator_reset_name, sizeof(port_regulator_reset_name),
             "RegulatorResetOutput_%s", port_labels_[i].c_str());
    port_regulator_reset_list_[i] = new typeof(*port_regulator_reset_list_[i]);
    this->ports()->addPort(port_regulator_reset_name,
                           *port_regulator_reset_list_[i]);

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

bool HwModel::startHook() {
  port_is_synchronised_.write(true);
  port_is_hardware_panic_.write(false);
  return true;
}

void HwModel::updateHook() {

  for (int servo = 0; servo < number_of_servos_; servo++) {
    port_desired_input_list_[servo]->read(desired_input_[servo]);

    // prad jest w przeliczany do amperow
    desired_torque_(servo) = desired_input_(servo) * torque_constant_[servo]
        / input_current_units_per_amper_[servo];

    for (int iteration = 0; iteration < iteration_per_step_; iteration++) {
      effective_torque_(servo) = desired_torque_(servo)
          - motor_velocity_(servo) * viscous_friction_[servo];
      motor_acceleration_(servo) = effective_torque_(servo) / inertia_[servo];
      motor_velocity_(servo) += motor_acceleration_(servo) / m_factor_;
      motor_position_(servo) += motor_velocity_(servo) / m_factor_;
    }
    inc_motor_position_[servo] = motor_position_[servo] * enc_res_[servo]
        / (2.0 * M_PI);

    port_motor_position_list_[servo]->write(inc_motor_position_[servo]);
  }
}

ORO_CREATE_COMPONENT(HwModel)
