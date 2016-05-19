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

#include "../../irp6_supervisor/src/Irp6otSupervisor.h"

#include <vector>
#include <string>
#include "common_headers/string_colors.h"

Irp6otSupervisor::Irp6otSupervisor(const std::string& name)
    : TaskContext(name),
      robot_state_(NOT_OPERATIONAL),
      number_of_servos_(0),
      last_servo_synchro_(0),
      servos_state_changed_(0),
      auto_(false),
      hi_mw_synchronised(false),
      EC(NULL),
      control_mode_(PROFILE_CURRENT),
      Scheme(NULL),
      ec_servo_state_(INVALID) {
  // ports addition

  this->ports()->addPort("DoSynchroIn", port_do_synchro_in_);
  this->ports()->addPort("EmegencyStopIn", port_emergency_stop_in_);
  this->ports()->addPort("GeneratorActiveIn", port_generator_active_in_);

  this->ports()->addPort("IsSynchronisedHiMwIn",
                         port_is_synchronised_hi_mw_in_);
  this->ports()->addPort("IsHardwarePanicHiMwIn",
                         port_is_hardware_panic_hi_mw_in_);

  this->ports()->addPort("IsSynchronisedOut", port_is_synchronised_out_);
  this->ports()->addPort("IsHardwarePanicOut", port_is_hardware_panic_out_);
  this->ports()->addPort("IsHardwareBusyOut", port_is_hardware_busy_out_);

  this->ports()->addPort("DoSynchroHiMwOut", port_do_synchro_hi_mw_out_);
  this->ports()->addPort("EmergencyStopHiMwOut",
                         port_emergency_stop_hi_mw_out_);

  // properties addition

  this->addProperty("hal_component_name", hal_component_name_).doc("");
  this->addProperty("scheme_component_name", scheme_component_name_).doc("");
  this->addProperty("debug", debug_).doc("");
  this->addProperty("autostart", autostart_).doc("");
  this->addProperty("fault_autoreset", fault_autoreset_).doc("");
  this->addProperty("services_names", services_names_).doc("");
  this->addProperty("regulators_names", regulators_names_).doc("");

  this->addOperation("auto", &Irp6otSupervisor::autoRun, this, RTT::OwnThread)
      .doc("");
  this->addOperation("resetFault", &Irp6otSupervisor::resetFaultAll, this,
                     RTT::OwnThread).doc("");
  this->addOperation("disable", &Irp6otSupervisor::disableAll, this,
                     RTT::OwnThread).doc("");
  this->addOperation("enable", &Irp6otSupervisor::enableAll, this,
                     RTT::OwnThread).doc("");
  this->addOperation("beginHoming", &Irp6otSupervisor::beginHomingAll, this,
                     RTT::OwnThread).doc("");
  this->addOperation("homingDone", &Irp6otSupervisor::homingDoneAll, this,
                     RTT::OwnThread).doc("");
  this->addOperation("state", &Irp6otSupervisor::stateAll, this, RTT::OwnThread)
      .doc("");
}

Irp6otSupervisor::~Irp6otSupervisor() {
}

bool Irp6otSupervisor::configureHook() {
  if (hal_component_name_.empty() || scheme_component_name_.empty()) {
    return false;
  }

  number_of_servos_ = services_names_.size();
  if (number_of_servos_ != regulators_names_.size()) {
    std::cout << std::endl << RED << "[error] Irp6otSupervisor "
        << "configuration failed: wrong properties vector length in launch file."
        << RESET << std::endl;
    return false;
  }
  if (debug_) {
    std::cout << "servos: " << number_of_servos_ << std::endl;
  }

  digital_in_port_list_.resize(number_of_servos_);
  upper_limit_port_list_.resize(number_of_servos_);
  lower_limit_port_list_.resize(number_of_servos_);
  current_upper_limit_.resize(number_of_servos_);
  previous_upper_limit_.resize(number_of_servos_);
  current_lower_limit_.resize(number_of_servos_);
  previous_lower_limit_.resize(number_of_servos_);
  upper_limit_bit_mask_.resize(number_of_servos_);
  lower_limit_bit_mask_.resize(number_of_servos_);

  servo_state_.resize(number_of_servos_);

  for (int i = 0; i < number_of_servos_; i++) {
    current_upper_limit_[i] = previous_upper_limit_[i] = false;
    current_lower_limit_[i] = previous_lower_limit_[i] = false;

    upper_limit_bit_mask_[i] = (1 << UPPER_LIMIT_BIT_POSITION[i]);
    lower_limit_bit_mask_[i] = (1 << LOWER_LIMIT_BIT_POSITION[i]);

    servo_state_[i] = NOT_OPERATIONAL;

    char digital_in_port_name[32];
    snprintf(digital_in_port_name, sizeof(digital_in_port_name),
             "digital_in_%s", services_names_[i].c_str());
    digital_in_port_list_[i] = new typeof(*digital_in_port_list_[i]);
    this->ports()->addPort(digital_in_port_name, *digital_in_port_list_[i]);

    char UpperLimit_out_port_name[32];
    snprintf(UpperLimit_out_port_name, sizeof(UpperLimit_out_port_name),
             "upper_limit_out_%s", services_names_[i].c_str());
    upper_limit_port_list_[i] = new typeof(*upper_limit_port_list_[i]);
    this->ports()->addPort(UpperLimit_out_port_name,
                           *upper_limit_port_list_[i]);

    char LowerLimit_out_port_name[32];
    snprintf(LowerLimit_out_port_name, sizeof(LowerLimit_out_port_name),
             "lower_limit_out_%s", services_names_[i].c_str());
    lower_limit_port_list_[i] = new typeof(*lower_limit_port_list_[i]);
    this->ports()->addPort(LowerLimit_out_port_name,
                           *lower_limit_port_list_[i]);
  }
  return true;
}

bool Irp6otSupervisor::startHook() {
  port_is_synchronised_out_.write(false);
  port_is_hardware_busy_out_.write(false);
  port_is_hardware_panic_out_.write(false);

  EC = RTT::TaskContext::getPeer(hal_component_name_);
  Scheme = RTT::TaskContext::getPeer(scheme_component_name_);

  if (debug_) {
    std::cout << "EC_active:" << EC->isActive() << std::endl;
    std::cout << "EC_running:" << EC->isRunning() << std::endl;
    std::cout << "Scheme_active:" << Scheme->isActive() << std::endl;
    std::cout << "Scheme_running:" << Scheme->isRunning() << std::endl;
    stateAll();
  }

  auto_ = autostart_;

  for (int i = 0; i < number_of_servos_; i++) {
    upper_limit_port_list_[i]->write(current_upper_limit_[i]);
    lower_limit_port_list_[i]->write(current_lower_limit_[i]);
  }

  return true;
}

void Irp6otSupervisor::readLimits() {
  uint32_t di;
  bool command_emergency_stop = false;

  for (int i = 0; i < number_of_servos_; i++) {
    if (RTT::NewData == digital_in_port_list_[i]->read(di)) {
      if ((di & upper_limit_bit_mask_[i]) == upper_limit_bit_mask_[i]) {
        current_upper_limit_[i] = true;
      } else {
        current_upper_limit_[i] = false;
      }
      if ((di & lower_limit_bit_mask_[i]) == lower_limit_bit_mask_[i]) {
        current_lower_limit_[i] = true;
      } else {
        current_lower_limit_[i] = false;
      }

      if (current_upper_limit_[i] != previous_upper_limit_[i]) {
        upper_limit_port_list_[i]->write(current_upper_limit_[i]);
        if (current_upper_limit_[i] == true) {
          std::cout << std::endl << RED << getName()
              << " [error] UPPER LIMIT SWITCH DRIVE: " << i << RESET
              << std::endl << std::endl;
          command_emergency_stop = true;
        }
      }

      if (current_lower_limit_[i] != previous_lower_limit_[i]) {
        lower_limit_port_list_[i]->write(current_lower_limit_[i]);
        if (current_lower_limit_[i] == true) {
          std::cout << std::endl << RED << getName()
              << " [error] LOWER LIMIT SWITCH DRIVE: " << i << RESET
              << std::endl << std::endl;
          command_emergency_stop = true;
        }
      }

      previous_upper_limit_[i] = current_upper_limit_[i];
      previous_lower_limit_[i] = current_lower_limit_[i];

      if (command_emergency_stop) {
        port_emergency_stop_hi_mw_out_.write(true);
        std::cout << RED << getName()
            << " Emergency stop commanded due to limit switch" << RESET
            << std::endl;
      }
    }
  }
}

void Irp6otSupervisor::updateHook() {
  readLimits();

  if (port_do_synchro_in_.read(do_synchro) == RTT::NewData) {
    if (do_synchro.data) {
      std::cout << getName() << " Synchronisation commanded" << std::endl;
      beginHomingAll();
    }
  }

  std_msgs::Bool emergency_stop;
  if (port_emergency_stop_in_.read(emergency_stop) == RTT::NewData) {
    if (emergency_stop.data) {
      port_emergency_stop_hi_mw_out_.write(true);
      std::cout << RED << getName()
          << " Emergency stop commanded due to external command" << RESET
          << std::endl;
    }
  }

  bool generator_active;
  if (port_generator_active_in_.read(generator_active) == RTT::NewData) {
    port_is_hardware_busy_out_.write(generator_active);
  } else {
    port_is_hardware_busy_out_.write(false);
  }

  bool is_hi_mw_panic;
  if (port_is_hardware_panic_hi_mw_in_.read(is_hi_mw_panic) == RTT::NewData) {
    port_is_hardware_panic_out_.write(is_hi_mw_panic);
  }

  bool is_hi_mw_synchronised;
  if (port_is_synchronised_hi_mw_in_.read(is_hi_mw_synchronised)
      == RTT::NewData) {
    if (is_hi_mw_synchronised) {
      hi_mw_synchronised = true;
      std::cout << "ROBOT PREV. SYNCHRONIZED" << std::endl;
    } else {
      port_is_synchronised_out_.write(false);
    }
  }

  switch (robot_state_) {
    case NOT_OPERATIONAL:

      for (int i = 0; i < number_of_servos_; i++) {
        if (servo_state_[i] != NOT_SYNCHRONIZED) {
          RTT::Attribute<ECServoState> * servo_ec_state = (RTT::Attribute<
              ECServoState> *) EC->provides(services_names_[i])->getAttribute(
              "state");
          ec_servo_state_ = servo_ec_state->get();

          // set "resetFault" if fault
          if (ec_servo_state_ == FAULT) {
            RTT::OperationCaller<bool(void)> resetFault;
            resetFault = EC->provides(services_names_[i])->getOperation(
                "resetFault");
            resetFault.setCaller(this->engine());
            resetFault();
          }

          // set "enable" if powered on
          if (ec_servo_state_ == SWITCH_ON) {
            RTT::OperationCaller<bool(void)> enable;
            enable = EC->provides(services_names_[i])->getOperation("enable");
            enable.setCaller(this->engine());
            enable();
          }

          // servo enabled
          if (ec_servo_state_ == OPERATION_ENABLED) {
            std::cout << services_names_[i] << ": ENABLED" << std::endl;
            servo_state_[i] = NOT_SYNCHRONIZED;
            ++servos_state_changed_;
          }
        }
      }

      // all servos enabled
      if (servos_state_changed_ == number_of_servos_) {
        robot_state_ = NOT_SYNCHRONIZED;
        if (hi_mw_synchronised) {
          std::cout << "ROBOT PREV. SYNCHRONIZED" << std::endl;
        } else {
          std::cout << "ROBOT NOT SYNCHRONIZED" << std::endl;
        }
        servos_state_changed_ = 0;
      }
      break;

    case NOT_SYNCHRONIZED:
      if (auto_)
        robot_state_ = SYNCHRONIZING;

      if (hi_mw_synchronised) {
        for (int i = 0; i < number_of_servos_; i++) {
          RTT::Attribute<bool> * homing = (RTT::Attribute<bool> *) EC->provides(
              services_names_[i])->getAttribute("homing_done");
          if (homing->get()) {
            disable_vec_.clear();
            enable_vec_.clear();
            enable_vec_.push_back(regulators_names_[i]);
            RTT::OperationCaller<
                bool(const std::vector<std::string> &disable_block_names,
                     const std::vector<std::string> &enable_block_names,
                     const bool strict, const bool force)> switchBlocks;
            switchBlocks = Scheme->getOperation("switchBlocks");
            switchBlocks.setCaller(this->engine());
            switchBlocks(disable_vec_, enable_vec_, true, true);
            std::cout << regulators_names_[i] << ": ENABLED" << std::endl;
            ++servos_state_changed_;
          } else {
            RTT::OperationCaller<bool(void)> forceHomingDone;
            forceHomingDone = EC->provides(services_names_[i])->getOperation(
                "forceHomingDone");
            forceHomingDone.setCaller(this->engine());
            forceHomingDone();
          }
        }

        // all regulators enabled
        if (servos_state_changed_ == number_of_servos_) {
          robot_state_ = SYNCHRONIZED;
          std::cout << "ROBOT SYNCHRONIZED" << std::endl;
          servos_state_changed_ = 0;
        }
      }
      break;

    case SYNCHRONIZING:
      for (int i = 0; i < number_of_servos_; i++) {
        RTT::Attribute<ECServoState> * servo_state = (RTT::Attribute<
            ECServoState> *) EC->provides(services_names_[i])->getAttribute(
            "state");
        ec_servo_state_ = servo_state->get();

        if (ec_servo_state_ == OPERATION_ENABLED) {
          switch (servo_state_[i]) {
            case NOT_SYNCHRONIZED:
              if (i == last_servo_synchro_) {
                RTT::OperationCaller<bool(void)> beginHoming;
                beginHoming = EC->provides(services_names_[i])->getOperation(
                    "beginHoming");
                beginHoming.setCaller(this->engine());
                beginHoming();
                servo_state_[i] = SYNCHRONIZING;
                std::cout << services_names_[i] << ": SYNCHRONIZING"
                    << std::endl;
              }
              break;
            case SYNCHRONIZING:
              RTT::Attribute<bool> * homing = (RTT::Attribute<bool> *) EC
                  ->provides(services_names_[i])->getAttribute("homing_done");
              if (homing->get()) {
                servo_state_[i] = SYNCHRONIZED;
                std::cout << services_names_[i] << ": SYNCHRONIZED"
                    << std::endl;
                last_servo_synchro_ = i + 1;
                ++servos_state_changed_;

                // switch Regulator
                disable_vec_.clear();
                enable_vec_.clear();
                enable_vec_.push_back(regulators_names_[i]);
                RTT::OperationCaller<
                    bool(const std::vector<std::string> &disable_block_names,
                         const std::vector<std::string> &enable_block_names,
                         const bool strict, const bool force)> switchBlocks;
                switchBlocks = Scheme->getOperation("switchBlocks");
                switchBlocks.setCaller(this->engine());
                switchBlocks(disable_vec_, enable_vec_, true, true);
                std::cout << regulators_names_[i] << ": ENABLED" << std::endl;
              }
              break;
          }
        }
      }
      // all servos synhronized
      if (servos_state_changed_ == number_of_servos_) {
        robot_state_ = SYNCHRONIZED;
        std::cout << "EC SERVOS SYNCHRONIZED" << std::endl;
        servos_state_changed_ = 0;
        port_do_synchro_hi_mw_out_.write(do_synchro);
      }
      break;

    case SYNCHRONIZED:
      if (hi_mw_synchronised) {
        port_is_synchronised_out_.write(true);
        robot_state_ = RUNNING;
        std::cout << "ROBOT READY" << std::endl;
      }
      break;

    case RUNNING:
      if (fault_autoreset_) {
        resetFaultAll();
        enableAll();
      }
      break;
    default:
      break;
  }
}

void Irp6otSupervisor::autoRun() {
  auto_ = true;
}

bool Irp6otSupervisor::resetFaultAll() {
  bool out = true;
  for (int i = 0; i < number_of_servos_; i++) {
    RTT::Attribute<ECServoState> * servo_ec_state =
        (RTT::Attribute<ECServoState> *) EC->provides(services_names_[i])
            ->getAttribute("state");
    ec_servo_state_ = servo_ec_state->get();

    // set "resetFault" if fault
    if (ec_servo_state_ == FAULT) {
      RTT::OperationCaller<bool(void)> resetFault;
      resetFault = EC->provides(services_names_[i])->getOperation("resetFault");
      resetFault.setCaller(this->engine());
      out = out && resetFault();
    }
  }
  return out;
}

bool Irp6otSupervisor::enableAll() {
  bool out = true;
  for (int i = 0; i < number_of_servos_; i++) {
    RTT::Attribute<ECServoState> * servo_ec_state =
        (RTT::Attribute<ECServoState> *) EC->provides(services_names_[i])
            ->getAttribute("state");
    ec_servo_state_ = servo_ec_state->get();

    // set "enable" if powered on
    if (ec_servo_state_ == SWITCH_ON) {
      RTT::OperationCaller<bool(void)> enable;
      enable = EC->provides(services_names_[i])->getOperation("enable");
      enable.setCaller(this->engine());
      out = out && enable();
    }
  }
  return out;
}

bool Irp6otSupervisor::disableAll() {
  bool out = true;
  for (int i = 0; i < number_of_servos_; i++) {
    RTT::Attribute<ECServoState> * servo_ec_state =
        (RTT::Attribute<ECServoState> *) EC->provides(services_names_[i])
            ->getAttribute("state");
    ec_servo_state_ = servo_ec_state->get();

    // set "disable" if powered on
    RTT::OperationCaller<bool(void)> disable;
    disable = EC->provides(services_names_[i])->getOperation("disable");
    disable.setCaller(this->engine());
    out = out && disable();
  }
  return out;
}

void Irp6otSupervisor::beginHomingAll() {
  if (robot_state_ == NOT_SYNCHRONIZED)
    robot_state_ = SYNCHRONIZING;
}

void Irp6otSupervisor::homingDoneAll() {
  for (int i = 0; i < number_of_servos_; i++) {
    RTT::Attribute<bool> * homing = (RTT::Attribute<bool> *) EC->provides(
        services_names_[i])->getAttribute("homing_done");
    std::cout << services_names_[i] << ": " << homing->get() << std::endl;
  }
}

void Irp6otSupervisor::stateAll() {
  for (int i = 0; i < number_of_servos_; i++) {
    RTT::Attribute<ECServoState> * servo_ec_state =
        (RTT::Attribute<ECServoState> *) EC->provides(services_names_[i])
            ->getAttribute("state");
    ec_servo_state_ = servo_ec_state->get();
    std::cout << services_names_[i] << ": " << state_text(ec_servo_state_)
        << std::endl;
  }
}

std::string Irp6otSupervisor::state_text(ECServoState state) {
  switch (state) {
    case INVALID:
      return "INVALID";
    case NOT_READY_TO_SWITCH_ON:
      return "NOT_READY_TO_SWITCH_ON";
    case SWITCH_ON_DISABLED:
      return "SWITCH_ON_DISABLED";
    case READY_TO_SWITCH_ON:
      return "READY_TO_SWITCH_ON";
    case SWITCH_ON:
      return "SWITCH_ON";
    case OPERATION_ENABLED:
      return "OPERATION_ENABLED";
    case QUICK_STOP_ACTIVE:
      return "QUICK_STOP_ACTIVE";
    case FAULT_REACTION_ACTIVE:
      return "FAULT_REACTION_ACTIVE";
    case FAULT:
      return "FAULT";
    default:
      return "";
  }
}

ORO_CREATE_COMPONENT(Irp6otSupervisor)
