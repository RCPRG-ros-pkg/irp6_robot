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

#include "Irp6pSupervisor.h"

#include <vector>
#include <string>
#include "common_headers/string_colors.h"

Irp6pSupervisor::Irp6pSupervisor(const std::string& name)
    : TaskContext(name) {
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
}

Irp6pSupervisor::~Irp6pSupervisor() {
}

bool Irp6pSupervisor::configureHook() {
  return true;
}

bool Irp6pSupervisor::startHook() {
  port_is_synchronised_out_.write(true);
  port_is_hardware_busy_out_.write(false);
  port_is_hardware_panic_out_.write(false);

  return true;
}

void Irp6pSupervisor::updateHook() {
  std_msgs::Bool do_synchro;
  if (port_do_synchro_in_.read(do_synchro) == RTT::NewData) {
    if (do_synchro.data) {
      port_do_synchro_hi_mw_out_.write(do_synchro);
      std::cout << getName() << " Synchronisation commanded" << std::endl;
    }
  }

  std_msgs::Bool emergency_stop;
  if (port_emergency_stop_in_.read(emergency_stop) == RTT::NewData) {
    if (emergency_stop.data) {
      port_emergency_stop_hi_mw_out_.write(true);
      std::cout << RED << getName() << " Emergency stop commanded" << RESET
          << std::endl;
    }
  }

  bool generator_active;
  if (port_generator_active_in_.read(generator_active) == RTT::NewData) {
    port_is_hardware_busy_out_.write(generator_active);
  } else {
    port_is_hardware_busy_out_.write(false);
  }

  bool is_hi_mw_synchronised;
  if (port_is_synchronised_hi_mw_in_.read(is_hi_mw_synchronised)
      == RTT::NewData) {
    port_is_synchronised_out_.write(is_hi_mw_synchronised);
  }

  bool is_hi_mw_panic;
  if (port_is_hardware_panic_hi_mw_in_.read(is_hi_mw_panic) == RTT::NewData) {
    port_is_hardware_panic_out_.write(is_hi_mw_panic);
  }
}

ORO_CREATE_COMPONENT(Irp6pSupervisor)
