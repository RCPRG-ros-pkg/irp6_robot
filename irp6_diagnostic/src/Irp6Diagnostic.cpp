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
#include "Irp6Diagnostic.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

Irp6Diagnostic::Irp6Diagnostic(const std::string& name)
    : RTT::TaskContext(name) {

  this->ports()->addPort("Diagnostics", port_Diagnostics).doc("");
  this->ports()->addPort("SynchroStateIn", synchro_state_in_).doc(
      "Synchro State from HardwareInterface");
  this->ports()->addPort("HardwaPanicIn", hardware_panic_in_).doc(
      "Hardware Panic from HardwareInterface");

  this->addProperty("hardware_label", hardware_label_).doc("");
}

Irp6Diagnostic::~Irp6Diagnostic() {
}

bool Irp6Diagnostic::configureHook() {
  diagnostic_.status.resize(1);
  diagnostic_.status[0].values.resize(2);

  diagnostic_.status[0].name = hardware_label_ + " Hardware Interface";
  diagnostic_.status[0].values[0].key = "Synchro";
  diagnostic_.status[0].values[1].key = "HardwarePanic";

  return true;
}

bool Irp6Diagnostic::startHook() {
  return true;
}

void Irp6Diagnostic::updateHook() {
  bool synchro_state;
  bool hardware_panic_state;

  if (RTT::NewData == synchro_state_in_.read(synchro_state)) {
    if (synchro_state) {
      diagnostic_.status[0].values[0].value = "TRUE";
    } else {
      diagnostic_.status[0].values[0].value = "FALSE";
    }
  }

  if (RTT::NewData == hardware_panic_in_.read(hardware_panic_state)) {
    if (hardware_panic_state) {
      diagnostic_.status[0].values[1].value = "TRUE";
    } else {
      diagnostic_.status[0].values[1].value = "FALSE";
    }
  }

  if (hardware_panic_state) {
    diagnostic_.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnostic_.status[0].message = hardware_label_ + " Hardware Interface ERROR";
  } else if (!synchro_state) {
    diagnostic_.status[0].level = diagnostic_msgs::DiagnosticStatus::WARN;
    diagnostic_.status[0].message =
        "Hardware Interface WARNING - NOT SYNCHRONISED";
  } else {
    diagnostic_.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnostic_.status[0].message = hardware_label_ + " Hardware Interface OK";
  }

  port_Diagnostics.write(diagnostic_);
}

ORO_CREATE_COMPONENT(Irp6Diagnostic)

