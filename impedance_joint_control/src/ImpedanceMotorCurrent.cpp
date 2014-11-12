// Copyright WUT 2014
#include <rtt/Component.hpp>

#include "ImpedanceMotorCurrent.h"
#include "eigen_conversions/eigen_msg.h"

ImpedanceMotorCurrent::ImpedanceMotorCurrent(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {
}

ImpedanceMotorCurrent::~ImpedanceMotorCurrent() {
}

bool ImpedanceMotorCurrent::configureHook() {
  return true;
}

bool ImpedanceMotorCurrent::startHook() {
  return true;
}

void ImpedanceMotorCurrent::updateHook() {
}

ORO_CREATE_COMPONENT(ImpedanceMotorCurrent)

