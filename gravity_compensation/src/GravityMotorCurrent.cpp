// Copyright WUT 2014
#include <rtt/Component.hpp>

#include "GravityMotorCurrent.h"
#include "eigen_conversions/eigen_msg.h"

GravityMotorCurrent::GravityMotorCurrent(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {
}

GravityMotorCurrent::~GravityMotorCurrent() {
}

bool GravityMotorCurrent::configureHook() {
  return true;
}

bool GravityMotorCurrent::startHook() {
  return true;
}

void GravityMotorCurrent::updateHook() {
}

ORO_CREATE_COMPONENT(GravityMotorCurrent)

