// Copyright WUT 2014
#include <rtt/Component.hpp>

#include "ImpedanceJointTorque.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

ImpedanceJointTorque::ImpedanceJointTorque(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

}

ImpedanceJointTorque::~ImpedanceJointTorque() {
}

bool ImpedanceJointTorque::configureHook() {
  return true;
}

bool ImpedanceJointTorque::startHook() {
  return true;
}

void ImpedanceJointTorque::updateHook() {
}

ORO_CREATE_COMPONENT(ImpedanceJointTorque)

