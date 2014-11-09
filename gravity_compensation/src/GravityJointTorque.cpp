// Copyright WUT 2014
#include <rtt/Component.hpp>

#include "GravityJointTorque.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

GravityJointTorque::GravityJointTorque(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

}

GravityJointTorque::~GravityJointTorque() {
}

bool GravityJointTorque::configureHook() {
  return true;
}

bool GravityJointTorque::startHook() {
  return true;
}

void GravityJointTorque::updateHook() {
}

ORO_CREATE_COMPONENT(GravityJointTorque)

