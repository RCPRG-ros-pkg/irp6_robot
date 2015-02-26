#include <rtt/Component.hpp>

#include "SarkofagJ2M.h"
#include "SarkofagTransmission.h"

SarkofagJ2M::SarkofagJ2M(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("MotorPosition", port_motor_position_);
  this->ports()->addPort("JointPosition", port_joint_position_);

}

SarkofagJ2M::~SarkofagJ2M() {

}

bool SarkofagJ2M::configureHook() {
  motor_position_.resize(1);
  joint_position_.resize(1);
  return true;
}

void SarkofagJ2M::updateHook() {
  if (port_joint_position_.read(joint_position_) == RTT::NewData) {
    if (i2mp(&joint_position_(0), &motor_position_(0))) {
      port_motor_position_.write(motor_position_);
    }
  }
}

bool SarkofagJ2M::i2mp(const double* joints, double* motors) {
  motors[0] = joints[0] * GEAR;
  return true;
}

ORO_CREATE_COMPONENT(SarkofagJ2M)

