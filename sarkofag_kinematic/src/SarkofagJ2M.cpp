#include <rtt/Component.hpp>

#include "SarkofagJ2M.h"
#include "SarkofagTransmission.h"

SarkofagJ2M::SarkofagJ2M(const std::string& name) : RTT::TaskContext(name, PreOperational) {

	this->ports()->addPort("MotorPosition", port_motor_position_);
	this->ports()->addPort("JointPosition", port_joint_position_);
}

SarkofagJ2M::~SarkofagJ2M() {

}

bool SarkofagJ2M::configureHook() {
	motor_position_.resize(1);
	joint_position_.resize(1);


	SYNCHRO_JOINT_POSITION[0] =  SYNCHRO_MOTOR_POSITION[0] - GEAR[0] * THETA[0];

	return true;
}

void SarkofagJ2M::updateHook() {
	if(port_joint_position_.read(joint_position_) == RTT::NewData) {
	  if(i2mp(&joint_position_(0), &motor_position_(0))) {
	    port_motor_position_.write(motor_position_);
	  }
	}
}

bool SarkofagJ2M::i2mp(const double* joints, double* motors)
{

  // Obliczanie kata obrotu walu silnika napedowego kolumny
  motors[0] = GEAR[0] * joints[0] + SYNCHRO_JOINT_POSITION[0];



  return checkMotorPosition(motors);
}

bool SarkofagJ2M::checkMotorPosition(const double * motor_position)
{

  if (motor_position[0] < LOWER_MOTOR_LIMIT[0]) // Kat f1 mniejszy od minimalnego
    return false;//throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_0);
  else if (motor_position[0] > UPPER_MOTOR_LIMIT[0]) // Kat f1 wiekszy od maksymalnego
    return false;//throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_0);


  return true;
} //: check_motor_position


ORO_CREATE_COMPONENT(SarkofagJ2M)

