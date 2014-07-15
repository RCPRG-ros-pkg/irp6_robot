#include <rtt/Component.hpp>

#include "Irp6ptfgJ2M.h"
#include "Irp6ptfgTransmission.h"

Irp6ptfgJ2M::Irp6ptfgJ2M(const std::string& name) : RTT::TaskContext(name, PreOperational) {

	this->ports()->addPort("MotorPosition", port_motor_position_);
	this->ports()->addPort("JointPosition", port_joint_position_);



  inv_a_7 = 0.3531946456e-5;
  inv_b_7 = 0.2622172716e19;
  inv_c_7 = -0.2831300000e20;
  inv_d_7 = -2564.034320;



}

Irp6ptfgJ2M::~Irp6ptfgJ2M() {

}

bool Irp6ptfgJ2M::configureHook() {
	motor_position_.resize(1);
	joint_position_.resize(1);
	return true;
}

void Irp6ptfgJ2M::updateHook() {
	if(port_joint_position_.read(joint_position_) == RTT::NewData) {
	  if(i2mp(&joint_position_(0), &motor_position_(0))) {
	    port_motor_position_.write(motor_position_);
	  }
	}
}

bool Irp6ptfgJ2M::i2mp(const double* joints, double* motors)
{

  // Obliczenie kata obrotu walu silnika napedowego chwytaka.
  motors[0] = inv_a_7 * sqrt(inv_b_7 + inv_c_7 * joints[0]) + inv_d_7;
  return checkMotorPosition(motors);
}

bool Irp6ptfgJ2M::checkMotorPosition(const double * motor_position)
{

  if (motor_position[0] < LOWER_MOTOR_LIMIT[0]) // Kat f1 mniejszy od minimalnego
    return false;//throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_0);
  else if (motor_position[0] > UPPER_MOTOR_LIMIT[0]) // Kat f1 wiekszy od maksymalnego
    return false;//throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_0);


  return true;
} //: check_motor_position


ORO_CREATE_COMPONENT(Irp6ptfgJ2M)

