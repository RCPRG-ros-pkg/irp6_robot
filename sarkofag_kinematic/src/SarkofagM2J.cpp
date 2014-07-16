#include <rtt/Component.hpp>

#include "SarkofagM2J.h"
#include "SarkofagTransmission.h"


SarkofagM2J::SarkofagM2J(const std::string& name) : RTT::TaskContext(name, PreOperational) {

	this->ports()->addPort("MotorPosition", port_motor_position_);
	this->ports()->addPort("JointPosition", port_joint_position_);
}

SarkofagM2J::~SarkofagM2J() {

}

bool SarkofagM2J::configureHook() {
	motor_position_.resize(1);
	joint_position_.resize(1);
	return true;
}

void SarkofagM2J::updateHook() {
	port_motor_position_.read(motor_position_);
	mp2i(&motor_position_(0), &joint_position_(0));	
	port_joint_position_.write(joint_position_);
}

void SarkofagM2J::mp2i(const double* motors, double* joints)
{

// Przelicznik polozenia walu silnika napedowego kolumny w radianach
// na kat obrotu kolumny (wspolrzedna wewnetrzna) w radianach
  joints[0] = (motors[0] - SYNCHRO_MOTOR_POSITION[0]) / GEAR[0] + THETA[0];


}

ORO_CREATE_COMPONENT(SarkofagM2J)

