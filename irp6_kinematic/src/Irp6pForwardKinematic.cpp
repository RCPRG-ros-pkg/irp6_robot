#include <rtt/Component.hpp>

#include "Irp6pForwardKinematic.h"


Irp6pForwardKinematic::Irp6pForwardKinematic(const std::string& name) : RTT::TaskContext(name, PreOperational) {

	this->ports()->addPort("JointPosition", port_joint_position_);
	this->ports()->addPort("OutputPose", port_output_pose_);


}

Irp6pForwardKinematic::~Irp6pForwardKinematic() {

}

bool Irp6pForwardKinematic::configureHook() {

	return true;
}

void Irp6pForwardKinematic::updateHook() {

}

void Irp6pForwardKinematic::mp2i(const double* motors, double* joints)
{

}

ORO_CREATE_COMPONENT(Irp6pForwardKinematic)

