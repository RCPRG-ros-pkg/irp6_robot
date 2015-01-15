#ifndef Irp6otmJ2M_H_
#define Irp6otmJ2M_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>

#include "Irp6otmTransmission.h"

class Irp6otmJ2M: public RTT::TaskContext {
public:
  Irp6otmJ2M(const std::string& name);
	virtual ~Irp6otmJ2M();

	bool configureHook();
	void updateHook();
private:
	bool i2mp(const double* joints, double* motors);
	bool checkMotorPosition(const double * motor_position);
	RTT::InputPort<Eigen::VectorXd> port_joint_position_;
	RTT::OutputPort<Eigen::VectorXd > port_motor_position_;

	Eigen::VectorXd motor_position_, joint_position_;

	double SYNCHRO_JOINT_POSITION[NUMBER_OF_SERVOS];

  // properties
  std::vector<double> synchro_motor_position_;
};

#endif /* Irp6otmJ2M_H_ */
