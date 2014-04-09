#ifndef IRP6PTransmission_H_
#define IRP6PTransmission_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>

class IRP6PTransmissionInv: public RTT::TaskContext {
public:
	IRP6PTransmissionInv(const std::string& name);
	virtual ~IRP6PTransmissionInv();

	bool configureHook();
	void updateHook();
private:
	bool i2mp(const double* joints, double* motors);
	bool checkMotorPosition(const double * motor_position);
	RTT::InputPort<Eigen::VectorXd> port_joint_position_;
	RTT::OutputPort<Eigen::VectorXd > port_motor_position_;

	Eigen::VectorXd motor_position_, joint_position_;
};

#endif /* IRP6PTransmission_H_ */
