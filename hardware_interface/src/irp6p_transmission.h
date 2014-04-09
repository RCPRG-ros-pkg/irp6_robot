#ifndef IRP6PTransmission_H_
#define IRP6PTransmission_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>

class IRP6PTransmission: public RTT::TaskContext {
public:
	IRP6PTransmission(const std::string& name);
	virtual ~IRP6PTransmission();

	bool configureHook();
	void updateHook();
private:
	void mp2i(const double* motors, double* joints);
	RTT::OutputPort<Eigen::VectorXd> port_joint_position_;
	RTT::InputPort<Eigen::VectorXd > port_motor_position_;

	Eigen::VectorXd motor_position_, joint_position_;
};

#endif /* IRP6PTransmission_H_ */
