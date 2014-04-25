#ifndef Irp6pForwardKinematic_H_
#define Irp6pForwardKinematic_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>

class Irp6pForwardKinematic: public RTT::TaskContext {
public:
	Irp6pForwardKinematic(const std::string& name);
	virtual ~Irp6pForwardKinematic();

	bool configureHook();
	void updateHook();
private:
	void mp2i(const double* motors, double* joints);
	RTT::OutputPort<Eigen::VectorXd> port_joint_position_;
	RTT::InputPort<Eigen::VectorXd > port_motor_position_;

	Eigen::VectorXd motor_position_, joint_position_;
};

#endif /* Irp6pForwardKinematic */
