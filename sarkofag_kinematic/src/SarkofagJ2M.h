#ifndef SarkofagJ2M_H_
#define SarkofagJ2M_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>

class SarkofagJ2M: public RTT::TaskContext {
public:
  SarkofagJ2M(const std::string& name);
	virtual ~SarkofagJ2M();

	bool configureHook();
	void updateHook();
private:
	bool i2mp(const double* joints, double* motors);
	bool checkMotorPosition(const double * motor_position);
	RTT::InputPort<Eigen::VectorXd> port_joint_position_;
	RTT::OutputPort<Eigen::VectorXd > port_motor_position_;

	Eigen::VectorXd motor_position_, joint_position_;

	double  SYNCHRO_JOINT_POSITION[1];

};

#endif /* SarkofagJ2M_H_ */
