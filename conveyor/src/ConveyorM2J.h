#ifndef ConveyorM2J_H_
#define ConveyorM2J_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>

class ConveyorM2J : public RTT::TaskContext {
 public:
  ConveyorM2J(const std::string& name);
  virtual ~ConveyorM2J();

  bool configureHook();
  void updateHook();
 private:
  void mp2i(const double* motors, double* joints);
  RTT::OutputPort<Eigen::VectorXd> port_joint_position_;
  RTT::InputPort<Eigen::VectorXd> port_motor_position_;

  Eigen::VectorXd motor_position_, joint_position_;
};

#endif /* ConveyorM2J */
