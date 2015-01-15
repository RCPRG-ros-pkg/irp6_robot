#ifndef Irp6otmM2J_H_
#define Irp6otmM2J_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>

class Irp6otmM2J : public RTT::TaskContext {
 public:
  Irp6otmM2J(const std::string& name);
  virtual ~Irp6otmM2J();

  bool configureHook();
  void updateHook();
 private:
  void mp2i(const double* motors, double* joints);
  RTT::OutputPort<Eigen::VectorXd> port_joint_position_;
  RTT::InputPort<Eigen::VectorXd> port_motor_position_;

  Eigen::VectorXd motor_position_, joint_position_;

  // properties
  std::vector<double> synchro_motor_position_;
};

#endif /* Irp6otmM2J_H_ */
