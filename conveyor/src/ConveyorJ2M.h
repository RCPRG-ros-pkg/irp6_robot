#ifndef ConveyorJ2M_H_
#define ConveyorJ2M_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>

class ConveyorJ2M : public RTT::TaskContext {
 public:
  ConveyorJ2M(const std::string& name);
  virtual ~ConveyorJ2M();

  bool configureHook();
  void updateHook();
 private:
  bool i2mp(const double* joints, double* motors);
  RTT::InputPort<Eigen::VectorXd> port_joint_position_;
  RTT::OutputPort<Eigen::VectorXd> port_motor_position_;

  Eigen::VectorXd motor_position_, joint_position_;

};

#endif /* ConveyorJ2M_H_ */
