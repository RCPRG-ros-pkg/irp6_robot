#ifndef Irp6ptfgM2J_H_
#define Irp6ptfgM2J_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>

class Irp6ptfgM2J : public RTT::TaskContext {
 public:
  Irp6ptfgM2J(const std::string& name);
  virtual ~Irp6ptfgM2J();

  bool configureHook();
  void updateHook();
 private:
  void mp2i(const double* motors, double* joints);
  RTT::OutputPort<Eigen::VectorXd> port_joint_position_;
  RTT::InputPort<Eigen::VectorXd> port_motor_position_;

  Eigen::VectorXd motor_position_, joint_position_;



  //! Variable related to the computations of the gripper spread.
  double dir_a_7;

  //! Variable related to the computations of the gripper spread.
  double dir_b_7;

  //! Variable related to the computations of the gripper spread.
  double dir_c_7;

  //! Variable related to the computations of the gripper spread.


};

#endif /* Irp6ptfgM2J_H_ */
