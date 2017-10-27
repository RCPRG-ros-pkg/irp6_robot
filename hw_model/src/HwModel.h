/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HWMODEL_H_
#define HWMODEL_H_

#include <std_msgs/Bool.h>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "rosgraph_msgs/Clock.h"
#include "rtt_rosclock/rtt_rosclock.h"

class HwModel : public RTT::TaskContext {
 public:
  explicit HwModel(const std::string& name);
  virtual ~HwModel();

  bool configureHook();
  bool startHook();
  void updateHook();

 private:
  // RTT::InputPort<Eigen::VectorXd> port_desired_input_;
  // RTT::OutputPort<Eigen::VectorXd> port_motor_position_;

  // Ports
  RTT::OutputPort<rosgraph_msgs::Clock> port_ros_time_;

  std::vector<RTT::InputPort<double>*> port_desired_input_list_;
  std::vector<RTT::OutputPort<double>*> port_motor_position_list_;

  std::vector<RTT::OutputPort<double>*> desired_position_out_list_;
  std::vector<RTT::InputPort<double>*> port_motor_position_command_list_;
  std::vector<RTT::OutputPort<double>*> port_motor_current_list_;
  std::vector<RTT::OutputPort<bool>*> port_regulator_reset_list_;

  RTT::InputPort<bool> port_emergency_stop_;
  RTT::InputPort<std_msgs::Bool> port_do_synchro_;  // do przerobienia na wersje ze zwrotnym statusem synchronziacji
  RTT::OutputPort<bool> port_is_synchronised_;
  RTT::OutputPort<bool> port_is_hardware_panic_;

  Eigen::VectorXd motor_position_, motor_velocity_, motor_acceleration_,
      inc_motor_position_;
  Eigen::VectorXd desired_input_, desired_torque_, effective_torque_;

  int number_of_servos_;
  int m_factor_;
  bool first_update_hook_;

  // properties
  int robot_code_;
  int iteration_per_step_;
  int step_per_second_;
  std::vector<double> torque_constant_;
  std::vector<double> inertia_;
  std::vector<double> input_current_units_per_amper_;
  std::vector<double> viscous_friction_;
  std::vector<double> enc_res_;
  std::vector<std::string> port_labels_;
};

#endif  // HWMODEL_H_
