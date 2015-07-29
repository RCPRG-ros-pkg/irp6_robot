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

#ifndef HARDWAREINTERFACE_H_
#define HARDWAREINTERFACE_H_

#include <hi_msgs/HardwareInterfacePort.h>
#include <std_msgs/Bool.h>

#include <vector>
#include <string>

#include "hi_moxa.h"

#define HI_SERVOS_NR 17

typedef enum {
  NOT_SYNCHRONIZED,
  PRE_SERVOING,
  SERVOING,
  PRE_SYNCHRONIZING,
  SYNCHRONIZING
} State;

typedef enum {
  MOVE_TO_SYNCHRO_AREA,
  STOP,
  MOVE_FROM_SYNCHRO_AREA,
  WAIT_FOR_IMPULSE,
  SYNCHRO_END
} SynchroState;

class HardwareInterface : public RTT::TaskContext {
 private:
  std::vector<RTT::InputPort<double>*> computedReg_in_list_;

  std::vector<RTT::OutputPort<double>*> desired_position_out_list_;

  std::vector<RTT::OutputPort<double>*> port_motor_position_list_;
  std::vector<RTT::OutputPort<double>*> port_motor_increment_list_;
  std::vector<RTT::OutputPort<double>*> port_motor_current_list_;

  RTT::InputPort<bool> port_emergency_stop_;
  RTT::InputPort<std_msgs::Bool> port_do_synchro_;  // do przerobienia na wersje ze zwrotnym statusem synchronziacji
  RTT::OutputPort<bool> port_is_synchronised_;
  RTT::OutputPort<bool> port_is_hardware_panic_;

  std::vector<RTT::InputPort<double>*> port_motor_position_command_list_;

  Eigen::VectorXd motor_position_, motor_increment_, motor_current_,
      motor_position_command_;

  std::vector<std::string> ports_adresses_;
  std::vector<int> max_current_;
  std::vector<double> max_increment_;
  std::vector<unsigned int> card_indexes_;
  std::vector<double> enc_res_;
  std::vector<double> synchro_step_coarse_;
  std::vector<double> synchro_step_fine_;
  std::vector<bool> current_mode_;
  std::vector<bool> synchro_needed_;

  int number_of_drives_;
  int error_msg_hardware_panic_;

  // Properties
  std::vector<std::string> active_motors_;
  bool auto_synchronize_;
  bool test_mode_;
  int timeouts_to_print_;
  int tx_prefix_len_;
  int rwh_nsec_;
  hi_msgs::HardwareInterfacePort hi_port_param_[HI_SERVOS_NR];
  std::string hardware_hostname_;

  int synchro_stop_iter_;
  int synchro_start_iter_;
  int servo_start_iter_;

  double counter_;

  State state_;
  SynchroState synchro_state_;
  int synchro_drive_;

  std::vector<double> desired_position_;
  std::vector<double> desired_position_increment_;
  std::vector<double> max_pos_inc_;

  std::vector<double> increment_;
  std::vector<double> motor_pos_;
  std::vector<double> pwm_or_current_;

  std::vector<RTT::TaskContext*> servo_list_;

  hi_moxa::HI_moxa *hi_;

  uint16_t convert_to_115(float input);
  void test_mode_sleep();

 public:
  explicit HardwareInterface(const std::string& name);
  ~HardwareInterface();

  bool configureHook();
  bool startHook();
  void updateHook();
};

#endif  // HARDWAREINTERFACE_H_
