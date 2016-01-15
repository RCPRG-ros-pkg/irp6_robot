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

#include <string>

#include "HardwareInterfaceMW.h"

HardwareInterfaceMW::HardwareInterfaceMW(const std::string& name)
    : TaskContext(name, PreOperational),
      synchro_start_iter_(0),
      servo_start_iter_(0),
      synchro_stop_iter_(0),
      test_mode_(0),
      hi_(NULL),
      state_(NOT_SYNCHRONIZED),
      synchro_drive_(0),
      tx_prefix_len_(0),
      synchro_state_(MOVE_TO_SYNCHRO_AREA),
      rwh_nsec_(1200000),
      timeouts_to_print_(1),
      number_of_drives_(0),
      error_msg_hardware_panic_(0) {

  // ports addition

  this->ports()->addPort("EmergencyStopIn", port_emergency_stop_);
  this->ports()->addPort("GeneratorActiveIn", port_generator_active_);
  this->ports()->addPort("DoSynchroIn", port_do_synchro_);
  this->ports()->addPort("IsSynchronised", port_is_synchronised_);
  this->ports()->addPort("IsHardwarePanic", port_is_hardware_panic_);
  this->ports()->addPort("IsHardwareBusy", port_is_hardware_busy_);

  this->ports()->addPort("DesiredHwModelOutput", port_desired_hw_model_output_);
  this->ports()->addPort("HwModelMotorPosition", port_hw_model_motor_position_);

  // properties addition

  this->addProperty("active_motors", active_motors_).doc("");
  this->addProperty("hardware_hostname", hardware_hostname_).doc("");
  this->addProperty("auto_synchronize", auto_synchronize_).doc("");
  this->addProperty("test_mode", test_mode_).doc("");
  this->addProperty("timeouts_to_print", timeouts_to_print_).doc("");
  this->addProperty("rwh_nsec", rwh_nsec_).doc("");
  this->addProperty("irp6pm_0", hi_port_param_[0]).doc("");
  this->addProperty("irp6pm_1", hi_port_param_[1]).doc("");
  this->addProperty("irp6pm_2", hi_port_param_[2]).doc("");
  this->addProperty("irp6pm_3", hi_port_param_[3]).doc("");
  this->addProperty("irp6pm_4", hi_port_param_[4]).doc("");
  this->addProperty("irp6pm_5", hi_port_param_[5]).doc("");
  this->addProperty("irp6ptfg", hi_port_param_[6]).doc("");
  this->addProperty("conveyor", hi_port_param_[7]).doc("");
  this->addProperty("irp6otm_0", hi_port_param_[8]).doc("");
  this->addProperty("irp6otm_1", hi_port_param_[9]).doc("");
  this->addProperty("irp6otm_2", hi_port_param_[10]).doc("");
  this->addProperty("irp6otm_3", hi_port_param_[11]).doc("");
  this->addProperty("irp6otm_4", hi_port_param_[12]).doc("");
  this->addProperty("irp6otm_5", hi_port_param_[13]).doc("");
  this->addProperty("irp6otm_6", hi_port_param_[14]).doc("");
  this->addProperty("irp6ottfg", hi_port_param_[15]).doc("");
}

HardwareInterfaceMW::~HardwareInterfaceMW() {
}

void HardwareInterfaceMW::configureHookInitVariables() {
  for (size_t j = 0; j < HI_SERVOS_NR; j++) {
    if (std::find(active_motors_.begin(), active_motors_.end(),
                  hi_port_param_[j].label) != active_motors_.end()) {
      number_of_drives_++;
    }
  }

  // dynamic ports list initialization

// std::cout << "number_of_drives_: " << number_of_drives_ << std::endl;

  computedReg_in_list_.resize(number_of_drives_);
  desired_position_out_list_.resize(number_of_drives_);
  port_motor_position_command_list_.resize(number_of_drives_);
  port_motor_position_list_.resize(number_of_drives_);
  port_motor_increment_list_.resize(number_of_drives_);
  port_motor_current_list_.resize(number_of_drives_);

  ports_adresses_.resize(number_of_drives_);
  max_current_.resize(number_of_drives_);
  max_increment_.resize(number_of_drives_);
  card_indexes_.resize(number_of_drives_);
  enc_res_.resize(number_of_drives_);
  synchro_step_coarse_.resize(number_of_drives_);
  synchro_step_fine_.resize(number_of_drives_);
  current_mode_.resize(number_of_drives_);
  synchro_needed_.resize(number_of_drives_);

  desired_position_.resize(number_of_drives_);

  pwm_or_current_.resize(number_of_drives_);
  max_pos_inc_.resize(number_of_drives_);
  motor_increment_.resize(number_of_drives_);
  port_desired_hw_model_output_.setDataSample(pwm_or_current_);

  motor_position_.resize(number_of_drives_);
  previous_motor_position_.resize(number_of_drives_);

  motor_current_.resize(number_of_drives_);
  motor_position_command_.resize(number_of_drives_);

  for (int i = 0; i < number_of_drives_; i++) {
    motor_increment_[i] = 0;
    pwm_or_current_[i] = 0.0;
    max_pos_inc_[i] = 0.0;
  }

  int i = -1;
  for (size_t j = 0; j < HI_SERVOS_NR; j++) {
    //  std::cout << "j: " << j << std::endl;

    if (std::find(active_motors_.begin(), active_motors_.end(),
                  hi_port_param_[j].label) != active_motors_.end()) {
      i++;
      //  std::cout << "i: " << i << std::endl;
      char computedReg_in_port_name[32];
      snprintf(computedReg_in_port_name, sizeof(computedReg_in_port_name),
               "computedReg_in_%s", hi_port_param_[j].label.c_str());
      computedReg_in_list_[i] = new typeof(*computedReg_in_list_[i]);
      this->ports()->addPort(computedReg_in_port_name,
                             *computedReg_in_list_[i]);

      char DesiredPosition_out_port_name[32];
      snprintf(DesiredPosition_out_port_name,
               sizeof(DesiredPosition_out_port_name), "DesiredPosition_out_%s",
               hi_port_param_[j].label.c_str());
      desired_position_out_list_[i] =
          new typeof(*desired_position_out_list_[i]);
      this->ports()->addPort(DesiredPosition_out_port_name,
                             *desired_position_out_list_[i]);

      char MotorPositionCommand_port_name[32];
      snprintf(MotorPositionCommand_port_name,
               sizeof(MotorPositionCommand_port_name),
               "MotorPositionCommand_%s", hi_port_param_[j].label.c_str());
      port_motor_position_command_list_[i] =
          new typeof(*port_motor_position_command_list_[i]);
      this->ports()->addPort(MotorPositionCommand_port_name,
                             *port_motor_position_command_list_[i]);

      char MotorPosition_port_name[32];
      snprintf(MotorPosition_port_name, sizeof(MotorPosition_port_name),
               "MotorPosition_%s", hi_port_param_[j].label.c_str());
      port_motor_position_list_[i] = new typeof(*port_motor_position_list_[i]);
      this->ports()->addPort(MotorPosition_port_name,
                             *port_motor_position_list_[i]);

      char MotorIncrement_port_name[32];
      snprintf(MotorIncrement_port_name, sizeof(MotorIncrement_port_name),
               "MotorIncrement_%s", hi_port_param_[j].label.c_str());
      port_motor_increment_list_[i] =
          new typeof(*port_motor_increment_list_[i]);
      this->ports()->addPort(MotorIncrement_port_name,
                             *port_motor_increment_list_[i]);

      char MotorCurrent_port_name[32];
      snprintf(MotorCurrent_port_name, sizeof(MotorCurrent_port_name),
               "MotorCurrent_%s", hi_port_param_[j].label.c_str());
      port_motor_current_list_[i] = new typeof(*port_motor_current_list_[i]);
      this->ports()->addPort(MotorCurrent_port_name,
                             *port_motor_current_list_[i]);

      //   std::cout << "i: "  << i << " label: " << hi_port_param_[j].label << std::endl;
      ports_adresses_[i] = hi_port_param_[j].ports_adresses;
      //   std::cout << "ports_adresses_: "  << ports_adresses_[i] << std::endl;
      max_current_[i] = hi_port_param_[j].max_current;
      //   std::cout << "max_current_: "  << max_current_[i] << std::endl;
      max_increment_[i] = hi_port_param_[j].max_increment;
//    std::cout << "max_increment_: "  << max_increment_[i] << std::endl;
      card_indexes_[i] = hi_port_param_[j].card_indexes;
      //   std::cout << "ports_adresses_: "  << ports_adresses_[i] << std::endl;
      enc_res_[i] = hi_port_param_[j].enc_res;
      //  std::cout << "card_indexes_: "  << card_indexes_[i] << std::endl;
      synchro_step_coarse_[i] = hi_port_param_[j].synchro_step_coarse;
      //  std::cout << "synchro_step_coarse_: "  << synchro_step_coarse_[i] << std::endl;
      synchro_step_fine_[i] = hi_port_param_[j].synchro_step_fine;
      //  std::cout << "synchro_step_fine_: "  << synchro_step_fine_[i] << std::endl;
      current_mode_[i] = hi_port_param_[j].current_mode;
      //   std::cout << "current_mode_: "  << current_mode_[i] << std::endl;
      synchro_needed_[i] = hi_port_param_[j].synchro_needed;
      //   std::cout << "synchro_needed_: "  << synchro_needed_[i] << std::endl << std::endl;
    }
  }
}

bool HardwareInterfaceMW::configureHookInitHardware() {
  try {
    hi_ = new hi_moxa::HI_moxa(number_of_drives_ - 1, card_indexes_,
                               max_increment_, tx_prefix_len_, getName());
    char hostname[128];
    if (gethostname(hostname, sizeof(hostname)) == -1) {
      perror("gethostname()");
      hostname[0] = '\0';
    }

    if (std::string(hostname) != hardware_hostname_) {
      std::cout << std::endl << RED << getName()
                << " [error] ERROR wrong host_name for hardware_mode: "
                << std::string(hostname) << ", " << hardware_hostname_ << RESET
                << std::endl << std::endl;
    }

    std::cout << getName() << " hostname: " << hostname << std::endl;

    hi_->init(ports_adresses_);

    for (int i = 0; i < number_of_drives_; i++) {
      hi_->set_parameter_now(i, NF_COMMAND_SetDrivesMaxCurrent,
                             (int16_t) max_current_[i]);
    }
    /*
     NF_STRUCT_Regulator tmpReg = { convert_to_115(0.0600), convert_to_115(
     0.0500), convert_to_115(0.0), 0 };

     hi_->set_pwm_mode(0);
     hi_->set_parameter_now(0, NF_COMMAND_SetCurrentRegulator, tmpReg);
     */

    for (int i = 0; i < number_of_drives_; i++) {
      if (current_mode_[i]) {
        hi_->set_current_mode(i);
      } else {
        hi_->set_pwm_mode(i);
      }
    }
  } catch (std::exception& e) {
    log(RTT::Info) << e.what() << RTT::endlog();

    std::cout
        << std::endl
        << RED
        << getName()
        << " [error] ERROR configuring HardwareInterfaceMW, check power switches"
        << RESET << std::endl << std::endl;

    return false;
  }
  return true;
}

bool HardwareInterfaceMW::configureHook() {
  configureHookInitVariables();

  if (test_mode_) {
    return true;
  } else {
    return configureHookInitHardware();
  }
}

uint16_t HardwareInterfaceMW::convert_to_115(float input) {
  uint16_t output = 0;

  if (input >= 1.0) {
    printf("convert_to_115 input bigger or equal then 1.0\n");
    return 0;
  } else if (input < -1.0) {
    printf("convert_to_115 input lower then -1.0\n");
    return 0;
  } else if (input < 0.0) {
    output = 65535 + static_cast<int>(input * 32768.0);
  } else if (input >= 0.0) {
    output = (uint16_t) (input * 32768.0);
  }

//  printf("convert_to_115 i: %f, o: %x\n",input, output);

  return output;
}

void HardwareInterfaceMW::test_mode_sleep() {
  struct timespec delay;
  delay.tv_nsec = rwh_nsec_ + 200000;
  delay.tv_sec = 0;

  nanosleep(&delay, NULL);
}

bool HardwareInterfaceMW::startHook() {
  try {
    if (!test_mode_) {
      hi_->write_read_hardware(rwh_nsec_, 0);
      servo_start_iter_ = 200;
      for (int i = 0; i < number_of_drives_; i++) {
        motor_position_(i) = static_cast<double>(hi_->get_position(i))
            * ((2.0 * M_PI) / enc_res_[i]);
      }
      if (!hi_->robot_synchronized()) {
        port_is_synchronised_.write(false);
        port_is_hardware_busy_.write(true);
        RTT::log(RTT::Info) << "Robot not synchronized" << RTT::endlog();
        synchro_start_iter_ = 500;
        synchro_stop_iter_ = 1000;
        synchro_state_ = MOVE_TO_SYNCHRO_AREA;
        synchro_drive_ = 0;
        if (auto_synchronize_) {
          RTT::log(RTT::Info) << "Auto synchronize" << RTT::endlog();
          std::cout << getName() << " Auto synchronize" << std::endl;
          state_ = PRE_SYNCHRONIZING;
        } else {
          RTT::log(RTT::Info) << "Manual synchronize" << RTT::endlog();
          std::cout << getName() << " Manual synchronize" << std::endl;
          state_ = NOT_SYNCHRONIZED;
        }
      } else {
        port_is_synchronised_.write(true);
        RTT::log(RTT::Info) << "Robot synchronized" << RTT::endlog();

        for (int i = 0; i < number_of_drives_; i++) {
          motor_position_command_(i) = motor_position_(i);
          motor_increment_(i) = static_cast<double>(hi_->get_increment(i));
          motor_current_(i) = static_cast<double>(hi_->get_current(i));
        }

        state_ = PRE_SERVOING;
      }
    } else {
      test_mode_sleep();
      port_is_synchronised_.write(true);
      port_is_hardware_busy_.write(false);
      RTT::log(RTT::Info) << "HI test mode activated" << RTT::endlog();

      for (int i = 0; i < number_of_drives_; i++) {
        motor_position_command_(i) = motor_position_(i) =
            previous_motor_position_(i) = 0.0;
        motor_increment_(i) = 0.0;
        motor_current_(i) = 0.0;
      }

      state_ = PRE_SERVOING;
    }
  } catch (const std::exception& e) {
    RTT::log(RTT::Error) << e.what() << RTT::endlog();
    return false;
  }

  for (int i = 0; i < number_of_drives_; i++) {
    port_motor_position_list_[i]->write(
        static_cast<double>(motor_position_[i]));
    port_motor_increment_list_[i]->write(
        static_cast<double>(motor_increment_[i]));
    port_motor_current_list_[i]->write(static_cast<double>(motor_current_[i]));
    desired_position_[i] = motor_position_(i);
  }
  return true;
}

void HardwareInterfaceMW::updateHook() {
  updateHookInit();
  updateHookStateMachine();
}

void HardwareInterfaceMW::updateHookInit() {
  for (int i = 0; i < number_of_drives_; i++) {
    if (RTT::NewData != computedReg_in_list_[i]->read(pwm_or_current_[i])) {
      RTT::log(RTT::Error) << "NO PWM DATA" << RTT::endlog();
    }
  }

  if (!test_mode_) {
    for (int i = 0; i < number_of_drives_; i++) {
      if (current_mode_[i]) {
        hi_->set_current(i, pwm_or_current_[i]);
      } else {
        hi_->set_pwm(i, pwm_or_current_[i]);
      }
    }

    hi_->write_read_hardware(rwh_nsec_, timeouts_to_print_);
    if (hi_->hardware_panic) {
      port_is_hardware_panic_.write(true);
      if (error_msg_hardware_panic_ == 0) {
        std::cout << RED << std::endl << getName() << " [error] hardware panic"
                  << RESET << std::endl << std::endl;
        error_msg_hardware_panic_++;
      }
    } else {
      port_is_hardware_panic_.write(false);
    }
    for (int i = 0; i < number_of_drives_; i++) {
      motor_position_(i) = static_cast<double>(hi_->get_position(i))
          * ((2.0 * M_PI) / enc_res_[i]);
    }
  } else {  // test_mode==true
    for (int i = 0; i < number_of_drives_; i++) {
      previous_motor_position_(i) = motor_position_(i);
    }
    port_hw_model_motor_position_.read(motor_position_);

    port_desired_hw_model_output_.write(pwm_or_current_);
    test_mode_sleep();
    port_is_hardware_panic_.write(false);
  }

  bool emergency_stop;
  if (port_emergency_stop_.read(emergency_stop) == RTT::NewData) {
    if (emergency_stop) {
      if (!test_mode_) {
        hi_->set_hardware_panic();
      } else if (error_msg_hardware_panic_ == 0) {
        std::cout << RED << std::endl << getName() << " [error] hardware panic"
                  << RESET << std::endl << std::endl;
        error_msg_hardware_panic_++;
      }
    }
  }

  for (int i = 0; i < number_of_drives_; i++) {
    if (!test_mode_) {
      motor_current_(i) = static_cast<double>(hi_->get_current(i));
      motor_increment_[i] = hi_->get_increment(i);
    } else {  // test_mode == true
      motor_current_(i) = static_cast<double>(pwm_or_current_(i));
      motor_increment_[i] = (motor_position_(i) - previous_motor_position_(i))
          * enc_res_[i] / (2.0 * M_PI);
    }
    port_motor_current_list_[i]->write(static_cast<double>(motor_current_[i]));
    port_motor_increment_list_[i]->write(
        static_cast<double>(motor_increment_[i]));
  }
}

void HardwareInterfaceMW::updateHookStateMachine() {
  switch (state_) {
    case NOT_SYNCHRONIZED: {
      std_msgs::Bool do_synchro;
      if (port_do_synchro_.read(do_synchro) == RTT::NewData) {
        if (do_synchro.data) {
          if (!test_mode_) {
            state_ = PRE_SYNCHRONIZING;
          }
          std::cout << getName() << " NOT_SYNCHRONIZED" << std::endl;
        }
      }
    }
      break;

    case PRE_SERVOING:

      for (int i = 0; i < number_of_drives_; i++) {
        motor_position_command_(i) = motor_position_(i);
        port_motor_position_list_[i]->write(
            static_cast<double>(motor_position_[i]));
      }

      if ((servo_start_iter_--) <= 0) {
        state_ = SERVOING;
        port_is_hardware_busy_.write(false);
        std::cout << getName() << " Servoing started" << std::endl;
      }

      break;

    case SERVOING: {
      bool generator_active;
      if (port_generator_active_.read(generator_active) == RTT::NewData) {
        port_is_hardware_busy_.write(generator_active);
      } else {
        port_is_hardware_busy_.write(false);
      }

      for (int i = 0; i < number_of_drives_; i++) {
        if (port_motor_position_command_list_[i]->read(
            motor_position_command_[i]) == RTT::NewData) {
          desired_position_[i] = motor_position_command_(i);
        }

        port_motor_position_list_[i]->write(
            static_cast<double>(motor_position_[i]));
      }
    }
      break;

    case PRE_SYNCHRONIZING:
      if ((synchro_start_iter_--) <= 0) {
        state_ = SYNCHRONIZING;
        std::cout << getName() << " Synchronization started" << std::endl;
      }
      break;

    case SYNCHRONIZING:
      switch (synchro_state_) {
        case MOVE_TO_SYNCHRO_AREA:

          if (synchro_needed_[synchro_drive_]) {
            if (hi_->in_synchro_area(synchro_drive_)) {
              RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                   << " ] MOVE_TO_SYNCHRO_AREA ended"
                                   << RTT::endlog();

              synchro_state_ = STOP;
            } else {
              // ruszam powoli w stronę synchro area
              RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                   << " ] MOVE_TO_SYNCHRO_AREA"
                                   << RTT::endlog();
              desired_position_[synchro_drive_] +=
                  synchro_step_coarse_[synchro_drive_];
            }
          } else {
            hi_->set_parameter_now(synchro_drive_, NF_COMMAND_SetDrivesMisc,
            NF_DrivesMisc_SetSynchronized);
            hi_->reset_position(synchro_drive_);
            if (++synchro_drive_ == number_of_drives_) {
              synchro_state_ = SYNCHRO_END;
            } else {
              synchro_state_ = MOVE_TO_SYNCHRO_AREA;
            }
          }
          break;

        case STOP:
          hi_->start_synchro(synchro_drive_);

          synchro_state_ = MOVE_FROM_SYNCHRO_AREA;

          break;

        case MOVE_FROM_SYNCHRO_AREA:
          if (!hi_->in_synchro_area(synchro_drive_)) {
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] MOVE_FROM_SYNCHRO_AREA ended"
                                 << RTT::endlog();

            synchro_state_ = WAIT_FOR_IMPULSE;
          } else {
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] MOVE_FROM_SYNCHRO_AREA"
                                 << RTT::endlog();
            desired_position_[synchro_drive_] +=
                synchro_step_fine_[synchro_drive_];
          }
          break;

        case WAIT_FOR_IMPULSE:
          if (hi_->drive_synchronized(synchro_drive_)) {
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] WAIT_FOR_IMPULSE ended"
                                 << RTT::endlog();

            hi_->finish_synchro(synchro_drive_);
            hi_->reset_position(synchro_drive_);

            if (++synchro_drive_ == number_of_drives_) {
              synchro_state_ = SYNCHRO_END;
            } else {
              synchro_state_ = MOVE_TO_SYNCHRO_AREA;
            }

          } else {
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] WAIT_FOR_IMPULSE" << RTT::endlog();
            desired_position_[synchro_drive_] +=
                synchro_step_fine_[synchro_drive_];
          }
          break;

        case SYNCHRO_END:

          if ((synchro_stop_iter_--) == 1) {
            for (int i = 0; i < number_of_drives_; i++) {
              motor_position_command_(i) = motor_position_(i);
              desired_position_[i] = motor_position_(i);
            }

            port_is_synchronised_.write(true);
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] SYNCHRONIZING ended" << RTT::endlog();
            std::cout << getName() << " synchro finished" << std::endl;
          } else if (synchro_stop_iter_ <= 0) {
            state_ = PRE_SERVOING;
          }
          break;
      }
      break;
  }

  for (int i = 0; i < number_of_drives_; i++) {
    if (state_ != SERVOING) {
      desired_position_out_list_[i]->write(
          static_cast<double>(desired_position_[i]));
    }
  }
}

ORO_CREATE_COMPONENT(HardwareInterfaceMW)
