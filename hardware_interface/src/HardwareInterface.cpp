#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <Eigen/Dense>

#include "HardwareInterface.h"

HardwareInterface::HardwareInterface(const std::string& name)
    : TaskContext(name, PreOperational),
      servo_stop_iter_(0) {

  this->addProperty("number_of_drives", number_of_drives_).doc(
      "Number of drives in robot");
  this->addProperty("auto_synchronize", auto_synchronize_).doc("");
  this->addProperty("ports_adresses", ports_adresses_).doc("");
  this->addProperty("card_indexes", card_indexes_).doc("");
  this->addProperty("max_increment", max_increment_).doc("");
  this->addProperty("max_current", max_current_).doc("");
  this->addProperty("tx_prefix_len", tx_prefix_len_).doc("");
  this->addProperty("enc_res", enc_res_).doc("");
  this->addProperty("synchro_step_coarse", synchro_step_coarse_).doc("");
  this->addProperty("synchro_step_fine", synchro_step_fine_).doc("");
  this->addProperty("current_mode", current_mode_).doc("");
}

HardwareInterface::~HardwareInterface() {
}

bool HardwareInterface::configureHook() {
  if (ports_adresses_.size() != number_of_drives_
      || max_increment_.size() != number_of_drives_
      || max_current_.size() != number_of_drives_
      || enc_res_.size() != number_of_drives_
      || synchro_step_coarse_.size() != number_of_drives_
      || synchro_step_fine_.size() != number_of_drives_
      || card_indexes_.size() != number_of_drives_
      || current_mode_.size() != number_of_drives_) {
    log(Error) << "Size of parameters is different than number of drives"
               << endlog();
    return false;
  }

  // dynamic ports list initialization

  computedReg_in_list_.resize(number_of_drives_);
  posInc_out_list_.resize(number_of_drives_);
  deltaInc_out_list_.resize(number_of_drives_);
  port_motor_position_command_list_.resize(number_of_drives_);
  port_motor_position_list_.resize(number_of_drives_);

  for (size_t i = 0; i < number_of_drives_; i++) {
    char computedReg_in_port_name[32];
    snprintf(computedReg_in_port_name, sizeof(computedReg_in_port_name),
             "computedReg_in_%zu", i);
    computedReg_in_list_[i] = new typeof(*computedReg_in_list_[i]);
    this->ports()->addPort(computedReg_in_port_name, *computedReg_in_list_[i]);

    char posInc_out_port_name[32];
    snprintf(posInc_out_port_name, sizeof(posInc_out_port_name),
             "posInc_out%zu", i);
    posInc_out_list_[i] = new typeof(*posInc_out_list_[i]);
    this->ports()->addPort(posInc_out_port_name, *posInc_out_list_[i]);

    char deltaInc_out_port_name[32];
    snprintf(deltaInc_out_port_name, sizeof(deltaInc_out_port_name),
             "deltaInc_out%zu", i);
    deltaInc_out_list_[i] = new typeof(*deltaInc_out_list_[i]);
    this->ports()->addPort(deltaInc_out_port_name, *deltaInc_out_list_[i]);

    char MotorPositionCommand_port_name[32];
    snprintf(MotorPositionCommand_port_name,
             sizeof(MotorPositionCommand_port_name), "MotorPositionCommand_%zu",
             i);
    port_motor_position_command_list_[i] =
        new typeof(*port_motor_position_command_list_[i]);
    this->ports()->addPort(MotorPositionCommand_port_name,
                           *port_motor_position_command_list_[i]);

    char MotorPosition_port_name[32];
    snprintf(MotorPosition_port_name, sizeof(MotorPosition_port_name),
             "MotorPosition_%zu", i);
    port_motor_position_list_[i] = new typeof(*port_motor_position_list_[i]);
    this->ports()->addPort(MotorPosition_port_name,
                           *port_motor_position_list_[i]);

  }

  hi_ = new hi_moxa::HI_moxa(number_of_drives_ - 1, card_indexes_,
                             max_increment_, tx_prefix_len_),

  counter_ = 0.0;
  auto_synchronize_ = true;

  increment_.resize(number_of_drives_);
  pos_inc_.resize(number_of_drives_);
  pwm_.resize(number_of_drives_);

  for (int i = 0; i < number_of_drives_; i++) {
    increment_[i] = 0;
    pwm_[0] = 0;
  }

  try {
    hi_->init(ports_adresses_);
    for (int i = 0; i < number_of_drives_; i++) {
      hi_->set_parameter_now(i, NF_COMMAND_SetDrivesMaxCurrent,
                             max_current_[i]);
      if (current_mode_[i]) {
        hi_->set_current_mode(i);
      } else {
        hi_->set_pwm_mode(i);

      }
    }
  } catch (std::exception& e) {
    log(Info) << e.what() << endlog();
    return false;
  }

  motor_position_.resize(number_of_drives_);
  motor_position_command_.resize(number_of_drives_);
  motor_position_command_old_.resize(number_of_drives_);

  return true;
}

bool HardwareInterface::startHook() {
  try {
    hi_->HI_read_write_hardware();

    if (!hi_->robot_synchronized()) {
      RTT::log(RTT::Info) << "Robot not synchronized" << RTT::endlog();
      if (auto_synchronize_) {
        RTT::log(RTT::Info) << "Auto synchronize" << RTT::endlog();
        state_ = SYNCHRONIZING;
        synchro_state_ = MOVE_TO_SYNCHRO_AREA;
        synchro_drive_ = 0;
      } else
        state_ = NOT_SYNCHRONIZED;
    } else {
      RTT::log(RTT::Info) << "Robot synchronized" << RTT::endlog();

      for (int i = 0; i < number_of_drives_; i++) {
        motor_position_command_(i) = (double) hi_->get_position(i)
            * ((2.0 * M_PI) / enc_res_[i]);
        motor_position_command_old_(i) = motor_position_command_(i);
      }

      state_ = SERVOING;
    }
  } catch (const std::exception& e) {
    RTT::log(RTT::Error) << e.what() << RTT::endlog();
    return false;
  }

  for (int i = 0; i < number_of_drives_; i++) {
    pos_inc_[i] = 0.0;
  }

  return true;
}

void HardwareInterface::updateHook() {

  for (int i = 0; i < number_of_drives_; i++) {
    if (NewData != computedReg_in_list_[i]->read(pwm_[i])) {
      RTT::log(RTT::Error) << "NO PWM DATA" << RTT::endlog();
    }
    if (current_mode_[i]) {
      hi_->set_current(i, pwm_[i]);
    } else {
      hi_->set_pwm(i, pwm_[i]);
    }

  }

  hi_->HI_read_write_hardware();

  switch (state_) {
    case NOT_SYNCHRONIZED:

      for (int i = 0; i < number_of_drives_; i++) {
        pos_inc_[i] = 0.0;
      }
      break;

    case SERVOING:

      for (int i = 0; i < number_of_drives_; i++) {
        if (port_motor_position_command_list_[i]->read(
            motor_position_command_[i]) == RTT::NewData) {
          pos_inc_[i] = (motor_position_command_(i)
              - motor_position_command_old_(i)) * (enc_res_[i] / (2.0 * M_PI));
          motor_position_command_old_(i) = motor_position_command_(i);

        } else {
          pos_inc_[i] = 0.0;
        }
      }

      for (int i = 0; i < number_of_drives_; i++) {
        motor_position_(i) = (double) hi_->get_position(i)
            * ((2.0 * M_PI) / enc_res_[i]);
      }
      // port_motor_position_.write(motor_position_);

      for (int i = 0; i < number_of_drives_; i++) {
        port_motor_position_list_[i]->write(motor_position_[i]);
      }

      break;

    case SYNCHRONIZING:
      switch (synchro_state_) {
        case MOVE_TO_SYNCHRO_AREA:
          servo_stop_iter_ = 1000;
          if (hi_->in_synchro_area(synchro_drive_)) {
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] MOVE_TO_SYNCHRO_AREA ended"
                                 << RTT::endlog();
            pos_inc_[synchro_drive_] = 0.0;
            synchro_state_ = STOP;
          } else {
            // ruszam powoli w stronę synchro area
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] MOVE_TO_SYNCHRO_AREA" << RTT::endlog();
            pos_inc_[synchro_drive_] = synchro_step_coarse_[synchro_drive_]
                * (enc_res_[synchro_drive_] / (2.0 * M_PI));
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
            pos_inc_[synchro_drive_] = synchro_step_fine_[synchro_drive_]
                * (enc_res_[synchro_drive_] / (2.0 * M_PI));
          }
          break;

        case WAIT_FOR_IMPULSE:
          if (hi_->drive_synchronized(synchro_drive_)) {
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] WAIT_FOR_IMPULSE ended"
                                 << RTT::endlog();

            for (int i = 0; i < number_of_drives_; i++) {
              pos_inc_[i] = 0.0;
            }

            hi_->finish_synchro(synchro_drive_);
            hi_->reset_position(synchro_drive_);

            if (++synchro_drive_ < number_of_drives_) {
              synchro_state_ = MOVE_TO_SYNCHRO_AREA;
            } else {
              synchro_state_ = SYNCHRO_END;
            }

          } else {
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] WAIT_FOR_IMPULSE" << RTT::endlog();
            pos_inc_[synchro_drive_] = synchro_step_fine_[synchro_drive_]
                * (enc_res_[synchro_drive_] / (2.0 * M_PI));
          }
          break;

        case SYNCHRO_END:

          if ((servo_stop_iter_--) <= 0) {

            for (int i = 0; i < number_of_drives_; i++) {
              motor_position_command_(i) = motor_position_command_old_(i) = hi_
                  ->get_position(i) * (2.0 * M_PI) / enc_res_[i];
            }

            state_ = SERVOING;
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] SYNCHRONIZING ended" << RTT::endlog();
            std::cout << "synchro finished" << std::endl;
          }
          break;
      }
      break;
  }

  for (int i = 0; i < number_of_drives_; i++) {
    increment_[i] = hi_->get_increment(i);

    if (abs(increment_[i]) > 400) {
      increment_[i] = 0;
      std::cout << "very high increment_" << std::endl;
    }

    if (fabs(pos_inc_[i]) > 400) {
      std::cout << "very high pos_inc_ i:" << i << "pos_inc: " << pos_inc_[i]
                << std::endl;
      pos_inc_[i] = 0;
    }
  }

  for (int i = 0; i < number_of_drives_; i++) {
    hi_->set_pwm(i, pwm_[i]);
    deltaInc_out_list_[i]->write(increment_[i]);
    posInc_out_list_[i]->write(pos_inc_[i]);
  }

}

ORO_CREATE_COMPONENT(HardwareInterface)
