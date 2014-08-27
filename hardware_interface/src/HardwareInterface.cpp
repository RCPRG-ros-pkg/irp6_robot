#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <rtt/extras/SlaveActivity.hpp>

#include <Eigen/Dense>

#include "HardwareInterface.h"

HardwareInterface::HardwareInterface(const std::string& name)
    : TaskContext(name, PreOperational),
      synchro_start_iter_(0),
      synchro_stop_iter_(0),
      test_mode_(0),
      counter_(0.0),
      hi_(NULL),
      state_(NOT_SYNCHRONIZED),
      synchro_drive_(0),
      tx_prefix_len_(0),
      synchro_state_(MOVE_TO_SYNCHRO_AREA),
      rwh_nsec_(1200000) {

  this->addProperty("number_of_drives", number_of_drives_).doc(
      "Number of drives in robot");
  this->addProperty("auto_synchronize", auto_synchronize_).doc("");
  this->addProperty("test_mode", test_mode_).doc("");
  this->addProperty("rwh_nsec", rwh_nsec_).doc("");
  this->addProperty("hi_port_param_0", hi_port_param_[0]).doc("");
  this->addProperty("hi_port_param_1", hi_port_param_[1]).doc("");
  this->addProperty("hi_port_param_2", hi_port_param_[2]).doc("");
  this->addProperty("hi_port_param_3", hi_port_param_[3]).doc("");
  this->addProperty("hi_port_param_4", hi_port_param_[4]).doc("");
  this->addProperty("hi_port_param_5", hi_port_param_[5]).doc("");
  this->addProperty("hi_port_param_6", hi_port_param_[6]).doc("");
  this->addProperty("hi_port_param_7", hi_port_param_[7]).doc("");
  this->addProperty("hi_port_param_8", hi_port_param_[8]).doc("");
  this->addProperty("hi_port_param_9", hi_port_param_[9]).doc("");
  this->addProperty("hi_port_param_10", hi_port_param_[10]).doc("");
  this->addProperty("hi_port_param_11", hi_port_param_[11]).doc("");
  this->addProperty("hi_port_param_12", hi_port_param_[12]).doc("");
  this->addProperty("hi_port_param_13", hi_port_param_[13]).doc("");
  this->addProperty("hi_port_param_14", hi_port_param_[14]).doc("");
  this->addProperty("hi_port_param_15", hi_port_param_[15]).doc("");
}

HardwareInterface::~HardwareInterface() {
}

bool HardwareInterface::configureHook() {
  // dynamic ports list initialization

  // std::cout << "SIZE: "  << hi_port_param_[0].label << "enc_res: " << hi_port_param_[0].enc_res << std::endl;

  computedReg_in_list_.resize(number_of_drives_);
  posInc_out_list_.resize(number_of_drives_);
  deltaInc_out_list_.resize(number_of_drives_);
  port_motor_position_command_list_.resize(number_of_drives_);
  port_motor_position_list_.resize(number_of_drives_);

  for (size_t i = 0; i < number_of_drives_; i++) {
    char computedReg_in_port_name[32];
    snprintf(computedReg_in_port_name, sizeof(computedReg_in_port_name),
             "computedReg_in_%s", hi_port_param_[i].label.c_str());
    computedReg_in_list_[i] = new typeof(*computedReg_in_list_[i]);
    this->ports()->addPort(computedReg_in_port_name, *computedReg_in_list_[i]);

    char posInc_out_port_name[32];
    snprintf(posInc_out_port_name, sizeof(posInc_out_port_name),
             "posInc_out_%s", hi_port_param_[i].label.c_str());
    posInc_out_list_[i] = new typeof(*posInc_out_list_[i]);
    this->ports()->addPort(posInc_out_port_name, *posInc_out_list_[i]);

    char deltaInc_out_port_name[32];
    snprintf(deltaInc_out_port_name, sizeof(deltaInc_out_port_name),
             "deltaInc_out_%s", hi_port_param_[i].label.c_str());
    deltaInc_out_list_[i] = new typeof(*deltaInc_out_list_[i]);
    this->ports()->addPort(deltaInc_out_port_name, *deltaInc_out_list_[i]);

    char MotorPositionCommand_port_name[32];
    snprintf(MotorPositionCommand_port_name,
             sizeof(MotorPositionCommand_port_name), "MotorPositionCommand_%s",
             hi_port_param_[i].label.c_str());
    port_motor_position_command_list_[i] =
        new typeof(*port_motor_position_command_list_[i]);
    this->ports()->addPort(MotorPositionCommand_port_name,
                           *port_motor_position_command_list_[i]);

    char MotorPosition_port_name[32];
    snprintf(MotorPosition_port_name, sizeof(MotorPosition_port_name),
             "MotorPosition_%s", hi_port_param_[i].label.c_str());
    port_motor_position_list_[i] = new typeof(*port_motor_position_list_[i]);
    this->ports()->addPort(MotorPosition_port_name,
                           *port_motor_position_list_[i]);

  }

  // properties copying to internal buffers

  ports_adresses_.resize(number_of_drives_);
  max_current_.resize(number_of_drives_);
  max_increment_.resize(number_of_drives_);
  card_indexes_.resize(number_of_drives_);
  enc_res_.resize(number_of_drives_);
  synchro_step_coarse_.resize(number_of_drives_);
  synchro_step_fine_.resize(number_of_drives_);
  current_mode_.resize(number_of_drives_);
  synchro_needed_.resize(number_of_drives_);

//  std::cout << "HI PARAMS:"  << std::endl;

  for (size_t i = 0; i < number_of_drives_; i++) {
    //   std::cout << "i: "  << i << " label: " << hi_port_param_[i].label << std::endl;
    ports_adresses_[i] = hi_port_param_[i].ports_adresses;
    //   std::cout << "ports_adresses_: "  << ports_adresses_[i] << std::endl;
    max_current_[i] = hi_port_param_[i].max_current;
    //   std::cout << "max_current_: "  << max_current_[i] << std::endl;
    max_increment_[i] = hi_port_param_[i].max_increment;
//    std::cout << "max_increment_: "  << max_increment_[i] << std::endl;
    card_indexes_[i] = hi_port_param_[i].card_indexes;
    //   std::cout << "ports_adresses_: "  << ports_adresses_[i] << std::endl;
    enc_res_[i] = hi_port_param_[i].enc_res;
    //  std::cout << "card_indexes_: "  << card_indexes_[i] << std::endl;
    synchro_step_coarse_[i] = hi_port_param_[i].synchro_step_coarse;
    //  std::cout << "synchro_step_coarse_: "  << synchro_step_coarse_[i] << std::endl;
    synchro_step_fine_[i] = hi_port_param_[i].synchro_step_fine;
    //  std::cout << "synchro_step_fine_: "  << synchro_step_fine_[i] << std::endl;
    current_mode_[i] = hi_port_param_[i].current_mode;
    //   std::cout << "current_mode_: "  << current_mode_[i] << std::endl;
    synchro_needed_[i] = hi_port_param_[i].synchro_needed;
    //   std::cout << "synchro_needed_: "  << synchro_needed_[i] << std::endl << std::endl;
  }

  if (!test_mode_) {
    hi_ = new hi_moxa::HI_moxa(number_of_drives_ - 1, card_indexes_,
                               max_increment_, tx_prefix_len_);
  }
  counter_ = 0.0;

  increment_.resize(number_of_drives_);
  pos_inc_.resize(number_of_drives_);
  pwm_or_current_.resize(number_of_drives_);

  for (int i = 0; i < number_of_drives_; i++) {
    increment_[i] = 0;
    pwm_or_current_[0] = 0;
  }

  try {
    struct timespec delay;
    if (!test_mode_) {
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
    }
  } catch (std::exception& e) {
    log(Info) << e.what() << endlog();
    return false;
  }

  motor_position_.resize(number_of_drives_);
  motor_position_command_.resize(number_of_drives_);
  motor_position_command_old_.resize(number_of_drives_);

  PeerList plist;

  plist = this->getPeerList();

  for (size_t i = 0; i < plist.size(); i++) {
    std::cout << plist[i] << std::endl;
    servo_list_.push_back(this->getPeer(plist[i]));
    servo_list_[i]->setActivity(
        new RTT::extras::SlaveActivity(this->getActivity(),
                                       servo_list_[i]->engine()));
  }

  return true;
}

uint16_t HardwareInterface::convert_to_115(float input) {
  uint16_t output;

  if (input >= 1.0) {
    printf("convert_to_115 input bigger or equal then 1.0\n");
    return 0;
  } else if (input < -1.0) {
    printf("convert_to_115 input lower then -1.0\n");
    return 0;
  } else if (input < 0.0) {
    output = 65535 + (int) (input * 32768.0);
  } else if (input >= 0.0) {
    output = (uint16_t) (input * 32768.0);
  }

//  printf("convert_to_115 i: %f, o: %x\n",input, output);

  return output;
}

void HardwareInterface::test_mode_sleep() {
  struct timespec delay;
  delay.tv_nsec = rwh_nsec_ + 200000;
  delay.tv_sec = 0;

  nanosleep(&delay, NULL);
}

bool HardwareInterface::startHook() {
  try {
    if (!test_mode_) {
      hi_->write_read_hardware(rwh_nsec_);

      if (!hi_->robot_synchronized()) {
        RTT::log(RTT::Info) << "Robot not synchronized" << RTT::endlog();
        if (auto_synchronize_) {
          RTT::log(RTT::Info) << "Auto synchronize" << RTT::endlog();
          state_ = SERVOING;
          synchro_start_iter_ = 500;
          synchro_stop_iter_ = 1000;
          synchro_state_ = MOVE_TO_SYNCHRO_AREA;
          synchro_drive_ = 0;
          std::cout << "Auto synchronize" << std::endl;
          state_ = PRE_SYNCHRONIZING;

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
    } else {
      test_mode_sleep();
      RTT::log(RTT::Info) << "HI test mode activated" << RTT::endlog();

      for (int i = 0; i < number_of_drives_; i++) {
        motor_position_command_(i) = 0.0;
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
    if (!test_mode_) {
      motor_position_(i) = (double) hi_->get_position(i)
          * ((2.0 * M_PI) / enc_res_[i]);
    } else {
      motor_position_(i) = 0.0;
    }

    port_motor_position_list_[i]->write(motor_position_[i]);
  }

  return true;
}

void HardwareInterface::updateHook() {

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

      break;

    case PRE_SYNCHRONIZING:
      for (int i = 0; i < number_of_drives_; i++) {
        pos_inc_[i] = 0.0;
      }

      if ((synchro_start_iter_--) <= 0) {
        state_ = SYNCHRONIZING;
        std::cout << "Synchronization started" << std::endl;
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
              pos_inc_[synchro_drive_] = 0.0;
              synchro_state_ = STOP;
            } else {
              // ruszam powoli w stronę synchro area
              RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                   << " ] MOVE_TO_SYNCHRO_AREA"
                                   << RTT::endlog();
              pos_inc_[synchro_drive_] = synchro_step_coarse_[synchro_drive_]
                  * (enc_res_[synchro_drive_] / (2.0 * M_PI));
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

            if (++synchro_drive_ == number_of_drives_) {
              synchro_state_ = SYNCHRO_END;
            } else {
              synchro_state_ = MOVE_TO_SYNCHRO_AREA;
            }

          } else {
            RTT::log(RTT::Debug) << "[servo " << synchro_drive_
                                 << " ] WAIT_FOR_IMPULSE" << RTT::endlog();
            pos_inc_[synchro_drive_] = synchro_step_fine_[synchro_drive_]
                * (enc_res_[synchro_drive_] / (2.0 * M_PI));
          }
          break;

        case SYNCHRO_END:

          if ((synchro_stop_iter_--) <= 0) {

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

  if (!test_mode_) {

    for (int i = 0; i < number_of_drives_; i++) {
      increment_[i] = hi_->get_increment(i);

      if (abs(increment_[i]) > 400) {
        increment_[i] = 0;
        std::cout << "very high increment_" << std::endl;
      }

      if (fabs(pos_inc_[i]) > 400) {
        std::cout << "very high pos_inc_ i: " << i << " pos_inc: "
                  << pos_inc_[i] << std::endl;
       // pos_inc_[i] = 0;
      }
    }

    for (int i = 0; i < number_of_drives_; i++) {
      deltaInc_out_list_[i]->write(increment_[i]);
      posInc_out_list_[i]->write(pos_inc_[i]);

    }

    for (size_t i = 0; i < servo_list_.size(); i++) {
      //std::cout << "servo update" << std::endl;
      servo_list_[i]->update();
    }

    for (int i = 0; i < number_of_drives_; i++) {
      if (NewData != computedReg_in_list_[i]->read(pwm_or_current_[i])) {
        RTT::log(RTT::Error) << "NO PWM DATA" << RTT::endlog();
      }
      if (current_mode_[i]) {

        hi_->set_current(i, pwm_or_current_[i]);
      } else {
        hi_->set_pwm(i, pwm_or_current_[i]);
      }
    }

    // std::cout << "aaaa: " << pwm_or_current_[0] << std::endl;

    hi_->write_read_hardware(rwh_nsec_);

    if (state_ == SERVOING) {

      for (int i = 0; i < number_of_drives_; i++) {
        motor_position_(i) = (double) hi_->get_position(i)
            * ((2.0 * M_PI) / enc_res_[i]);

        port_motor_position_list_[i]->write(motor_position_[i]);
      }
    }
  } else {
    test_mode_sleep();
    for (int i = 0; i < number_of_drives_; i++) {
      motor_position_(i) = motor_position_command_(i);
      port_motor_position_list_[i]->write(motor_position_[i]);
    }

  }

}

ORO_CREATE_COMPONENT(HardwareInterface)
