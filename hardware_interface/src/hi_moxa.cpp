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

#include "hi_moxa.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>

#include <exception>
#include <stdexcept>
#include <cstring>

#include <iostream> // NOLINT
#include <cstdarg>

#include <vector>
#include <string>

namespace hi_moxa {

HI_moxa::HI_moxa(unsigned int numberOfDrivers,
                 std::vector<unsigned int> card_addresses,
                 std::vector<double> max_increments, int tx_prefix_len,
                 std::string hardware_name_)
    : howMuchItSucks(tx_prefix_len),
      last_drive_number(numberOfDrivers),
      drives_addresses(card_addresses),
      ridiculous_increment(max_increments),
      hardware_panic(false),
      all_hardware_read(true),
      longest_delay_(0),
      longest_read_delay_(0),
      cycle_nr(0),
      error_msg_hardware_panic_(0),
      hardware_name(hardware_name_) {
  for (unsigned int drive_number = 0; drive_number <= last_drive_number;
      drive_number++) {
    memset(servo_data + drive_number, 0, sizeof(servo_St));
    memset(oldtio + drive_number, 0, sizeof(termios));
    drive_buff[drive_number].rxCnt = 0;
    receiveFail[drive_number] = false;
    receiveFailCnt[drive_number] = 0;
    SerialPort[drive_number] = NULL;
    // clear_buffer(drive_number);
  }
  memset(&NFComBuf, 0, sizeof(NF_STRUCT_ComBuf));
  memset(txBuf, 0, BUFF_SIZE);
  txCnt = 0;
// memset(rxBuf, 0, BUFF_SIZE);

  memset(rxCommandArray, 0, BUFF_SIZE);
  rxCommandCnt = 0;
}

HI_moxa::~HI_moxa() {
}

void HI_moxa::init(std::vector<std::string> ports) {
  port_names = ports;

// inicjalizacja crcTable[]
  NFv2_CrcInit();

// zerowanie danych i ustawienie neutralnych adresow
  NF_ComBufReset(&NFComBuf);

// inicjalizacja zmiennych
  NFComBuf.myAddress = NF_MasterAddress;

  for (unsigned int drive_number = 0; drive_number <= last_drive_number;
      drive_number++) {
    NFComBuf.ReadDeviceStatus.addr[drive_number] = 255;
    NFComBuf.ReadDeviceVitals.addr[drive_number] = 255;
    NFComBuf.SetDrivesMode.addr[drive_number] = 255;
    NFComBuf.SetDrivesPWM.addr[drive_number] = 255;
    NFComBuf.SetDrivesCurrent.addr[drive_number] = 255;
    NFComBuf.SetDrivesMaxCurrent.addr[drive_number] = 255;
    NFComBuf.ReadDrivesCurrent.addr[drive_number] = 255;
    NFComBuf.ReadDrivesPosition.addr[drive_number] = 255;
    NFComBuf.SetDrivesMisc.addr[drive_number] = 255;
    NFComBuf.ReadDrivesStatus.addr[drive_number] = 255;
    NFComBuf.SetCurrentRegulator.addr[drive_number] = 255;
  }

  for (unsigned int drive_number = 0; drive_number <= last_drive_number;
      drive_number++) {
    NFComBuf.ReadDeviceStatus.addr[drive_number] =
        drives_addresses[drive_number];
    NFComBuf.ReadDeviceVitals.addr[drive_number] =
        drives_addresses[drive_number];
    NFComBuf.SetDrivesMode.addr[drive_number] = drives_addresses[drive_number];
    NFComBuf.SetDrivesPWM.addr[drive_number] = drives_addresses[drive_number];
    NFComBuf.SetDrivesCurrent.addr[drive_number] =
        drives_addresses[drive_number];
    NFComBuf.SetDrivesMaxCurrent.addr[drive_number] =
        drives_addresses[drive_number];
    NFComBuf.ReadDrivesCurrent.addr[drive_number] =
        drives_addresses[drive_number];
    NFComBuf.ReadDrivesPosition.addr[drive_number] =
        drives_addresses[drive_number];
    NFComBuf.SetDrivesMisc.addr[drive_number] = drives_addresses[drive_number];
    NFComBuf.ReadDrivesStatus.addr[drive_number] =
        drives_addresses[drive_number];
    NFComBuf.SetCurrentRegulator.addr[drive_number] =
        drives_addresses[drive_number];

    NFComBuf.ReadDeviceStatus.data[drive_number] = 0;
    NFComBuf.ReadDeviceVitals.data[drive_number] = 0;
    NFComBuf.SetDrivesMode.data[drive_number] = 0;
    NFComBuf.SetDrivesPWM.data[drive_number] = 0;
    NFComBuf.SetDrivesCurrent.data[drive_number] = 0;
    NFComBuf.SetDrivesMaxCurrent.data[drive_number] = 0;
    NFComBuf.ReadDrivesCurrent.data[drive_number] = 0;
    NFComBuf.ReadDrivesPosition.data[drive_number] = 0;
    NFComBuf.SetDrivesMisc.data[drive_number] = 0;
    NFComBuf.ReadDrivesStatus.data[drive_number] = 0;

    servo_data[drive_number].first_hardware_reads =
        FIRST_HARDWARE_READS_WITH_ZERO_INCREMENT;
  }

  for (unsigned int drive_number = 0; drive_number <= last_drive_number;
      drive_number++) {
    SerialPort[drive_number] = new SerialComm(port_names[drive_number].c_str(),
                                              BAUD);
    if (SerialPort[drive_number]->isConnected()) {
      std::cout << "[info] Połączono (port [" << drive_number << "]: "
                << port_names[drive_number] << ")" << std::endl;
    } else {
      std::cout << std::endl << "[error] Nie wykryto sprzetu! (port ["
                << drive_number << "]: " << port_names[drive_number] << ")"
                << std::endl;
      throw(std::runtime_error("unable to open device!!!"));
    }
    // start driver in MANUAL mode
    set_parameter_now(drive_number, NF_COMMAND_SetDrivesMode,
    NF_DrivesMode_MANUAL);

    receiveFailCnt[drive_number] = 0;
  }
// maxReceiveFailCnt = MAX_RECEIVE_FAIL_CNT;
  reset_counters();
}

void HI_moxa::reset_counters(void) {
  for (int i = 0; i <= last_drive_number; i++) {
    servo_data[i].current_absolute_position = 0L;
    servo_data[i].previous_absolute_position = 0L;
    servo_data[i].current_position_inc = 0.0;
    servo_data[i].previous_position_inc = 0.0;
    check_ridicolous_increment_[i] = true;
  }
}

int HI_moxa::get_current(int drive_number) {
  int ret;
  ret = NFComBuf.ReadDrivesCurrent.data[drive_number];
  return ret;
}

float HI_moxa::get_voltage(int drive_number) {
  float ret = VOLTAGE;
  return ret;
}

double HI_moxa::get_increment(int drive_number) {
  double ret;
  ret = servo_data[drive_number].current_position_inc;
  return ret;
}

int64_t HI_moxa::get_position(int drive_number) {
  int ret;
  ret = servo_data[drive_number].current_absolute_position;
  return ret;
}

void HI_moxa::set_pwm_mode(int drive_number) {
  set_parameter_now(drive_number, NF_COMMAND_SetDrivesMode, NF_DrivesMode_PWM);
}

void HI_moxa::set_current_mode(int drive_number) {
  set_parameter_now(drive_number, NF_COMMAND_SetDrivesMode,
  NF_DrivesMode_CURRENT);
}

void HI_moxa::set_pwm(int drive_number, double set_value) {
// sklaowanie w celu dostosowania do starych regulatorow pozycyjnych
  NFComBuf.SetDrivesPWM.data[drive_number] = set_value * (1000.0 / 255.0);
  servo_data[drive_number].commandArray[servo_data[drive_number].commandCnt++] =
  NF_COMMAND_SetDrivesPWM;
}

void HI_moxa::set_current(int drive_number, double set_value) {
  NFComBuf.SetDrivesCurrent.data[drive_number] = static_cast<int>(set_value);
  servo_data[drive_number].commandArray[servo_data[drive_number].commandCnt++] =
  NF_COMMAND_SetDrivesCurrent;
}

uint64_t HI_moxa::read_hardware(int timeouts_to_print) {
  cycle_nr++;

// static int accel_limit[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static int valid_msr_nr[] = { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
      10, 10, 10, 10 };

  static int64_t receive_attempts = 0;
// UNUSED: static int64_t receive_timeouts = 0;
  static int error_msg_power_stage = 0;

  static int error_msg_overcurrent = 0;
  static int error_limit_switch = 0;
  static int last_synchro_state[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0 };
  static int comm_timeouts[] =
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  static int synchro_switch_filter[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0 };
  const int synchro_switch_filter_th = 2;
  bool robot_synchronized = true;
  bool power_fault;
  uint64_t ret = 0;
  uint8_t drive_number;
  static int status_disp_cnt = 0;
  static std::stringstream temp_message;

  receive_attempts++;

// Tu kiedys byl SELECT
  bool current_all_hardware_read = true;
  bool read_needed[MOXA_SERVOS_NR];

// Read data from all drives
  for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
    read_needed[drive_number] = (all_hardware_read
        || (!all_hardware_read && receiveFail[drive_number]));

    if (read_needed[drive_number]) {
      //  rxCnt = 0;
//    while (1) {
//      if (SerialPort[drive_number]->read(&(rxBuf[rxCnt]), 1) > 0
//          && (rxCnt < 255)) {
//        if (NF_Interpreter(&NFComBuf, rxBuf, &rxCnt, rxCommandArray,
//                           &rxCommandCnt) > 0) {
//          // TODO: Check Status
//          break;
//        }
//      } else {
//        comm_timeouts[drive_number]++;
//        if (all_hardware_read) {
//          all_hardware_read = false;
//          std::cout << "[error] timeout in " << (int) receive_attempts
//                    << " communication cycle on drives";
//        }
//        std::cout << " " << (int) drive_number << "("
//                  << port_names[drive_number].c_str() << ")";
//        break;
//      }
//    }

      int bytes_received = 0;
      uint8_t receive_buffer[255];
      int receive_success = 0;

      bytes_received = SerialPort[drive_number]->read(receive_buffer, 255);

      receiveFailCnt[drive_number]++;
      receiveFail[drive_number] = true;
      for (int i = 0; i < bytes_received; i++) {
        drive_buff[drive_number].rxBuf[drive_buff[drive_number].rxCnt] =
            receive_buffer[i];
        if (NF_Interpreter(&NFComBuf, drive_buff[drive_number].rxBuf,
                           &drive_buff[drive_number].rxCnt, rxCommandArray,
                           &rxCommandCnt) > 0) {
          drive_buff[drive_number].rxCnt = 0;
          receive_success = 1;
          receiveFailCnt[drive_number] = 0;
          receiveFail[drive_number] = false;
          valid_msr_nr[drive_number]++;
          break;
        }
        if (drive_buff[drive_number].rxCnt == 255) {
          drive_buff[drive_number].rxCnt = 0;
        }
      }
      if (receiveFailCnt[drive_number]) {
        /*
         if (receiveFailCnt[drive_number] > maxReceiveFailCnt) {
         drive_buff[drive_number].rxCnt = 0;
         receiveFailCnt[drive_number] = 0;
         std::cout << "[warn] extra receive time: drive " << (int) drive_number
         << " counter reset " << "bytes_received: " << bytes_received
         << std::endl;

         } else {
         */

        valid_msr_nr[drive_number] = 0;
        if (static_cast<int>(receiveFailCnt[drive_number])
            > timeouts_to_print) {
          std::cout << "[warn] extra receive time: drive "
                    << static_cast<int>(drive_number) << " event "
                    << static_cast<int>(receiveFailCnt[drive_number])
                    << " bytes_received: " << bytes_received << " cycle: "
                    << cycle_nr << std::endl;
        }
        // }
      }

      if (receive_success) {
      } else {
        comm_timeouts[drive_number]++;
        if (current_all_hardware_read) {
          current_all_hardware_read = false;
          //   std::cout << "[error] timeout in " << (int) receive_attempts << " communication cycle on drives";
        }
        // std::cout << " " << (int) drive_number << "(" << port_names[drive_number].c_str() << ")";
        // break;
      }
    }
  }
  all_hardware_read = current_all_hardware_read;
// If Hardware Panic, after receiving data, wait till the end of comm cycle and return.
  if (hardware_panic) {
    /*
     struct timespec delay;
     delay.tv_nsec = 2000000;
     delay.tv_sec = 0;

     nanosleep(&delay, NULL);
     */
    return ret;
  }

  if (all_hardware_read) {
    for (drive_number = 0; drive_number <= last_drive_number; drive_number++)
      comm_timeouts[drive_number] = 0;
  } else {
    // std::cout << std::endl;
  }

// Inicjalizacja flag
  robot_synchronized = true;
  power_fault = false;

  for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
    servo_data[drive_number].previous_absolute_position =
        servo_data[drive_number].current_absolute_position;

    servo_data[drive_number].previous_position_inc = servo_data[drive_number]
        .current_position_inc;

    // Wykrywanie sekwencji timeoutow komunikacji
    if (comm_timeouts[drive_number] >= MAX_COMM_TIMEOUTS) {
      hardware_panic = true;
      temp_message << "[error] multiple communication timeouts on drive "
                   << static_cast<int>(drive_number) << "("
                   << port_names[drive_number].c_str() << "): limit = "
                   << MAX_COMM_TIMEOUTS << std::endl;
      // master.msg->message(lib::FATAL_ERROR, temp_message.str());
      std::cerr << temp_message.str() << std::cerr.flush();
    }

    if (read_needed[drive_number] && !receiveFail[drive_number]) {
      // Wypelnienie pol odebranymi danymi
      // NFComBuf.ReadDrivesPosition.data[] contains last received value

      // Ustawienie flagi wlaczonej mocy
      if ((NFComBuf.ReadDrivesStatus.data[drive_number]
          & NF_DrivesStatus_PowerStageFault) != 0) {
        power_fault = true;
      }

      // Ustawienie flagi synchronizacji
      if ((NFComBuf.ReadDrivesStatus.data[drive_number]
          & NF_DrivesStatus_Synchronized) == 0) {
        robot_synchronized = false;
      }

      // Sprawdzenie, czy wlasnie nastapila synchronizacja kolejnej osi
      if (last_synchro_state[drive_number] == 0
          && (NFComBuf.ReadDrivesStatus.data[drive_number]
              & NF_DrivesStatus_Synchronized) != 0) {
        servo_data[drive_number].first_hardware_reads =
            FIRST_HARDWARE_READS_WITH_ZERO_INCREMENT;
        last_synchro_state[drive_number] = 1;
      }

      int32_t rdp = NFComBuf.ReadDrivesPosition.data[drive_number];

      // W pierwszych odczytach danych z napedu przyrost pozycji musi byc 0.
      if ((servo_data[drive_number].first_hardware_reads > 0)) {
        servo_data[drive_number].previous_absolute_position = rdp;
        servo_data[drive_number].first_hardware_reads--;
      }

      double cpi = static_cast<double>(rdp)
          - servo_data[drive_number].previous_absolute_position;

      // pierwszy pomiar jest spóźniony więc nadal interpolujemy
      if (valid_msr_nr[drive_number] < 2) {
        servo_data[drive_number].current_absolute_position =
            servo_data[drive_number].previous_absolute_position
                + servo_data[drive_number].current_position_inc;
      } else {
        servo_data[drive_number].current_position_inc = cpi;
        servo_data[drive_number].current_absolute_position = rdp;
      }

      if ((robot_synchronized && check_ridicolous_increment_[drive_number])
          && (static_cast<int>(ridiculous_increment[drive_number]) != 0)) {
        /*if (drive_number == 0) {

         std::cout << "inc: " << servo_data[drive_number].current_position_inc << " cur: "
         << servo_data[drive_number].current_absolute_position << " prev: "
         << servo_data[drive_number].previous_absolute_position << " time: " << boost::get_system_time()
         << std::endl;
         }*/

        if ((servo_data[drive_number].current_position_inc
            > ridiculous_increment[drive_number])
            || (servo_data[drive_number].current_position_inc
                < -ridiculous_increment[drive_number])) {
          hardware_panic = true;
          temp_message << std::endl << RED << hardware_name
                       << ": [error] RIDICOLOUS INCREMENT on drive "
                       << static_cast<int>(drive_number) << ", "
                       << port_names[drive_number].c_str() << ", c.cycle "
                       << static_cast<int>(receive_attempts) << ": read = "
                       << servo_data[drive_number].current_position_inc
                       << ", max = " << ridiculous_increment[drive_number]
                       << RESET << std::endl << std::endl;
          // master.msg->message(lib::FATAL_ERROR, temp_message.str());
          std::cerr << temp_message.str() << std::cerr.flush();
        }
      }

      // Sprawdzenie ograniczenia nadpradowego
      if ((NFComBuf.ReadDrivesStatus.data[drive_number]
          & NF_DrivesStatus_Overcurrent) != 0) {
        if (error_msg_overcurrent == 0) {
          // master.msg->message(lib::NON_FATAL_ERROR, "Overcurrent");
          std::cout << std::endl << RED << hardware_name
                    << ": [error] OVERCURRENT on drive "
                    << static_cast<int>(drive_number) << ", "
                    << port_names[drive_number].c_str() << ": read = "
                    << NFComBuf.ReadDrivesCurrent.data[drive_number] << "mA"
                    << RESET << std::endl << std::endl;
          error_msg_overcurrent++;
        }
      }

      // Sprawdzenie krancowek gorny limit
      if ((NFComBuf.ReadDrivesStatus.data[drive_number]
          & NF_DrivesStatus_LimitSwitchUp) != 0) {
        hardware_panic = true;
        if (error_limit_switch == 0) {
          // master.msg->message(lib::NON_FATAL_ERROR, "Overcurrent");
          std::cout << std::endl << RED << hardware_name
                    << ": [error] UPPER LIMIT SWITCH on drive "
                    << static_cast<int>(drive_number) << RESET << std::endl
                    << std::endl;
          error_limit_switch++;
        }
      }

      // Sprawdzenie krancowek dolny limit
      if ((NFComBuf.ReadDrivesStatus.data[drive_number]
          & NF_DrivesStatus_LimitSwitchDown) != 0) {
        hardware_panic = true;
        if (error_limit_switch == 0) {
          // master.msg->message(lib::NON_FATAL_ERROR, "Overcurrent");
          std::cout << std::endl << RED << hardware_name
                    << ": [error] LOWER LIMIT SWITCH on drive "
                    << static_cast<int>(drive_number) << RESET << std::endl
                    << std::endl;
          error_limit_switch++;
        }
      }

    } else {
      // jak nie odbierzemy biezacej pozycji to zakladamy, ze robot porusza sie ze stala predkoscia
      // wygladza to trajektorie w sytuacji zaklocen w komunikacji z kartami,
      // gdyz wczesniej zakladala sie wowczas bezruch

      servo_data[drive_number].current_absolute_position =
          servo_data[drive_number].previous_absolute_position
              + servo_data[drive_number].current_position_inc;
      valid_msr_nr[drive_number] = 0;
    }
  }

// master.controller_state_edp_buf.is_synchronised = robot_synchronized;
// master.controller_state_edp_buf.robot_in_fault_state = power_fault;
  if (power_fault) {
    hardware_panic = true;
    // std::cout << "[error] power_fault" << std::endl;
    if (error_msg_power_stage == 0) {
      temp_message << std::endl << RED << hardware_name
                   << ": [error] POWER FAULT" << RESET << std::endl
                   << std::endl;
      std::cerr << temp_message.str() << std::cerr.flush();
      // master.msg->message(lib::NON_FATAL_ERROR, "Wylaczono moc - robot zablokowany");
      error_msg_power_stage++;
    }
  } else {
    error_msg_power_stage = 0;
  }
  for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
    if (read_needed[drive_number] && !receiveFail[drive_number]) {
      if ((NFComBuf.ReadDrivesStatus.data[drive_number]
          & NF_DrivesStatus_LimitSwitchUp) != 0)
        ret |= (uint64_t) (UPPER_LIMIT_SWITCH << (5 * (drive_number)));  // Zadzialal wylacznik "gorny" krancowy
      if ((NFComBuf.ReadDrivesStatus.data[drive_number]
          & NF_DrivesStatus_LimitSwitchDown) != 0)
        ret |= (uint64_t) (LOWER_LIMIT_SWITCH << (5 * (drive_number)));  // Zadzialal wylacznik "dolny" krancowy
      if ((NFComBuf.ReadDrivesStatus.data[drive_number]
          & NF_DrivesStatus_EncoderIndexSignal) != 0)
        ret |= (uint64_t) (SYNCHRO_ZERO << (5 * (drive_number)));  // Impuls zera rezolwera
      if ((NFComBuf.ReadDrivesStatus.data[drive_number]
          & NF_DrivesStatus_Overcurrent) != 0)
        ret |= (uint64_t) (OVER_CURRENT << (5 * (drive_number)));  // Przekroczenie dopuszczalnego pradu
      if ((NFComBuf.ReadDrivesStatus.data[drive_number]
          & NF_DrivesStatus_SynchroSwitch) != 0) {
        if (synchro_switch_filter[drive_number] == synchro_switch_filter_th)
          ret |= (uint64_t) (SYNCHRO_SWITCH_ON << (5 * (drive_number)));  // Zadzialal wylacznik synchronizacji
        else
          synchro_switch_filter[drive_number]++;
      } else {
        synchro_switch_filter[drive_number] = 0;
      }
    }
  }
  if (status_disp_cnt++ == STATUS_DISP_T) {
    status_disp_cnt = 0;
  }
  return ret;
}

uint64_t HI_moxa::write_hardware(void) {
  uint64_t ret = 0;
  uint8_t drive_number;
// static std::stringstream temp_message;

// If Hardware Panic, send PARAM_DRIVER_MODE_ERROR to motor drivers
  if (hardware_panic) {
    for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
      // only set error parameter, do not wait for answer
      servo_data[drive_number].commandCnt = 0;
      NFComBuf.SetDrivesMode.data[drive_number] = NF_DrivesMode_ERROR;
      servo_data[drive_number].commandArray[servo_data[drive_number].commandCnt++] =
      NF_COMMAND_SetDrivesMode;
      txCnt = NF_MakeCommandFrame(
          &NFComBuf, txBuf,
          (const uint8_t*) servo_data[drive_number].commandArray,
          servo_data[drive_number].commandCnt, drives_addresses[drive_number]);
      // Clear communication request
      servo_data[drive_number].commandCnt = 0;
      // Send command frame
      SerialPort[drive_number]->write(txBuf, txCnt);
    }
    if (error_msg_hardware_panic_ == 0) {
      std::cout << RED << std::endl << hardware_name
                << ": [error] hardware panic" << RESET << std::endl
                << std::endl;
      error_msg_hardware_panic_++;
    }
    // std::cout << temp_message.str() << std::endl;
    /*
     struct timespec delay;
     delay.tv_nsec = 2000000;
     delay.tv_sec = 0;

     nanosleep(&delay, NULL);
     */
    return ret;
  } else {
    // przygotowanie pod test zablokowania komunikacji po bledzie
    if (all_hardware_read)
    // if (1)
    {  // NOLINT
       // Make command frames and send them to drives
      for (drive_number = 0; drive_number <= last_drive_number;
          drive_number++) {
        // Set communication requests
        //   std::cout << "write drive_number: " << drive_number ;

        servo_data[drive_number].commandArray[servo_data[drive_number]
            .commandCnt++] =
        NF_COMMAND_ReadDrivesPosition;
        servo_data[drive_number].commandArray[servo_data[drive_number]
            .commandCnt++] =
        NF_COMMAND_ReadDrivesCurrent;
        servo_data[drive_number].commandArray[servo_data[drive_number]
            .commandCnt++] =
        NF_COMMAND_ReadDrivesStatus;
        // Make command frame
        servo_data[drive_number].txCnt = NF_MakeCommandFrame(
            &NFComBuf, servo_data[drive_number].txBuf + howMuchItSucks,
            (const uint8_t*) servo_data[drive_number].commandArray,
            servo_data[drive_number].commandCnt,
            drives_addresses[drive_number]);
        // Clear communication requests
        servo_data[drive_number].commandCnt = 0;
      }

      for (drive_number = 0; drive_number <= last_drive_number;
          drive_number++) {
        // Send command frame
        if (receiveFailCnt[drive_number] == 0)
          SerialPort[drive_number]->write(
              servo_data[drive_number].txBuf,
              servo_data[drive_number].txCnt + howMuchItSucks);

#define SPN
#ifdef SPN
        static int rwhprints = 18;
        if (rwhprints) {
          rwhprints--;
          std::cout << "RWH: ";
          for (int i = 0; i < servo_data[drive_number].txCnt + howMuchItSucks;
              i++) {
            std::cout << std::hex
                      << (unsigned int) servo_data[drive_number].txBuf[i];
            std::cout << " ";
          }
          std::cout << std::endl;
        }
#endif
      }
    } else {
      // std::cout << "write hardware !all_hardware_read " << std::endl;
    }
  }

  ret = 1;
  return ret;
}

// do communication cycle
uint64_t HI_moxa::write_read_hardware(uint64_t nsec,
                                      uint64_t timeouts_to_print) {
  struct timespec start_time, current_time, read_time;
  clock_gettime(CLOCK_MONOTONIC, &start_time);

  uint64_t ret = 0;
  if ((ret = write_hardware()) != 0) {
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    int64_t current_delay = (current_time.tv_sec - start_time.tv_sec)
        * 1000000000 + (current_time.tv_nsec - start_time.tv_nsec);

    if (current_delay > longest_delay_) {
      longest_delay_ = current_delay;
//  std::cout << std::dec << "longest_delay_: " << longest_delay_ << std::endl;
    }

    struct timespec delay;
    delay.tv_nsec = nsec;
    delay.tv_sec = 0;

// clock_nanosleep(CLOCK_MONOTONIC,0, &delay, NULL);

    nanosleep(&delay, NULL);

// std::cout << std::dec << "longest_delay_: " << longest_delay_ << std::endl;
    /*
     hi_sleep(nsec);
     */

    ret = read_hardware(timeouts_to_print);
    clock_gettime(CLOCK_MONOTONIC, &read_time);

    int64_t read_delay = (read_time.tv_sec - current_time.tv_sec) * 1000000000
        + (read_time.tv_nsec - current_time.tv_nsec);

    if (read_delay > longest_read_delay_) {
      longest_read_delay_ = read_delay;
//  std::cout << std::dec << "longest_read_delay_: " << longest_read_delay_    << std::endl;
    }
  }

// std::cout <<"write_read_hardware ret: " << ret << std::endl;

  return ret;
}

void HI_moxa::set_hardware_panic(void) {
  hardware_panic = true;
}

// send parameter to motor driver
void HI_moxa::set_parameter(int drive_number, const int parameter, ...) {
}

// read motor current from communication buffer
int HI_moxa::set_parameter_now(int drive_number, const int parameter, ...) {
  struct timespec delay;
  uint8_t setParamCommandCnt = 0;
  uint8_t setParamCommandArray[10];

  va_list newValue;
  va_start(newValue, parameter);

  switch (parameter) {
    case NF_COMMAND_SetDrivesMisc:
      NFComBuf.SetDrivesMisc.data[drive_number] = (uint32_t) va_arg(newValue,
                                                                    int);
      setParamCommandArray[setParamCommandCnt++] = NF_COMMAND_SetDrivesMisc;
      break;
    case NF_COMMAND_SetDrivesMaxCurrent:
      NFComBuf.SetDrivesMaxCurrent.data[drive_number] = (int16_t) va_arg(
          newValue, int);
      setParamCommandArray[setParamCommandCnt++] =
      NF_COMMAND_SetDrivesMaxCurrent;
      break;
    case NF_COMMAND_SetDrivesMode:
      NFComBuf.SetDrivesMode.data[drive_number] = (uint8_t) va_arg(newValue,
                                                                   int);
      setParamCommandArray[setParamCommandCnt++] = NF_COMMAND_SetDrivesMode;
      break;
    case NF_COMMAND_SetCurrentRegulator:
      NFComBuf.SetCurrentRegulator.data[drive_number] = va_arg(
          newValue, NF_STRUCT_Regulator);
      setParamCommandArray[setParamCommandCnt++] =
      NF_COMMAND_SetCurrentRegulator;
      break;
    default:
      std::cout << "[error] HI_moxa::set_parameter_now() invalid parameter "
                << static_cast<int>(parameter) << std::endl;
      return -1;
      break;
  }
  va_end(newValue);

// Add Read Drive Status request
  setParamCommandArray[setParamCommandCnt++] = NF_COMMAND_ReadDrivesStatus;
// Make command frame
  txCnt = NF_MakeCommandFrame(&NFComBuf, txBuf,
                              (const uint8_t*) setParamCommandArray,
                              setParamCommandCnt,
                              drives_addresses[drive_number]);
// Clear communication request
  setParamCommandCnt = 0;

  for (int param_set_attempt = 0; param_set_attempt < MAX_PARAM_SET_ATTEMPTS;
      param_set_attempt++) {
    // Send command frame
    SerialPort[drive_number]->write(txBuf, txCnt);

#define SPN
#ifdef SPN
    std::cout << "SPN: ";
    for (int i = 0; i < txCnt; i++) {
      std::cout << std::hex << (unsigned int) txBuf[i];
      std::cout << " ";
    }
    std::cout << std::endl;
#endif

    // hardware panic; do not print error information; do not wait for response
    if (parameter == NF_COMMAND_SetDrivesMode
        && NFComBuf.SetDrivesMode.data[drive_number] == NF_DrivesMode_ERROR)
      return 0;

    // Give some time for a response to return
    delay.tv_nsec = 1700000;
    delay.tv_sec = 0;
    nanosleep(&delay, NULL);

    drive_buff[drive_number].rxCnt = 0;
    while (1) {
      if (SerialPort[drive_number]->read(
          &(drive_buff[drive_number].rxBuf[drive_buff[drive_number].rxCnt]), 1)
          > 0 && (drive_buff[drive_number].rxCnt < 255)) {
        if (NF_Interpreter(&NFComBuf, drive_buff[drive_number].rxBuf,
                           &drive_buff[drive_number].rxCnt, rxCommandArray,
                           &rxCommandCnt) > 0) {
          drive_buff[drive_number].rxCnt = 0;
          return 0;
        }
      } else {
        std::cout << "[error] param (" << parameter
                  << ") set ack timeout for drive (" << drive_number << ")"
                  << std::endl;
        break;
      }
    }
  }
  throw std::runtime_error(
      "HI_Moxa: Unable to communicate with motor controllers. Try switching the power on.");
  return 1;
}

void HI_moxa::start_synchro(int drive_number) {
  if (NFComBuf.SetDrivesMode.data[drive_number] == NF_DrivesMode_PWM)
    NFComBuf.SetDrivesMode.data[drive_number] = NF_DrivesMode_SYNC_PWM0;
  else if (NFComBuf.SetDrivesMode.data[drive_number] == NF_DrivesMode_CURRENT) {
    NFComBuf.SetDrivesMode.data[drive_number] = NF_DrivesMode_SYNC_CURRENT0;
  } else {
    hardware_panic = true;
    std::stringstream temp_message;
    temp_message << "[error] unknown mode on drive "
                 << static_cast<int>(drive_number)
                 << " when start_synchro() called" << std::endl;
    std::cout << temp_message.str();
  }
  servo_data[drive_number].commandArray[servo_data[drive_number].commandCnt++] =
  NF_COMMAND_SetDrivesMode;
  std::cout << "[func] HI_moxa::start_synchro(" << drive_number << ")"
            << std::endl;
  check_ridicolous_increment_[drive_number] = false;
}

void HI_moxa::finish_synchro(int drive_number) {
  if (NFComBuf.SetDrivesMode.data[drive_number] == NF_DrivesMode_SYNC_PWM0)
    NFComBuf.SetDrivesMode.data[drive_number] = NF_DrivesMode_PWM;
  else if (NFComBuf.SetDrivesMode.data[drive_number]
      == NF_DrivesMode_SYNC_CURRENT0) {
    NFComBuf.SetDrivesMode.data[drive_number] = NF_DrivesMode_CURRENT;
  } else {
    hardware_panic = true;
    std::stringstream temp_message;
    temp_message << "[error] unknown mode on drive "
                 << static_cast<int>(drive_number)
                 << " when finish_synchro() called" << std::endl;
    std::cout << temp_message.str();
  }
  servo_data[drive_number].commandArray[servo_data[drive_number].commandCnt++] =
  NF_COMMAND_SetDrivesMode;
  std::cout << "[func] HI_moxa::finish_synchro(" << drive_number << ")"
            << std::endl;

  check_ridicolous_increment_[drive_number] = true;
}

void HI_moxa::unsynchro(int drive_number) {
  set_parameter_now(drive_number, NF_COMMAND_SetDrivesMisc,
  NF_DrivesMisc_ResetSynchronized);

  std::cout << "[func] HI_moxa::unsynchro(" << drive_number << ")" << std::endl;
}

bool HI_moxa::in_synchro_area(int drive_number) {
  return ((NFComBuf.ReadDrivesStatus.data[drive_number]
      & NF_DrivesStatus_SynchroSwitch) != 0);
}

bool HI_moxa::robot_synchronized() {
  bool ret = true;
  for (std::size_t drive_number = 0; drive_number <= last_drive_number;
      drive_number++) {
    if ((NFComBuf.ReadDrivesStatus.data[drive_number]
        & NF_DrivesStatus_Synchronized) == 0) {
      ret = false;
    }
  }
  std::cout << "[func] HI_moxa::robot_synchronized()" << std::endl;
  return ret;
}

bool HI_moxa::drive_synchronized(int drive_number) {
  if ((NFComBuf.ReadDrivesStatus.data[drive_number]
      & NF_DrivesStatus_Synchronized) == 0) {
    return false;
  } else {
    return true;
  }
}

void HI_moxa::reset_position(int drive_number) {
  servo_data[drive_number].current_absolute_position = 0L;
  servo_data[drive_number].previous_absolute_position = 0L;
  servo_data[drive_number].current_position_inc = 0.0;
  servo_data[drive_number].previous_position_inc = 0.0;

  servo_data[drive_number].first_hardware_reads =
      FIRST_HARDWARE_READS_WITH_ZERO_INCREMENT;
  std::cout << "[func] HI_moxa::reset_position(" << drive_number << ")"
            << std::endl;
}

void HI_moxa::clear_buffer(int drive_number) {
  servo_data[drive_number].commandCnt = 0;
}

uint16_t HI_moxa::convert_to_115(float input) {
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
  return output;
}

}  // namespace hi_moxa

