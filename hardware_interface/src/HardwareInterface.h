#ifndef HARDWAREINTERFACE_H_
#define HARDWAREINTERFACE_H_

#include "hi_moxa.h"
#include <hi_msgs/HardwareInterfacePort.h>

using namespace RTT;

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

  std::vector<InputPort<double>*> computedReg_in_list_;

  std::vector<OutputPort<double>*> posInc_out_list_;
  std::vector<OutputPort<double>*> deltaInc_out_list_;

  std::vector<OutputPort<double>*> port_motor_position_list_;
  std::vector<OutputPort<double>*> port_motor_increment_list_;
  //std::vector<OutputPort<double>*> port_motor_voltage_list_;
  std::vector<OutputPort<double>*> port_motor_current_list_;

  std::vector<InputPort<double>*> port_motor_position_command_list_;

  Eigen::VectorXd motor_position_, motor_increment_, motor_current_, motor_position_command_,
      motor_position_command_old_;//, motor_voltage_;

  std::vector<std::string> ports_adresses_;
  std::vector<int> max_current_;
  std::vector<double> max_increment_;
  std::vector<double> max_desired_increment_;
  std::vector<unsigned int> card_indexes_;
  std::vector<double> enc_res_;
  std::vector<double> synchro_step_coarse_;
  std::vector<double> synchro_step_fine_;
  std::vector<bool> current_mode_;
  std::vector<bool> synchro_needed_;

  // Properties
  int number_of_drives_;
  bool auto_synchronize_;
  bool test_mode_;
  int timeouts_to_print_;
  int tx_prefix_len_;
  int rwh_nsec_;
  hi_msgs::HardwareInterfacePort hi_port_param_[hi_moxa::MOXA_SERVOS_NR];

  int synchro_stop_iter_;
  int synchro_start_iter_;
  int servo_start_iter_;

  double counter_;

  State state_;
  SynchroState synchro_state_;
  int synchro_drive_;
  bool burst_mode_;

  std::vector<double> pos_inc_;
  std::vector<double> max_pos_inc_;

  std::vector<double> increment_;
  std::vector<double> motor_pos_;
  std::vector<double> pwm_or_current_;

  std::vector<RTT::TaskContext*> servo_list_;

  hi_moxa::HI_moxa *hi_;

  uint16_t convert_to_115(float input);
  void test_mode_sleep();

 public:
  HardwareInterface(const std::string& name);
  ~HardwareInterface();

  bool configureHook();
  bool startHook();
  void updateHook();

};

#endif // HARDWAREINTERFACE_H_
