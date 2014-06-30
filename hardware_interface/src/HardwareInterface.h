#ifndef HARDWAREINTERFACE_H_
#define HARDWAREINTERFACE_H_

#include "hi_moxa.h"

using namespace RTT;

typedef enum {
  NOT_SYNCHRONIZED,
  SERVOING,
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
  std::vector<OutputPort<int>*> deltaInc_out_list_;

  std::vector<OutputPort<double>*> port_motor_position_list_;

  std::vector<InputPort<double>*> port_motor_position_command_list_;

  Eigen::VectorXd motor_position_, motor_position_command_,
      motor_position_command_old_;

  int number_of_drives_;
  bool auto_synchronize_;

  // Properties
  std::vector<std::string> ports_adresses_;
  std::vector<int> max_current_;
  std::vector<double> max_increment_;
  std::vector<unsigned int> card_indexes_;
  int tx_prefix_len_;
  std::vector<int> enc_res_;
  std::vector<double> synchro_step_coarse_;
  std::vector<double> synchro_step_fine_;
  std::vector<bool> current_mode_;

  int servo_stop_iter_;

  double counter_;

  State state_;
  SynchroState synchro_state_;
  int synchro_drive_;

  std::vector<double> pos_inc_;

  std::vector<int> increment_;
  std::vector<double> motor_pos_;
  std::vector<double> pwm_;

  hi_moxa::HI_moxa *hi_;

 public:
  HardwareInterface(const std::string& name);
  ~HardwareInterface();

  bool configureHook();
  bool startHook();
  void updateHook();
};

#endif // HARDWAREINTERFACE_H_
