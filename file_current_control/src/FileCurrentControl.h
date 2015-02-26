#ifndef FILECURRENTCONTROL_H_
#define FILECURRENTCONTROL_H_

#include <fstream>
#include <string>

using namespace std;
using namespace RTT;

class FileCurrentControl : public RTT::TaskContext {

 public:

  FileCurrentControl(const std::string& name);
  ~FileCurrentControl();

  int doServo(double, int);
  void reset();

 private:

  bool configureHook();
  void updateHook();

  InputPort<double> desired_position_;
  InputPort<double> deltaInc_in;
  InputPort<bool> synchro_state_in_;

  OutputPort<double> computedPwm_out;
  OutputPort<bool> emergency_stop_out_;

  double desired_position_increment_;
  double desired_position_old_, desired_position_new_;
  double deltaIncData;

  bool synchro_state_old_, synchro_state_new_;

  long update_hook_iteration_number_;
  long new_position_iteration_number_;

  // Properties
  int reg_number_;
  bool debug_;
  double A_;
  double BB0_;
  double BB1_;
  bool current_mode_;
  double max_output_current_;
  double current_reg_kp_;
  double eint_dif_;
  double max_desired_increment_;
  double enc_res_;


  double position_increment_old;  // przedosatnio odczytany przyrost polozenie (delta y[k-2]
  // -- mierzone w impulsach)
  double position_increment_new;  // ostatnio odczytany przyrost polozenie (delta y[k-1]
  // -- mierzone w impulsach)
  double step_old_pulse;  // poprzednia wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-2]
  // -- mierzone w radianach)
  double step_new;  // nastepna wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-1]
  // -- mierzone w radianach)
  double step_old;  // poprzednia wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-1]
  // -- mierzone w radianach)

  double set_value_new;  // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k])
  double set_value_old;  // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k-1])
  double set_value_very_old;  // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k-2])
  double delta_eint;  // przyrost calki uchybu
  double delta_eint_old;  // przyrost calki uchybu w poprzednim kroku

  double output_value;

  double a_, b0_, b1_;

  std::string filename_;

  int curr;
  std::vector<double> currents;

};
#endif
