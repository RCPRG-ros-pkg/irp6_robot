#ifndef SarkofagREGULATOR_H_
#define SarkofagREGULATOR_H_

using namespace RTT;

class SarkofagRegulator : public RTT::TaskContext {

 public:

  SarkofagRegulator(const std::string& name);
  ~SarkofagRegulator();

  int doServo(double, int);
  void reset();

 private:

  bool configureHook();
  void updateHook();

  InputPort<double> posInc_in;
  InputPort<double> deltaInc_in;

  OutputPort<double> computedPwm_out;

  double posIncData;
  double deltaIncData;

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

};
#endif
