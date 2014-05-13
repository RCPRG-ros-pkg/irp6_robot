
class Regulator
{
public:
  Regulator();
  ~Regulator();

  int doServo(double, int);
  void reset();
  void setParam(double a, double b0, double b1);
private:
  double position_increment_old; // przedosatnio odczytany przyrost polozenie (delta y[k-2]
  // -- mierzone w impulsach)
  double position_increment_new; // ostatnio odczytany przyrost polozenie (delta y[k-1]
  // -- mierzone w impulsach)
  double step_old_pulse; // poprzednia wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-2]
  // -- mierzone w radianach)
  double step_new; // nastepna wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-1]
  // -- mierzone w radianach)
  double step_old; // poprzednia wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-1]
  // -- mierzone w radianach)

  double set_value_new; // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k])
  double set_value_old; // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k-1])
  double set_value_very_old; // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k-2])
  double delta_eint; // przyrost calki uchybu
  double delta_eint_old; // przyrost calki uchybu w poprzednim kroku

  double a_, b0_, b1_;
};
