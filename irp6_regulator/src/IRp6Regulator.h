#ifndef IRP6REGULATOR_H_
#define IRP6REGULATOR_H_

#include "Regulator.h"

using namespace RTT;

class IRp6Regulator : public RTT::TaskContext {

 private:

  InputPort<double> posInc_in;
  InputPort<int> deltaInc_in;

  OutputPort<double> computedPwm_out;

  double posIncData;
  int deltaIncData;

  Regulator regulator;

  // Properties
  double A_;
  double BB0_;
  double BB1_;

 public:

  IRp6Regulator(const std::string& name);
  ~IRp6Regulator();

 private:

  bool configureHook();
  void updateHook();
  double computePwmValue(const double& posInc, const int& deltaInc);

};
#endif
