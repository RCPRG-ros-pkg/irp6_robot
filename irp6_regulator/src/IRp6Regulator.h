#ifndef IRP6REGULATOR_H_
#define IRP6REGULATOR_H_

#include "Regulator.h"

using namespace RTT;

class IRp6Regulator : public RTT::TaskContext {

 private:

  InputPort<std::vector<double> > posInc_in;
  InputPort<std::vector<int> > deltaInc_in;

  OutputPort<std::vector<double> > computedPwm_out;

  std::vector<double> posIncData;
  std::vector<int> deltaIncData;

  Regulator regulator[6];

  // Properties
  int number_of_drives_;
  std::vector<double> A_;
  std::vector<double> BB0_;
  std::vector<double> BB1_;

 public:

  IRp6Regulator(const std::string& name);
  ~IRp6Regulator();

 private:

  bool configureHook();
  void updateHook();
  std::vector<double> computePwmValue(const std::vector<double>& posInc,
                                      const std::vector<int>& deltaInc);

};
#endif
