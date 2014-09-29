#ifndef MatEXPORT_H_
#define MatEXPORT_H_

using namespace RTT;

class MatExport : public RTT::TaskContext {

 public:

  MatExport(const std::string& name);
  ~MatExport();

  void reset();

 private:

  bool configureHook();
  void updateHook();

  InputPort<double> position_in;
  InputPort<double> increment_in;
  InputPort<double> voltage_in;
  InputPort<double> current_in;

  double positionData;
  double incrementData;
  double voltageData;
  double currentData;

  // Properties
  int reg_number_;
  bool debug_;

  long step_reg;

};
#endif
