#ifndef MatEXPORT_H_
#define MatEXPORT_H_

#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
#include <stdio.h>
#include <unistd.h>
//#include "matio.h"

using namespace RTT;
using namespace std;

class DataExport : public RTT::TaskContext {

 public:

	DataExport(const std::string& name);
  ~DataExport();

  void reset();

 private:

  bool configureHook();
  void updateHook();

  InputPort<double> position_in;
  InputPort<double> increment_in;
  //InputPort<double> voltage_in;
  InputPort<double> current_in;

  double positionData;
  double incrementData;
  //double voltageData;
  double currentData;

  // Properties
  int drive_number_;
  bool debug_;
  bool position_;
  bool increment_;
  bool voltage_;
  bool current_;
  bool matfile_;
  bool csvfile_;

  long step_reg;

  ofstream csv;

};
#endif
