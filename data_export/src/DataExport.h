#ifndef DATAEXPORT_H_
#define DATAEXPORT_H_

#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
#include <stdio.h>
#include <unistd.h>
//#include "matio.h"

#include <hi_msgs/HardwareInterfacePort.h>

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

  std::vector<InputPort<double>*> port_motor_position_list_;
  std::vector<InputPort<double>*> port_motor_increment_list_;
  std::vector<InputPort<double>*> port_motor_current_list_;

  std::vector<std::string> label_;

  double positionData;
  double incrementData;
  double currentData;

  // Properties
  int number_of_drives_;
  std::string filename_;
  bool debug_;
  bool matfile_;
  bool csvfile_;
  hi_msgs::HardwareInterfacePort hi_port_param_[16];
  long step;

  ofstream csv;

};
#endif
