#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include "MatExport.h"

const int MAX_PWM = 190;

MatExport::MatExport(const std::string& name)
    : TaskContext(name),
      position_in("position_in"),
      increment_in("increment_in"),
      voltage_in("voltage_in"),
      current_in("current_in"),
      positionData(0.0),
      incrementData(0.0),
      voltageData(0.0),
      currentData(0.0)
      {

	  this->addEventPort(position_in).doc("Receiving a value of motor position");
	  this->addPort(increment_in).doc("Receiving a value of motor increment");
	  this->addPort(voltage_in).doc("Receiving a value of motor voltage");
	  this->addPort(current_in).doc("Receiving a value of motor current");


  this->addProperty("reg_number", reg_number_).doc("");
  this->addProperty("debug", debug_).doc("");

  step_reg=0;
}

MatExport::~MatExport() {

}

bool MatExport::configureHook() {
  reset();

  return true;
}

void MatExport::updateHook()
{
	  step_reg++;

	  if (debug_)
	  {
		  std::cout << step_reg;
		  if (NewData == position_in.read(positionData)
				  && NewData == increment_in.read(incrementData)
				  && NewData == voltage_in.read(voltageData)
				  && NewData == current_in.read(currentData))
		  {
			  std::cout << "  positionData: " << positionData;
			  std::cout << "  incrementData: " << incrementData;
			  std::cout << "  voltageData: " << voltageData;
			  std::cout << "  currentData: " << currentData;
		  }
		  std::cout << std::endl;
	  }
}


void MatExport::reset() {
}

ORO_CREATE_COMPONENT(MatExport)
