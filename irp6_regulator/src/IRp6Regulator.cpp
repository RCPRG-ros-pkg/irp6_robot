#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include "IRp6Regulator.h"

IRp6Regulator::IRp6Regulator(const std::string& name)
    : TaskContext(name),
      posInc_in("posInc_in"),
      deltaInc_in("deltaInc_in"),
      computedPwm_out("computedPwm_out") {

  this->addEventPort(posInc_in).doc("Receiving a value of position step");
  this->addPort(deltaInc_in).doc("Receiving a value of measured increment.");
  this->addPort(computedPwm_out).doc("Sending value of calculated pwm.");

  this->addProperty("A", A_).doc("");
  this->addProperty("BB0", BB0_).doc("");
  this->addProperty("BB1", BB1_).doc("");
}

IRp6Regulator::~IRp6Regulator() {

}

bool IRp6Regulator::configureHook() {
  regulator.reset();
  regulator.setParam(A_, BB0_, BB1_);
  return true;
}

void IRp6Regulator::updateHook() {
  if (NewData == posInc_in.read(posIncData)
      && NewData == deltaInc_in.read(deltaIncData)) {
    computedPwm_out.write(computePwmValue(posIncData, deltaIncData));
  }
}

double IRp6Regulator::computePwmValue(const double& posInc,
                                      const int& deltaInc) {

  double ret = regulator.doServo(posInc, deltaInc);

  return ret;
}

ORO_CREATE_COMPONENT(IRp6Regulator)
