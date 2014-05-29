#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include "IRp6Regulator.h"

IRp6Regulator::IRp6Regulator(const std::string& name) :
		TaskContext(name), posInc_in("posInc_in"), deltaInc_in("deltaInc_in"), computedPwm_out(
				"computedPwm_out") {

	this->addEventPort(posInc_in).doc("Receiving a value of position step");
	this->addPort(deltaInc_in).doc("Receiving a value of measured increment.");
	this->addPort(computedPwm_out).doc("Sending value of calculated pwm.");

	this->addProperty("number_of_drives", number_of_drives).doc(
			"Number of drives in robot");
}

IRp6Regulator::~IRp6Regulator() {

}

bool IRp6Regulator::configureHook() {

	for (int i = 0; i < number_of_drives; i++) {
		regulator[i].reset();
		regulator[i].setParam(A[i], BB0[i], BB1[i]);
	}
}

void IRp6Regulator::updateHook() {
	if (NewData == posInc_in.read(posIncData)
			&& NewData == deltaInc_in.read(deltaIncData)) {
		computedPwm_out.write(computePwmValue(posIncData, deltaIncData));
	}
}

std::vector<double> IRp6Regulator::computePwmValue(
		const std::vector<double>& posInc, const std::vector<int>& deltaInc) {
	std::vector<double> ret(number_of_drives);
	for (int i = 0; i < number_of_drives; i++) {
		ret[i] = regulator[i].doServo(posInc[i], deltaInc[i]);
	}

	return ret;
}

ORO_CREATE_COMPONENT(IRp6Regulator)
