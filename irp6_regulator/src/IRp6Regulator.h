#ifndef IRP6REGULATOR_H_
#define IRP6REGULATOR_H_

#include "Regulator.h"

//IRP6
const double A[6] = { 0.412429378531, 0.655629139073, 0.315789473684,
		0.548946716233, 0.391982182628, 0.3 };
const double BB0[6] = { 2.594932 * 0.6, 1.030178 * 0.6, 1.997464 * 0.6, 1.576266
		* 0.4, 1.114648 * 0.4, 1.364 * 0.4 };
const double BB1[6] = { 2.504769 * 0.6, 0.986142 * 0.6, 1.904138 * 0.6, 1.468599
		* 0.4, 1.021348 * 0.4, 1.264 * 0.4 };

const int ENC_RES[] = { 4000, 4000, 4000, 4000, 4000, 2000 };

using namespace RTT;

class IRp6Regulator: public RTT::TaskContext {

private:

	InputPort<std::vector<double> > posInc_in;
	InputPort<std::vector<int> > deltaInc_in;

	OutputPort<std::vector<double> > computedPwm_out;

	std::vector<double> posIncData;
	std::vector<int> deltaIncData;

	int number_of_drives;

	Regulator regulator[6];

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
