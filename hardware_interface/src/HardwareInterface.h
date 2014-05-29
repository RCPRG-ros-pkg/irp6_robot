#ifndef HARDWAREINTERFACE_H_
#define HARDWAREINTERFACE_H_


#include "hi_moxa.h"

//IRP6
//wyrzucić niepotrzebne
//dodać nazwy portów
//reszte do parametr serwera

const int NUMBER_OF_DRIVES = 6;
const int16_t MAX_CURRENT[] = { 25000, 18000, 15000, 17000, 10000, 2000 };
const double MAX_INCREMENT[] = { 1000, 1000, 1000, 1000, 1000, 1000 };
const unsigned int CARD_ADDRESSES[] = { 0, 1, 2, 3, 4, 5 };
const int TX_PREFIX_LEN = 0;

const double GEAR[6] = { -158.0, 2 * M_PI / 5.0, 2 * M_PI / 5.0, -128.0, -128.0
		* 0.6, 288.8845 };
const double THETA[6] = { 0.0, 2.203374e+02, 1.838348e+02, 1.570796e+00, 0.0,
		0.0 };

const double SYNCHRO_MOTOR_POSITION[6] = { -15.9, -5.0, -8.527, 151.31, 432.25,
		791.0 };
const double SYNCHRO_JOINT_POSITION[6] = { SYNCHRO_MOTOR_POSITION[0]
		- GEAR[0] * THETA[0], SYNCHRO_MOTOR_POSITION[1] - GEAR[1] * THETA[1],
		SYNCHRO_MOTOR_POSITION[2] - GEAR[2] * THETA[2],
		SYNCHRO_MOTOR_POSITION[3] - GEAR[3] * THETA[3],
		SYNCHRO_MOTOR_POSITION[4] - GEAR[4] * THETA[4]
				- SYNCHRO_MOTOR_POSITION[3], SYNCHRO_MOTOR_POSITION[5]
				- GEAR[5] * THETA[5] };

const int ENC_RES[6] = { 4000, 4000, 4000, 4000, 4000, 2000 };
const double SYNCHRO_STEP_COARSE[6] =
		{ -0.03, -0.03, -0.03, -0.03, -0.03, -0.05 };
const double SYNCHRO_STEP_FINE[6] = { 0.007, 0.007, 0.007, 0.007, 0.007, 0.05 };

using namespace RTT;

typedef enum {
	NOT_SYNCHRONIZED, SERVOING, SYNCHRONIZING
} State;
typedef enum {
	MOVE_TO_SYNCHRO_AREA,
	STOP,
	MOVE_FROM_SYNCHRO_AREA,
	WAIT_FOR_IMPULSE,
	SYNCHRO_END
} SynchroState;

class HardwareInterface: public RTT::TaskContext {

private:

	InputPort<std::vector<double> > computedPwm_in;

	OutputPort<std::vector<double> > posInc_out;
	OutputPort<std::vector<int> > deltaInc_out;

	OutputPort<Eigen::VectorXd> port_motor_position_;
	InputPort<Eigen::VectorXd> port_motor_position_command_;

	Eigen::VectorXd motor_position_, motor_position_command_,
			motor_position_command_old_;

	int number_of_drives;
	bool auto_synchronize;

	double counter;

	State state;
	SynchroState synchro_state;
	int synchro_drive;

	std::vector<double> pos_inc;

	std::vector<int> increment;
	std::vector<double> motor_pos;
	std::vector<double> pwm;

	hi_moxa::HI_moxa hi_;

	bool debug;

public:

	HardwareInterface(const std::string& name);
	~HardwareInterface();

	bool configureHook();
	bool startHook();
	void updateHook();

};

#endif /*HARDWAREINTERFACE_H_*/
