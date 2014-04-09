#include <rtt/Component.hpp>

#include "irp6p_transmission_inv.h"

const double GEAR[6] = {-158.0, 2*M_PI/5.0, 2*M_PI/5.0, -128.0, -128.0*0.6, 288.8845};
const double SYNCHRO_MOTOR_POSITION[6] = {-15.9, -5.0, -8.527, 151.31, 432.25, 791.0};
const double THETA[6] = {0.0, 2.203374e+02, 1.838348e+02, 1.570796e+00, 0.0, 0.0};

const double 	SYNCHRO_JOINT_POSITION[6] = { SYNCHRO_MOTOR_POSITION[0] - GEAR[0] * THETA[0],
                                            SYNCHRO_MOTOR_POSITION[1] - GEAR[1] * THETA[1],
	                                          SYNCHRO_MOTOR_POSITION[2] - GEAR[2] * THETA[2],
	                                          SYNCHRO_MOTOR_POSITION[3] - GEAR[3] * THETA[3],
	                                          SYNCHRO_MOTOR_POSITION[4] - GEAR[4] * THETA[4] - SYNCHRO_MOTOR_POSITION[3],
	                                          SYNCHRO_MOTOR_POSITION[5] - GEAR[5] * THETA[5] };

const int ENC_RES[6] = {4000, 4000, 4000, 4000, 4000, 2000};

const double LOWER_MOTOR_LIMIT[6] = { -470, -110, -80, -70, -80, -1000};
const double UPPER_MOTOR_LIMIT[6] = { 450, 100, 100, 380, 490, 3000};

IRP6PTransmissionInv::IRP6PTransmissionInv(const std::string& name) : RTT::TaskContext(name, PreOperational) {

	this->ports()->addPort("MotorPosition", port_motor_position_);
	this->ports()->addPort("JointPosition", port_joint_position_);
}

IRP6PTransmissionInv::~IRP6PTransmissionInv() {

}

bool IRP6PTransmissionInv::configureHook() {
	motor_position_.resize(6);
	joint_position_.resize(6);
	return true;
}

void IRP6PTransmissionInv::updateHook() {
	if(port_joint_position_.read(joint_position_) == RTT::NewData) {
	  if(i2mp(&joint_position_(0), &motor_position_(0))) {
	    port_motor_position_.write(motor_position_);
	  }
	}
}

bool IRP6PTransmissionInv::i2mp(const double* joints, double* motors)
{
  const double sl123 = 7.789525e+04;
  const double mi1 = 6.090255e+04;
  const double ni1 = -2.934668e+04;

  const double mi2 = -4.410000e+04;
  const double ni2 = -5.124000e+04;

  // Niejednoznacznosc polozenia dla 3-tej osi (obrot kisci < 180).
  const double joint_3_revolution = M_PI;
  // Niejednoznacznosc polozenia dla 4-tej osi (obrot kisci > 360).
  const double axis_4_revolution = 2 * M_PI;

  // Obliczanie kata obrotu walu silnika napedowego kolumny
  motors[0] = GEAR[0] * joints[0] + SYNCHRO_JOINT_POSITION[0];

  // Obliczanie kata obrotu walu silnika napedowego ramienia dolnego
  motors[1] = GEAR[1] * sqrt(sl123 + mi1 * cos(joints[1]) + ni1
* sin(-joints[1])) + SYNCHRO_JOINT_POSITION[1];

  // Obliczanie kata obrotu walu silnika napedowego ramienia gornego
  motors[2] = GEAR[2] * sqrt(sl123 + mi2 * cos(joints[2] + joints[1] + M_PI_2) + ni2
* sin(-(joints[2] + joints[1] + M_PI_2))) + SYNCHRO_JOINT_POSITION[2];

  // Obliczanie kata obrotu walu silnika napedowego obotu kisci T
  // jesli jest mniejsze od -pi/2
  double joints_tmp3 = joints[3] + joints[2] + joints[1] + M_PI_2;
  if (joints_tmp3 < -M_PI_2)
    joints_tmp3 += joint_3_revolution;
  motors[3] = GEAR[3] * (joints_tmp3 + THETA[3]) + SYNCHRO_JOINT_POSITION[3];

  // Obliczanie kata obrotu walu silnika napedowego obrotu kisci V
  motors[4] = GEAR[4] * joints[4] + SYNCHRO_JOINT_POSITION[4]
            + motors[3];

  // Ograniczenie na obrot.
  while (motors[4] < -80)
    motors[4] += axis_4_revolution;
  while (motors[4] > 490)
    motors[4] -= axis_4_revolution;

  // Obliczanie kata obrotu walu silnika napedowego obrotu kisci N
  motors[5] = GEAR[5] * joints[5] + SYNCHRO_JOINT_POSITION[5];

  return checkMotorPosition(motors);
}

bool IRP6PTransmissionInv::checkMotorPosition(const double * motor_position)
{

  if (motor_position[0] < LOWER_MOTOR_LIMIT[0]) // Kat f1 mniejszy od minimalnego
    return false;//throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_0);
  else if (motor_position[0] > UPPER_MOTOR_LIMIT[0]) // Kat f1 wiekszy od maksymalnego
    return false;//throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_0);

  if (motor_position[1] < LOWER_MOTOR_LIMIT[1]) // Kat f2 mniejszy od minimalnego
    return false;//throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_1);
  else if (motor_position[1] > UPPER_MOTOR_LIMIT[1]) // Kat f2 wiekszy od maksymalnego
    return false;//throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_1);

  if (motor_position[2] < LOWER_MOTOR_LIMIT[2]) // Kat f3 mniejszy od minimalnego
    return false;//throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_2);
  else if (motor_position[2] > UPPER_MOTOR_LIMIT[2]) // Kat f3 wiekszy od maksymalnego
    return false;//throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_2);

  if (motor_position[3] < LOWER_MOTOR_LIMIT[3]) // Kat f4 mniejszy od minimalnego
    return false; //throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_3);
  else if (motor_position[3] > UPPER_MOTOR_LIMIT[3]) // Kat f4 wiekszy od maksymalnego
    return false; //throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_3);

  if (motor_position[4] < LOWER_MOTOR_LIMIT[4]) // Kat f5 mniejszy od minimalnego
    return false; //throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_4);
  else if (motor_position[4] > UPPER_MOTOR_LIMIT[4]) // Kat f5 wiekszy od maksymalnego
    return false; //throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_4);

  if (motor_position[5] < LOWER_MOTOR_LIMIT[5]) // Kat f6 mniejszy od minimalnego
    return false; //throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_5);
  else if (motor_position[5] > UPPER_MOTOR_LIMIT[5]) // Kat f6 wiekszy od maksymalnego
    return false;//throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_5);

  return true;
} //: check_motor_position


ORO_CREATE_COMPONENT(IRP6PTransmissionInv)

