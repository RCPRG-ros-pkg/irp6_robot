#ifndef Irp6ptfgTransmission_H_
#define Irp6ptfgTransmission_H_

const double GEAR[1] = {-158.0,};
const double SYNCHRO_MOTOR_POSITION[1] = {-15.9};
const double THETA[1] = {0.0};

const double  SYNCHRO_JOINT_POSITION[1] = { SYNCHRO_MOTOR_POSITION[0] - GEAR[0] * THETA[0] };

const int ENC_RES[1] = {128.0};

const double LOWER_MOTOR_LIMIT[1] = { -2000.0};
const double UPPER_MOTOR_LIMIT[1] = { 5000.0};

#endif /* Irp6ptfgTransmission_H_ */
