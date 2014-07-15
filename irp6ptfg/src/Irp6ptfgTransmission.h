#ifndef Irp6ptfgTransmission_H_
#define Irp6ptfgTransmission_H_

const double GEAR[6] = {-158.0, 2*M_PI/5.0, 2*M_PI/5.0, -128.0, -128.0*0.6, 288.8845};
const double SYNCHRO_MOTOR_POSITION[6] = {-15.9, -5.0, -8.527, 151.31, 432.25, 791.0};
const double THETA[6] = {0.0, 2.203374e+02, 1.838348e+02, 1.570796e+00, 0.0, 0.0};

const double  SYNCHRO_JOINT_POSITION[6] = { SYNCHRO_MOTOR_POSITION[0] - GEAR[0] * THETA[0],
                                            SYNCHRO_MOTOR_POSITION[1] - GEAR[1] * THETA[1],
                                            SYNCHRO_MOTOR_POSITION[2] - GEAR[2] * THETA[2],
                                            SYNCHRO_MOTOR_POSITION[3] - GEAR[3] * THETA[3],
                                            SYNCHRO_MOTOR_POSITION[4] - GEAR[4] * THETA[4] - SYNCHRO_MOTOR_POSITION[3],
                                            SYNCHRO_MOTOR_POSITION[5] - GEAR[5] * THETA[5] };

const int ENC_RES[6] = {4000, 4000, 4000, 4000, 4000, 2000};

const double LOWER_MOTOR_LIMIT[6] = { -470, -110, -80, -70, -80, -1000};
const double UPPER_MOTOR_LIMIT[6] = { 450, 100, 100, 380, 490, 3000};

#endif /* Irp6ptfgTransmission_H_ */
