#ifndef Irp6pmTransmission_H_
#define Irp6pmTransmission_H_

#define NUMBER_OF_SERVOS 6

const double GEAR[NUMBER_OF_SERVOS] = {-158.0, 2*M_PI/5.0, 2*M_PI/5.0, -128.0, -128.0*0.6, 288.8845};
const double SYNCHRO_MOTOR_POSITION[NUMBER_OF_SERVOS] = {-15.9, -5.0, -8.527, 151.31, 432.25, 791.0};
const double THETA[NUMBER_OF_SERVOS] = {0.0, 2.203374e+02, 1.838348e+02, 1.570796e+00, 0.0, 0.0};

const double  SYNCHRO_JOINT_POSITION[NUMBER_OF_SERVOS] = { SYNCHRO_MOTOR_POSITION[0] - GEAR[0] * THETA[0],
                                            SYNCHRO_MOTOR_POSITION[1] - GEAR[1] * THETA[1],
                                            SYNCHRO_MOTOR_POSITION[2] - GEAR[2] * THETA[2],
                                            SYNCHRO_MOTOR_POSITION[3] - GEAR[3] * THETA[3],
                                            SYNCHRO_MOTOR_POSITION[4] - GEAR[4] * THETA[4] - SYNCHRO_MOTOR_POSITION[3],
                                            SYNCHRO_MOTOR_POSITION[5] - GEAR[5] * THETA[5] };

const int ENC_RES[NUMBER_OF_SERVOS] = {4000, 4000, 4000, 4000, 4000, 2000};

const double LOWER_MOTOR_LIMIT[NUMBER_OF_SERVOS] = { -470, -110, -80, -70, -80, -1000};
const double UPPER_MOTOR_LIMIT[NUMBER_OF_SERVOS] = { 450, 100, 100, 380, 490, 3000};

const double sl123 = 7.789525e+04;
const double mi1 = 6.090255e+04;
const double ni1 = -2.934668e+04;

const double mi2 = -4.410000e+04;
const double ni2 = -5.124000e+04;

#endif /* Irp6pmTransmission_H_ */
