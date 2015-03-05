#ifndef Irp6otmTransmission_H_
#define Irp6otmTransmission_H_

#define NUMBER_OF_SERVOS 7

const double GEAR[NUMBER_OF_SERVOS] = { (1000 * 2 * 2 * M_PI) / 8.0, -158.0, 2
    * M_PI / 5.0, 2 * M_PI / 5.0, -128.0, -128.0 * 0.6, 288.8845 };

const double THETA[NUMBER_OF_SERVOS] = { 0.0, 0.0, 2.203374e+02, 1.838348e+02,
    1.570796e+00, 0.0, 0.0 };

const int ENC_RES[NUMBER_OF_SERVOS] =
    { 4000, 4000, 4000, 4000, 4000, 4000, 2000 };

const double LOWER_MOTOR_LIMIT[NUMBER_OF_SERVOS] = { -200, -470, -110, -80, -70,
    -50, -1000 };
const double UPPER_MOTOR_LIMIT[NUMBER_OF_SERVOS] = { 1900, 450, 100, 100, 380,
    490, 3000 };

const double sl123 = 7.789525e+04;
const double mi2 = 6.090255e+04;
const double ni2 = -2.934668e+04;

const double mi3 = -4.410000e+04;
const double ni3 = -5.124000e+04;

#endif /* Irp6otmTransmission_H_ */
