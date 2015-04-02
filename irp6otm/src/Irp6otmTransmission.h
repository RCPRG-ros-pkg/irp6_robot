/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IRP6OTMTRANSMISSION_H_
#define IRP6OTMTRANSMISSION_H_

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

#endif  // IRP6OTMTRANSMISSION_H_
