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

#include <rtt/Component.hpp>
#include <string>
#include "Irp6otmJ2M.h"

Irp6otmJ2M::Irp6otmJ2M(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {
  this->ports()->addPort("MotorPosition", port_motor_position_);
  this->ports()->addPort("JointPosition", port_joint_position_);

  this->addProperty("synchro_motor_position", synchro_motor_position_);

  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
    SYNCHRO_JOINT_POSITION[i] = 0.0;
  }
}

Irp6otmJ2M::~Irp6otmJ2M() {
}

bool Irp6otmJ2M::configureHook() {
  motor_position_.resize(NUMBER_OF_SERVOS);
  joint_position_.resize(NUMBER_OF_SERVOS);

  SYNCHRO_JOINT_POSITION[0] = synchro_motor_position_[0] - GEAR[0] * THETA[0];
  SYNCHRO_JOINT_POSITION[1] = synchro_motor_position_[1] - GEAR[1] * THETA[1];
  SYNCHRO_JOINT_POSITION[2] = synchro_motor_position_[2] - GEAR[2] * THETA[2];
  SYNCHRO_JOINT_POSITION[3] = synchro_motor_position_[3] - GEAR[3] * THETA[3];
  SYNCHRO_JOINT_POSITION[4] = synchro_motor_position_[4] - GEAR[4] * THETA[4];
  SYNCHRO_JOINT_POSITION[5] = synchro_motor_position_[5] - GEAR[5] * THETA[5]
      - synchro_motor_position_[4];
  SYNCHRO_JOINT_POSITION[6] = synchro_motor_position_[6] - GEAR[6] * THETA[6];

  return true;
}

void Irp6otmJ2M::updateHook() {
  if (port_joint_position_.read(joint_position_) == RTT::NewData) {
    if (i2mp(&joint_position_(0), &motor_position_(0))) {
      port_motor_position_.write(motor_position_);
    }
  }
}

bool Irp6otmJ2M::i2mp(const double* joints, double* motors) {
  // Niejednoznacznosc polozenia dla 4-tej osi (obrot kisci < 180).
  const double joint_4_revolution = M_PI;
  // Niejednoznacznosc polozenia dla 5-tej osi (obrot kisci > 360).
  const double axis_5_revolution = 2 * M_PI;

  // Obliczanie kata obrotu walu silnika napedowego toru
  motors[0] = GEAR[0] * joints[0] + SYNCHRO_JOINT_POSITION[0];

  // Obliczanie kata obrotu walu silnika napedowego kolumny
  motors[1] = GEAR[1] * joints[1] + SYNCHRO_JOINT_POSITION[1];

  // Obliczanie kata obrotu walu silnika napedowego ramienia dolnego
  motors[2] = GEAR[2]
      * sqrt(sl123 + mi2 * cos(joints[2]) + ni2 * sin(-joints[2]))
      + SYNCHRO_JOINT_POSITION[2];

  // Obliczanie kata obrotu walu silnika napedowego ramienia gornego
  motors[3] = GEAR[3]
      * sqrt(
          sl123 + mi3 * cos(joints[3] + joints[2] + M_PI_2)
              + ni3 * sin(-(joints[3] + joints[2] + M_PI_2)))
      + SYNCHRO_JOINT_POSITION[3];

  // Obliczanie kata obrotu walu silnika napedowego obotu kisci T
  // jesli jest mniejsze od -pi/2

  double joints_tmp4 = joints[4] + joints[3] + joints[2] + M_PI_2;
  if (joints_tmp4 < -M_PI_2)
    joints_tmp4 += joint_4_revolution;

  motors[4] = GEAR[4] * (joints_tmp4 + THETA[4]) + SYNCHRO_JOINT_POSITION[4];

  // Obliczanie kata obrotu walu silnika napedowego obrotu kisci V
  motors[5] = GEAR[5] * joints[5] + SYNCHRO_JOINT_POSITION[5] + motors[4];

  // Ograniczenie na obrot.
  while (motors[5] < LOWER_MOTOR_LIMIT[5])
    motors[5] += axis_5_revolution;
  while (motors[5] > UPPER_MOTOR_LIMIT[5])
    motors[5] -= axis_5_revolution;

  // Obliczanie kata obrotu walu silnika napedowego obrotu kisci N
  motors[6] = GEAR[6] * joints[6] + SYNCHRO_JOINT_POSITION[6];

  return true;
}


ORO_CREATE_COMPONENT(Irp6otmJ2M)

