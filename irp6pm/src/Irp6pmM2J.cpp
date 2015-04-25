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

#include "Irp6pmM2J.h"
#include "Irp6pmTransmission.h"

Irp6pmM2J::Irp6pmM2J(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {
  this->ports()->addPort("MotorPosition", port_motor_position_);
  this->ports()->addPort("JointPosition", port_joint_position_);

  this->addProperty("synchro_motor_position", synchro_motor_position_);
}

Irp6pmM2J::~Irp6pmM2J() {
}

bool Irp6pmM2J::configureHook() {
  motor_position_.resize(NUMBER_OF_SERVOS);
  joint_position_.resize(NUMBER_OF_SERVOS);
  return true;
}

void Irp6pmM2J::updateHook() {
  if (RTT::NewData == port_motor_position_.read(motor_position_)) {
    mp2i(&motor_position_(0), &joint_position_(0));
    port_joint_position_.write(joint_position_);
  }
}

void Irp6pmM2J::mp2i(const double* motors, double* joints) {
  // zmienne pomocnicze
  double c, d, l;
  double sinus, cosinus;
  double M1, M2;

// Przelicznik polozenia walu silnika napedowego kolumny w radianach
// na kat obrotu kolumny (wspolrzedna wewnetrzna) w radianach
  joints[0] = (motors[0] - synchro_motor_position_[0]) / GEAR[0] + THETA[0];

// Przelicznik polozenia walu silnika napedowego ramienia dolnego w radianach
// na kat obrotu ramienia (wspolrzedna wewnetrzna) w radianach
  l = (motors[1] - synchro_motor_position_[1]) / GEAR[1] + THETA[1];
  M1 = mi1 * mi1 + ni1 * ni1;
  c = l * l - sl123;
  d = sqrt(M1 - c * c);
  cosinus = (mi1 * c - ni1 * d) / M1;
  sinus = -(ni1 * c + mi1 * d) / M1;
  joints[1] = atan2(sinus, cosinus);

// Przelicznik polozenia walu silnika napedowego ramienia gornego w radianach
// na kat obrotu ramienia (wspolrzedna wewnetrzna) w radianach
  l = (motors[2] - synchro_motor_position_[2]) / GEAR[2] + THETA[2];
  M2 = mi2 * mi2 + ni2 * ni2;
  c = l * l - sl123;
  d = sqrt(M2 - c * c);
  cosinus = (mi2 * c - ni2 * d) / M2;
  sinus = -(ni2 * c + mi2 * d) / M2;
  joints[2] = atan2(sinus, cosinus);

// Przelicznik polozenia walu silnika napedowego obrotu kisci T w radianach
// na kat pochylenia kisci (wspolrzedna wewnetrzna) w radianach
  joints[3] = (motors[3] - synchro_motor_position_[3]) / GEAR[3];

// Przelicznik polozenia walu silnika napedowego obrotu kisci V w radianach
// na kat obrotu kisci (wspolrzedna wewnetrzna) w radianach
  joints[4] = (motors[4] - synchro_motor_position_[4]
      - (motors[3] - synchro_motor_position_[3])) / GEAR[4] + THETA[4];

// Przelicznik polozenia walu silnika napedowego obrotu kisci N w radianach
// na kat obrotu kisci (wspolrzedna wewnetrzna) w radianach
  joints[5] = (motors[5] - synchro_motor_position_[5]) / GEAR[5] + THETA[5];

  joints[2] -= joints[1] + M_PI_2;
  joints[3] -= joints[2] + joints[1] + M_PI_2;
}

ORO_CREATE_COMPONENT(Irp6pmM2J)

