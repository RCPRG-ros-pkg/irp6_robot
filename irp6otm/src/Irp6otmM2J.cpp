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
#include "Irp6otmM2J.h"
#include "Irp6otmTransmission.h"

Irp6otmM2J::Irp6otmM2J(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      desired_motor_pos_initiated_(false) {
  this->ports()->addPort("MotorPosition", port_motor_position_);
  this->ports()->addPort("DesiredMotorPosition", port_desired_motor_position_);
  this->ports()->addPort("JointPosition", port_joint_position_);

  this->addProperty("synchro_motor_position", synchro_motor_position_);
}

Irp6otmM2J::~Irp6otmM2J() {
}

bool Irp6otmM2J::configureHook() {
  motor_position_.resize(NUMBER_OF_SERVOS);
  joint_position_.resize(NUMBER_OF_SERVOS);

  return true;
}

void Irp6otmM2J::updateHook() {
  if (RTT::NewData == port_desired_motor_position_.read(motor_position_)) {
    mp2i(&motor_position_(0), &joint_position_(0));
    port_joint_position_.write(joint_position_);
    desired_motor_pos_initiated_ = true;
  } else if ((!desired_motor_pos_initiated_)
      && (RTT::NewData == port_motor_position_.read(motor_position_))) {
    mp2i(&motor_position_(0), &joint_position_(0));
    port_joint_position_.write(joint_position_);
  }
}

void Irp6otmM2J::mp2i(const double* motors, double* joints) {
  // zmienne pomocnicze
  double c, d, l;
  double sinus, cosinus;
  double M2, M3;

  // Przelicznik polozenia walu silnika napedowego toru w radianach
  // na przesuniecie toru (wspolrzedna wewnetrzna) w metrach
  joints[0] = (motors[0] - synchro_motor_position_[0]) / GEAR[0];

  // Przelicznik polozenia walu silnika napedowego kolumny w radianach
  // na kat obrotu kolumny (wspolrzedna wewnetrzna) w radianach
  joints[1] = (motors[1] - synchro_motor_position_[1]) / GEAR[1] + THETA[1];

  // Przelicznik polozenia walu silnika napedowego ramienia dolnego w radianach
  // na kat obrotu ramienia (wspolrzedna wewnetrzna) w radianach
  l = (motors[2] - synchro_motor_position_[2]) / GEAR[2] + THETA[2];
  M2 = mi2 * mi2 + ni2 * ni2;
  c = l * l - sl123;
  d = sqrt(M2 - c * c);
  cosinus = (mi2 * c - ni2 * d) / M2;
  sinus = -(ni2 * c + mi2 * d) / M2;
  joints[2] = atan2(sinus, cosinus);

  // Przelicznik polozenia walu silnika napedowego ramienia gornego w radianach
  // na kat obrotu ramienia (wspolrzedna wewnetrzna) w radianach
  l = (motors[3] - synchro_motor_position_[3]) / GEAR[3] + THETA[3];
  M3 = mi3 * mi3 + ni3 * ni3;
  c = l * l - sl123;
  d = sqrt(M3 - c * c);
  cosinus = (mi3 * c - ni3 * d) / M3;
  sinus = -(ni3 * c + mi3 * d) / M3;
  joints[3] = atan2(sinus, cosinus);

  // Przelicznik polozenia walu silnika napedowego obrotu kisci T w radianach
  // na kat pochylenia kisci (wspolrzedna wewnetrzna) w radianach
  joints[4] = (motors[4] - synchro_motor_position_[4]) / GEAR[4];

  // Przelicznik polozenia walu silnika napedowego obrotu kisci V w radianach
  // na kat obrotu kisci (wspolrzedna wewnetrzna) w radianach
  joints[5] = (motors[5] - synchro_motor_position_[5]
      - (motors[4] - synchro_motor_position_[4])) / GEAR[5] + THETA[5];

  // Przelicznik polozenia walu silnika napedowego obrotu kisci N w radianach
  // na kat obrotu kisci (wspolrzedna wewnetrzna) w radianach
  joints[6] = (motors[6] - synchro_motor_position_[6]) / GEAR[6] + THETA[6];

  // poprawka w celu dostosowania do konwencji DH
  joints[3] -= joints[2] + M_PI_2;
  joints[4] -= joints[3] + joints[2] + M_PI_2;
}

ORO_CREATE_COMPONENT(Irp6otmM2J)

