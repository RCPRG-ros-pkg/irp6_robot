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
#include "Irp6otmInverseKinematic.h"
#include "eigen_conversions/eigen_msg.h"

Irp6otmInverseKinematic::Irp6otmInverseKinematic(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      d1(0.0),
      a2(0.0),
      a3(0.0),
      d5(0.0) {

  this->ports()->addPort("CurrentJointPosition", port_current_joint_position_);
  this->ports()->addPort("OutputJointPosition", port_output_joint_position_);

  this->ports()->addPort("Tool", port_tool_);

  this->ports()->addPort("InputWristPose", port_input_wrist_pose_);
  this->ports()->addPort("InputEndEffectorPose", port_input_end_effector_pose_);
}

Irp6otmInverseKinematic::~Irp6otmInverseKinematic() {
}

bool Irp6otmInverseKinematic::configureHook() {
  /* -----------------------------------------------------------------------
   Dlugosci czlonow robota [m].
   ------------------------------------------------------------------------- */
  d1 = d1_const;
  a2 = a2_const;
  a3 = a3_const;
  d5 = d5_const;

  local_desired_joints_.resize(NUMBER_OF_SERVOS);
  local_current_joints_.resize(NUMBER_OF_SERVOS);
  local_current_joints_tmp_.resize(NUMBER_OF_SERVOS);

  if (port_tool_.read(tool_msgs_) == RTT::NewData) {
    return false;
  }
  return true;
}

void Irp6otmInverseKinematic::updateHook() {
  if (port_input_wrist_pose_.read(wrist_pose_) == RTT::NewData) {
    Eigen::Affine3d trans;
    tf::poseMsgToEigen(wrist_pose_, trans);

    port_current_joint_position_.read(local_current_joints_);

    inverse_kinematics_single_iteration(local_current_joints_, trans,
                                        &local_desired_joints_);

    port_output_joint_position_.write(local_desired_joints_);

  } else if (port_input_end_effector_pose_.read(end_effector_pose_)
      == RTT::NewData) {
    port_tool_.read(tool_msgs_);

    Eigen::Affine3d tool;
    Eigen::Affine3d trans;
    Eigen::Affine3d wrist_pose;
    tf::poseMsgToEigen(end_effector_pose_, trans);
    tf::poseMsgToEigen(tool_msgs_, tool);

    wrist_pose = trans * tool.inverse();

    port_current_joint_position_.read(local_current_joints_);

    inverse_kinematics_single_iteration(local_current_joints_, wrist_pose,
                                        &local_desired_joints_);

    port_output_joint_position_.write(local_desired_joints_);
  }
}

void Irp6otmInverseKinematic::inverse_kinematics_single_iteration(
    const Eigen::VectorXd& local_current_joints,
    const Eigen::Affine3d& local_desired_end_effector_frame,
    Eigen::VectorXd* local_desired_joints) {
  // poprawka w celu uwzglednienia konwencji DH
  local_current_joints_tmp_ = local_current_joints;

  local_current_joints_tmp_[3] += local_current_joints_tmp_[2] + M_PI_2;
  local_current_joints_tmp_[4] += local_current_joints_tmp_[3];

  // Stale
  const double EPS = 1e-10;

  // Zmienne pomocnicze.
  double Nx, Ox, Ax, Px;
  double Ny, Oy, Ay, Py;
  // double Nz, Oz,
  double Az, Pz;
  double s1, c1, s2, c2, s4, c4, s5, c5;
  double E, F, K, ro, G, H;
  double t6, t_ok;

  // Przepisanie zmiennych.
  Nx = local_desired_end_effector_frame(0, 0);
  Ny = local_desired_end_effector_frame(1, 0);
  // Nz = local_desired_end_effector_frame(2, 0);
  Ox = local_desired_end_effector_frame(0, 1);
  Oy = local_desired_end_effector_frame(1, 1);
  // Oz = local_desired_end_effector_frame(2, 1);
  Ax = local_desired_end_effector_frame(0, 2);
  Ay = local_desired_end_effector_frame(1, 2);
  Az = local_desired_end_effector_frame(2, 2);
  Px = local_desired_end_effector_frame(0, 3);
  Py = local_desired_end_effector_frame(1, 3) - local_current_joints_tmp_[0];
  Pz = local_desired_end_effector_frame(2, 3) - z_offset_const;

  //  Wyliczenie Theta1.
  (*local_desired_joints)[1] = (atan2(Py, Px));
  s1 = sin((*local_desired_joints)[1]);
  c1 = cos((*local_desired_joints)[1]);

  // Wyliczenie Theta5.
  c5 = Ay * c1 - Ax * s1;
  // Sprawdzenie bledow numerycznych.
  if (fabs(c5 * c5 - 1) > EPS)
    s5 = sqrt(1 - c5 * c5);
  else
    s5 = 0;

  double cj_tmp;
  double dj_translation;
  // Sprawdzenie rozwiazania.
  if (local_current_joints_tmp_[5] > M_PI) {
    cj_tmp = local_current_joints_tmp_[5] - 2 * M_PI;
    dj_translation = 2 * M_PI;
  } else if (local_current_joints_tmp_[5] < -M_PI) {
    cj_tmp = local_current_joints_tmp_[5] + 2 * M_PI;
    dj_translation = -2 * M_PI;
  } else {
    cj_tmp = local_current_joints_tmp_[5];
    dj_translation = 0.0;
  }

  // Niejednoznacznosc - uzywamy rozwiazanie blizsze poprzedniemu.
  if (cj_tmp > 0)
    (*local_desired_joints)[5] = atan2(s5, c5);
  else
    (*local_desired_joints)[5] = atan2(-s5, c5);

  // Dodanie przesuniecia.
  (*local_desired_joints)[5] += dj_translation;

  // Wyliczenie Theta4 i Theta6.
  if (fabs(s5) < EPS) {
    printf("Osobliwosc ot\n");
    // W przypadku osobliwosci katowi theta4 przypisywana wartosc poprzednia.
    (*local_desired_joints)[4] = local_current_joints_tmp_[4];
    t6 = atan2(c1 * Nx + s1 * Ny, c1 * Ox + s1 * Oy);

    // Sprawdzenie warunkow.
    t_ok = t6 + (*local_desired_joints)[4];
    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(
            t6 - M_PI + (*local_desired_joints)[4]
                - (local_current_joints_tmp_[6])))
      t_ok = t6 - M_PI + (*local_desired_joints)[4];
    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(
            t6 + M_PI + (*local_desired_joints)[4]
                - (local_current_joints_tmp_[6])))
      t_ok = t6 + M_PI + (*local_desired_joints)[4];

    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(
            t6 - 2 * M_PI + (*local_desired_joints)[4]
                - (local_current_joints_tmp_[6])))
      t_ok = t6 - 2 * M_PI + (*local_desired_joints)[4];
    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(
            t6 + 2 * M_PI + (*local_desired_joints)[4]
                - (local_current_joints_tmp_[6])))
      t_ok = t6 + 2 * M_PI + (*local_desired_joints)[4];

    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(
            t6 - (*local_desired_joints)[4] - (local_current_joints_tmp_[6])))
      t_ok = t6 - (*local_desired_joints)[4];
    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(
            t6 - M_PI - (*local_desired_joints)[4]
                - (local_current_joints_tmp_[6])))
      t_ok = t6 - M_PI - (*local_desired_joints)[4];
    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(
            t6 + M_PI - (*local_desired_joints)[4]
                - (local_current_joints_tmp_[6])))
      t_ok = t6 + M_PI - (*local_desired_joints)[4];

    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(
            t6 - 2 * M_PI - (*local_desired_joints)[4]
                - (local_current_joints_tmp_[6])))
      t_ok = t6 - 2 * M_PI - (*local_desired_joints)[4];
    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(
            t6 + 2 * M_PI - (*local_desired_joints)[4]
                - (local_current_joints_tmp_[6])))
      t_ok = t6 + 2 * M_PI - (*local_desired_joints)[4];

    //  std::cout << "variant 1" << std::endl;

    (*local_desired_joints)[6] = t_ok;
  } else {
    t6 = atan2(-s1 * Ox + c1 * Oy, s1 * Nx - c1 * Ny);
    t_ok = t6;
    //  std::cout << "variant 2" << std::endl;
    // Sprawdzenie warunkow.
    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(t6 - M_PI - (local_current_joints_tmp_[6]))) {
      //  std::cout << "variant 2 a" << std::endl;
      t_ok = t6 - M_PI;
    }
    if (fabs(t_ok - local_current_joints_tmp_[6])
        > fabs(t6 + M_PI - (local_current_joints_tmp_[6]))) {
      //  std::cout << "variant 2 b" << std::endl;
      t_ok = t6 + M_PI;
    }

    (*local_desired_joints)[6] = t_ok;
    t_ok = atan2(c1 * Ax + s1 * Ay, Az);

    if (fabs(t_ok - local_current_joints_tmp_[4])
        > fabs(t_ok - M_PI - (local_current_joints_tmp_[4]))) {
      //  std::cout << "variant 2 c" << std::endl;
      t_ok = t_ok - M_PI;
    }
    if (fabs(t_ok - local_current_joints_tmp_[4])
        > fabs(t_ok + M_PI - (local_current_joints_tmp_[4]))) {
      //  std::cout << "variant 2 d" << std::endl;
      t_ok = t_ok + M_PI;
    }

    (*local_desired_joints)[4] = t_ok;
  }  // else

  // Wyliczenie Theta2.
  c4 = cos(static_cast<double>((*local_desired_joints)[4]));
  s4 = sin(static_cast<double>((*local_desired_joints)[4]));

  E = c1 * Px + s1 * Py - c4 * d5;
  F = -Pz - s4 * d5;
  G = 2 * E * a2;
  H = 2 * F * a2;
  K = E * E + F * F + a2 * a2 - a3 * a3;
  ro = sqrt(G * G + H * H);

  (*local_desired_joints)[2] = atan2(K / ro, sqrt(1 - ((K * K) / (ro * ro))))
      - atan2(G, H);

  // Wyliczenie Theta3.
  s2 = sin(static_cast<double>((*local_desired_joints)[2]));
  c2 = cos(static_cast<double>((*local_desired_joints)[2]));
  (*local_desired_joints)[3] = atan2(F - a2 * s2, E - a2 * c2);

  // Tor. Nie bierze udzialu w tym rozwiazaniu.
  (*local_desired_joints)[0] = local_current_joints_tmp_[0];

  // poprawka w celu dostosowania do konwencji DH
  (*local_desired_joints)[3] -= (*local_desired_joints)[2] + M_PI_2;
  (*local_desired_joints)[4] -= (*local_desired_joints)[3]
      + (*local_desired_joints)[2] + M_PI_2;
}

ORO_CREATE_COMPONENT(Irp6otmInverseKinematic)

