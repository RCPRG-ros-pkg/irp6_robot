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
#include "Irp6otmForwardKinematic.h"
#include "eigen_conversions/eigen_msg.h"

Irp6otmForwardKinematic::Irp6otmForwardKinematic(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      d1(0.0),
      a2(0.0),
      a3(0.0),
      d5(0.0) {
  this->ports()->addPort("JointPosition", port_joint_position_);
  this->ports()->addPort("Tool", port_tool_);

  this->ports()->addPort("WristOutputPose", port_output_wrist_pose_);
  this->ports()->addPort("EndEffectorOutputPose",
                         port_output_end_effector_pose_);
}

Irp6otmForwardKinematic::~Irp6otmForwardKinematic() {
}

bool Irp6otmForwardKinematic::configureHook() {
  /* -----------------------------------------------------------------------
   Dlugosci czlonow robota [m].
   ------------------------------------------------------------------------- */
  d1 = d1_const;
  a2 = a2_const;
  a3 = a3_const;
  d5 = d5_const;
//  d6 = d6_const;
//  d7 = d7_const;

  joint_position_.resize(NUMBER_OF_SERVOS);
  local_current_joints_tmp_.resize(NUMBER_OF_SERVOS);

  if (port_tool_.read(tool_msgs_) == RTT::NewData) {
    return false;
  }

  return true;
}

void Irp6otmForwardKinematic::updateHook() {
  if (port_joint_position_.read(joint_position_) == RTT::NewData) {
    port_tool_.read(tool_msgs_);

    Eigen::Affine3d tool;
    Eigen::Affine3d end_effector_pose;
    Eigen::Affine3d wrist_pose;
    geometry_msgs::Pose wrist_pose_msgs;
    geometry_msgs::Pose end_effector_pose_msgs;

    tf::poseMsgToEigen(tool_msgs_, tool);

    direct_kinematics_transform(joint_position_, &wrist_pose);

    end_effector_pose = wrist_pose * tool;

    tf::poseEigenToMsg(wrist_pose, wrist_pose_msgs);
    tf::poseEigenToMsg(end_effector_pose, end_effector_pose_msgs);

    port_output_wrist_pose_.write(wrist_pose_msgs);
    port_output_end_effector_pose_.write(end_effector_pose_msgs);
  }
}

void Irp6otmForwardKinematic::direct_kinematics_transform(
    const Eigen::VectorXd& local_current_joints,
    Eigen::Affine3d* local_current_end_effector_frame) {

  // poprawka w celu uwzglednienia konwencji DH
  local_current_joints_tmp_ = local_current_joints;

  local_current_joints_tmp_[3] += local_current_joints_tmp_[2] + M_PI_2;
  local_current_joints_tmp_[4] += local_current_joints_tmp_[3];

  // Parametry pomocnicze - przeliczenie zmiennych.
  double d0 = local_current_joints_tmp_[0];
  double s1 = sin(local_current_joints_tmp_[1]);
  double c1 = cos(local_current_joints_tmp_[1]);
  double s2 = sin(local_current_joints_tmp_[2]);
  double c2 = cos(local_current_joints_tmp_[2]);
  double s3 = sin(local_current_joints_tmp_[3]);
  double c3 = cos(local_current_joints_tmp_[3]);
  double s4 = sin(local_current_joints_tmp_[4]);
  double c4 = cos(local_current_joints_tmp_[4]);
  double s5 = sin(local_current_joints_tmp_[5]);
  double c5 = cos(local_current_joints_tmp_[5]);
  double s6 = sin(local_current_joints_tmp_[6]);
  double c6 = cos(local_current_joints_tmp_[6]);

  // Proste zadanie kinematyki.
  (*local_current_end_effector_frame)(0, 0) = (c1 * s4 * c5 + s1 * s5) * c6
      + c1 * c4 * s6;  // NX
  (*local_current_end_effector_frame)(0, 1) = -(c1 * s4 * c5 + s1 * s5) * s6
      + c1 * c4 * c6;  // OX
  (*local_current_end_effector_frame)(0, 2) = c1 * s4 * s5 - s1 * c5;  // AX
  (*local_current_end_effector_frame)(0, 3) = c1
      * (a2 * c2 + a3 * c3 + d5 * c4);  // PX
  (*local_current_end_effector_frame)(1, 0) = (s1 * s4 * c5 - c1 * s5) * c6
      + s1 * c4 * s6;  // NY
  (*local_current_end_effector_frame)(1, 1) = -(s1 * s4 * c5 - c1 * s5) * s6
      + s1 * c4 * c6;  // OY
  (*local_current_end_effector_frame)(1, 2) = s1 * s4 * s5 + c1 * c5;  // AY
  (*local_current_end_effector_frame)(1, 3) = s1 * (a2 * c2 + a3 * c3 + d5 * c4)
      + d0;  // PY
  (*local_current_end_effector_frame)(2, 0) = c4 * c5 * c6 - s4 * s6;  // NZ
  (*local_current_end_effector_frame)(2, 1) = -c4 * c5 * s6 - s4 * c6;  // OZ
  (*local_current_end_effector_frame)(2, 2) = c4 * s5;  // AZ
  (*local_current_end_effector_frame)(2, 3) = -a2 * s2 - a3 * s3 - d5 * s4
      + z_offset_const;  // PZ
}

ORO_CREATE_COMPONENT(Irp6otmForwardKinematic)

