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

#ifndef IRP6PMFORWARDKINEMATIC_H_
#define IRP6PMFORWARDKINEMATIC_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <string>


#include "Irp6pmKinematic.h"

class Irp6pmForwardKinematic : public RTT::TaskContext {
 public:
  explicit Irp6pmForwardKinematic(const std::string& name);
  virtual ~Irp6pmForwardKinematic();

  bool configureHook();
  void updateHook();

 private:
  void direct_kinematics_transform(
      const Eigen::VectorXd& local_current_joints,
      Eigen::Affine3d* local_current_end_effector_frame);

  RTT::InputPort<Eigen::VectorXd> port_joint_position_;

  RTT::InputPort<geometry_msgs::Pose> port_tool_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_wrist_pose_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_end_effector_pose_;

  Eigen::VectorXd joint_position_;
  Eigen::VectorXd local_current_joints_tmp_;

  geometry_msgs::Pose tool_msgs_;

  // ! D-H kinematic parameters - length of 2nd segment.
  double a2;

  // ! D-H kinematic parameters - length of 3rd segment.
  double a3;

  // ! D-H kinematic parameters - length of 4th segment.
  double d5;
};

#endif  // IRP6PMFORWARDKINEMATIC_H_
