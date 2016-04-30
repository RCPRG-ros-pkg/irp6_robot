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

#ifndef CONVSUPERVISOR_H_
#define CONVSUPERVISOR_H_

#include <std_msgs/Bool.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/SendHandle.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/base/AttributeBase.hpp>

#include <rtt/extras/SlaveActivity.hpp>

#include <Eigen/Dense>
#include <string>
#include <vector>


class ConvSupervisor : public RTT::TaskContext {
 private:
  // ports

  RTT::InputPort<std_msgs::Bool> port_do_synchro_in_;
  RTT::InputPort<std_msgs::Bool> port_emergency_stop_in_;
  RTT::InputPort<bool> port_generator_active_in_;

  RTT::InputPort<bool> port_is_synchronised_hi_mw_in_;
  RTT::InputPort<bool> port_is_hardware_panic_hi_mw_in_;

  RTT::OutputPort<bool> port_is_synchronised_out_;
  RTT::OutputPort<bool> port_is_hardware_panic_out_;
  RTT::OutputPort<bool> port_is_hardware_busy_out_;

  RTT::OutputPort<std_msgs::Bool> port_do_synchro_hi_mw_out_;
  RTT::OutputPort<bool> port_emergency_stop_hi_mw_out_;


 public:
  explicit ConvSupervisor(const std::string& name);
  ~ConvSupervisor();

  bool configureHook();
  bool startHook();
  void updateHook();
};

#endif  // CONVSUPERVISOR_H_
