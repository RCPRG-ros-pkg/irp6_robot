/*
 * Copyright (c) 2014-2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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

#include "SimClock.h"

#include <rtt/Component.hpp>
#include <string>

SimClock::SimClock(const std::string& name)
    : RTT::TaskContext(name),
      simclock_ns_interval_(0) {
  this->ports()->addPort("ROS_TIME", port_ros_time_).doc("");
  this->addProperty("simclock_ns_interval", simclock_ns_interval_).doc("");
  // this->addProperty("simclock_ratio", simclock_ratio_).doc("");
}

SimClock::~SimClock() {
}

bool SimClock::configureHook() {
  now = ros::Time(0, 0);

  // ponizsza linie mozna tez wywolac w OPS zgodnie z
  // https://github.com/orocos/rtt_ros_integration/tree/indigo-devel/rtt_rosclock
  rtt_rosclock::enable_sim();
  return true;
}

bool SimClock::startHook() {
  // ponizsza linia bedzie sesnowna kiedy da sie uruchamiac komponenty z inna czestotliwoscia pobudzane zegarem
  // narazie (25.10.2017) pomimo tego ze jest to zadeklarowane w dokumentqacji powyzej to nie dziala
  // final_ns_interval_ = static_cast<int>(simclock_ratio_  *static_cast<double>(simclock_ns_interval_)  );

  return true;
}

void SimClock::updateHook() {
  now += ros::Duration(0, simclock_ns_interval_);
  rtt_rosclock::update_sim_clock(now);
  rosgraph_msgs::Clock now_exp;
  now_exp.clock = now;
  port_ros_time_.write(now_exp);
 // simclock_singleton::update_sim_clock();

//  std::cout << "now: " << rtt_rosclock::host_now() << std::endl;
}

ORO_CREATE_COMPONENT(SimClock)

