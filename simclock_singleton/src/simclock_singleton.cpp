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

#include <rtt/TaskContext.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt/plugin/Plugin.hpp>

#include <rtt/os/StartStopManager.hpp>

#include <ros/time.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/subscribe_options.h>

#include <simclock_singleton/simclock_singleton.h>
#include <simclock_singleton/simclock_singleton_interface.h>

using namespace simclock_singleton;

boost::shared_ptr<SimClockSingleton> SimClockSingleton::singleton;

boost::shared_ptr<SimClockSingleton> SimClockSingleton::GetInstance() {
  return singleton;
}

boost::shared_ptr<SimClockSingleton> SimClockSingleton::Instance() {
  // Create a new singleton, if necessary
  boost::shared_ptr<SimClockSingleton> shared = GetInstance();
  if (!shared) {
    shared.reset(new SimClockSingleton());
    singleton = shared;
  }

  return shared;
}

void SimClockSingleton::Release() {
  singleton.reset();
}

namespace {
RTT::os::CleanupFunction cleanup(&SimClockSingleton::Release);
}

SimClockSingleton::SimClockSingleton()
    : simclock_ns_interval_(0) {
  now = ros::Time(0, 0);
  rtt_rosclock::enable_sim();

  for (int i = 0; i < MAX_NR_OF_ROBOTS; i++) {
    is_robot_active[i] = false;
    is_robot_ready[i] = 0;
  }
}

SimClockSingleton::~SimClockSingleton() {
}

bool SimClockSingleton::registerRobotActive(int robot_code, int ns_interval) {
//  std::cout << "update_clock_counter: " << update_clock_counter << std::endl;

  bool to_return_robot_code = false;
  bool to_return_ns_interval = false;

  RTT::os::MutexLock lock(m_);

  if ((robot_code >= 0) && (robot_code < MAX_NR_OF_ROBOTS)) {
    if (!is_robot_active[robot_code]) {
      is_robot_active[robot_code] = true;
      to_return_robot_code = true;
    } else {
      std::cout
          << "SimClockSingleton::registerRobotActive, robot_code argument: "
          << robot_code << "previously registered" << std::endl;
    }
  } else {
    std::cout << "SimClockSingleton::registerRobotActive, robot_code argument: "
        << robot_code << "out of range" << std::endl;
  }

  if (ns_interval > 0) {
    if (!simclock_ns_interval_) {
      simclock_ns_interval_ = ns_interval;
      to_return_ns_interval = true;
    } else {
      if (!(ns_interval == simclock_ns_interval_)) {
        std::cout
            << "SimClockSingleton::registerRobotActive, ns_interval argument: "
            << ns_interval << "differs from previous one: "
            << simclock_ns_interval_ << std::endl;
      } else {
        to_return_ns_interval = true;
      }
    }
  } else {
    std::cout
        << "SimClockSingleton::registerRobotActive, ns_interval argument: "
        << ns_interval << "out of range" << std::endl;
  }

  return to_return_robot_code && to_return_ns_interval;
}

bool SimClockSingleton::declareReadiness(int robot_code) {
//  std::cout << "update_clock_counter: " << update_clock_counter << std::endl;
  RTT::os::MutexLock lock(m_);
  bool to_return = false;

  if ((robot_code >= 0) && (robot_code < MAX_NR_OF_ROBOTS)) {
    is_robot_ready[robot_code]++;
  } else {
    std::cout << "SimClockSingleton::declareReadiness, robot_code argument: "
        << robot_code << "out of range" << std::endl;
  }

  bool do_increment_time = true;

  for (int i = 0; i < MAX_NR_OF_ROBOTS; i++) {
    if (is_robot_active[i] && (!is_robot_ready[i])) {
      do_increment_time = false;
    }
  }

  if (do_increment_time) {
    for (int i = 0; i < MAX_NR_OF_ROBOTS; i++) {
      is_robot_ready[i]--;
    }

    now += ros::Duration(0, simclock_ns_interval_);
    /*
     std::cout << "robot_code: " << robot_code << " , now 2: " << now
     << " is_track_ready: " << is_track_ready << " is_postument_ready: "
     << is_postument_ready << " is_conveyor_ready: " << is_conveyor_ready
     << std::endl;
     */
    rtt_rosclock::update_sim_clock(now);
    to_return = true;
  }

  return to_return;
}

