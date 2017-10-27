/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Johannes Meyer, TU Darmstadt
 *  Copyright (c) 2013, Intermodalics BVBA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TU Darmstadt and Intermodalics BVBA
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Copyright (c) 2014, Jonathan Bohren, The Johns Hopkins University
 *  - Generalized for multiple time sources
 *  - Integrated with simclock_singleton package
 *********************************************************************/

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
    : is_track_active(false),
      is_postument_active(false),
      is_conveyor_active(false),
      is_track_ready(false),
      is_postument_ready(false),
      is_conveyor_ready(false),
      simclock_ns_interval_(2000000) {
  now = ros::Time(0, 0);
  rtt_rosclock::enable_sim();
}

SimClockSingleton::~SimClockSingleton() {

}

bool SimClockSingleton::registerRobotActive(int robot_code) {
//  std::cout << "update_clock_counter: " << update_clock_counter << std::endl;

  bool to_return = false;

  switch (robot_code) {
    case TRACK:
      if (!is_track_active) {
        is_track_active = true;
        to_return = true;
      }
      break;
    case POSTUMENT:
      if (!is_postument_active) {
        is_postument_active = true;
        to_return = true;
      }
      break;
    case CONVEYOR:
      if (!is_conveyor_active) {
        is_conveyor_active = true;
        to_return = true;
      }
      break;
    default:
      std::cout
          << "SimClockSingleton::registerRobotActive, robot_code argument: "
          << robot_code << "out of range: " << robot_code << std::endl;
      break;
  }

  return to_return;
}

bool SimClockSingleton::declareReadiness(int robot_code) {

//  std::cout << "update_clock_counter: " << update_clock_counter << std::endl;

  bool to_return = false;

  switch (robot_code) {
    case TRACK:
      is_track_ready = true;
      break;
    case POSTUMENT:
      is_postument_ready = true;
      break;
    case CONVEYOR:
      is_conveyor_ready = true;
      break;
    default:
      std::cout << "SimClockSingleton::declareReadiness, robot_code argument: "
          << robot_code << "out of range: " << robot_code << std::endl;
      break;
  }

  if ((!is_track_active || (is_track_active && is_track_ready))
      && (!is_postument_active || (is_postument_active && is_postument_ready))
      && (!is_conveyor_active || (is_conveyor_active && is_conveyor_ready))) {
    is_track_ready = false;
    is_postument_ready = false;
    is_conveyor_ready = false;
    now += ros::Duration(0, simclock_ns_interval_);
    /*
     std::cout << "robot_code: " << robot_code << " , now 2: " << now
     << std::endl;
     */
    rtt_rosclock::update_sim_clock(now);
    to_return = true;
  }

  return to_return;
}

