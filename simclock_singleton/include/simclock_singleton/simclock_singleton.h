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

#ifndef SIMCLOCK_SINGLETON_H_
#define SIMCLOCK_SINGLETON_H_

#include <rtt/Service.hpp>
#include <rtt/os/Thread.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/os/MutexLock.hpp>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <rosgraph_msgs/Clock.h>
#include "rtt_rosclock/rtt_rosclock.h"

#define TRACK 1
#define POSTUMENT 2
#define CONVEYOR 3

namespace simclock_singleton {

/**
 * This activity subscribes to the ROS /clock topic and overrides the RTT
 * TimeService if the `/use_sim_time` ROS parameter is set.
 */
class SimClockSingleton {
 public:
  // ! Get an instance to the singleton SimClockSingleton or create one
  static boost::shared_ptr<SimClockSingleton> Instance();
  // !  Get an instance to the singleton SimClockSingleton or NULL
  static boost::shared_ptr<SimClockSingleton> GetInstance();
  // ! Release the singleton SimClockSingleton
  static void Release();

  virtual ~SimClockSingleton();

  /**
   * Update the RTT clock and SimClockActivities with a new time
   *
   * This can be called internally from clockMsgCallback or externally
   * by another library (e.g. Gazebo) for in-process triggering. This
   * can be called manually via the ros.clock global service's updateSimClock()
   * operation.
   */
  bool registerRobotActive(int robot_code);

  // returns true when time was incremented.
  // If so, the calling component should publish new time on port to be then streamed to ROS
  bool declareReadiness(int robot_code);

 protected:
  // ! Constructor is protected, use Instance() to create and get a singleton
  SimClockSingleton();
  SimClockSingleton(SimClockSingleton const&);
  void operator=(SimClockSingleton const&);

  // ! SimClockSingleton singleton
  static boost::shared_ptr<SimClockSingleton> singleton;

 private:
  RTT::os::Mutex m_;

  bool is_track_active, is_postument_active, is_conveyor_active;
  int is_track_ready, is_postument_ready, is_conveyor_ready;
  ros::Time now;

  // rozwazenia w przyszlosci jako property
  int simclock_ns_interval_;
};

}  // namespace simclock_singleton

#endif  // SIMCLOCK_SINGLETON_H_

