#include <time.h>
#include <rtt/RTT.hpp>

#include <simclock_singleton/simclock_singleton.h>
#include <simclock_singleton/simclock_singleton_interface.h>

namespace simclock_singleton {
boost::shared_ptr<simclock_singleton::SimClockSingleton> sim_clock_thread;
}

const bool simclock_singleton::register_robot_active(const int robot_code) {
  return SimClockSingleton::Instance()->registerRobotActive(robot_code);
}

const bool simclock_singleton::declare_readiness(const int robot_code) {
  return SimClockSingleton::Instance()->declareReadiness(robot_code);
}

//
