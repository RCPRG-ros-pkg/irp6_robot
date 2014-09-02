#ifndef ATI3084_H
#define ATI3084_H

#include <string>
#include <comedilib.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <kdl/frames.hpp>
#include "geometry_msgs/Wrench.h"
#include "../../force_sensor/src/ForceSensor.h"


#define USLEEP_MUX 50


class ATI3084 : public ForceSensor {
 public:
  ATI3084(const std::string &name);

  bool configureParticularSensorHook();


 protected:
  RTT::Property<std::string> device_prop_;

 private:


  lsampl_t maxdata_;
  comedi_range *rangetype_;


  void readData();



};

#endif // ATI3084_H
