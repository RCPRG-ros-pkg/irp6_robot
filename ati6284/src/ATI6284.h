#ifndef ATI6284_H
#define ATI6284_H

#include <string>
#include <comedilib.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <kdl/frames.hpp>

#include "geometry_msgs/Wrench.h"
#include "../../force_sensor/src/ForceSensor.h"





class ATI6284 : public ForceSensor {
 public:
  ATI6284(const std::string &name);

  bool configureParticularSensorHook();



 protected:
  RTT::Property<std::string> device_prop_;

 private:

  comedi_polynomial_t calib_ADC_;



  void readData();


};

#endif // ATI6284_H
