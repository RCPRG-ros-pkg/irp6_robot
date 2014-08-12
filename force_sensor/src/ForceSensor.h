#ifndef FORCESENSOR_H
#define FORCESENSOR_H

#include <string>
#include <comedilib.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

const double FORCE_CONSTRAINTS[6] = { 65.0, 65.0, 130.0, 5.0, 5.0, 5.0 };

class ForceSensor : public RTT::TaskContext {

 public:
  ForceSensor(const std::string &name);
  bool startHook();
  void updateHook();

 protected:
  Matrix6d conversion_matrix;  // F/T conversion matrix
  Vector6d conversion_scale;  // F/T scaling

  KDL::Wrench wrench_;

  RTT::OutputPort<geometry_msgs::Wrench> wrench_port_;
  RTT::Property<KDL::Wrench> offset_prop_;

  void WrenchKDLToMsg(const KDL::Wrench &in, geometry_msgs::Wrench &out);
  void voltage2FT();
  virtual void readData() = 0;

  comedi_t *device_;

  lsampl_t raw_ADC_[6];
  Vector6d voltage_ADC_;
  Vector6d bias_;

};

#endif // FORCESENSOR_H
