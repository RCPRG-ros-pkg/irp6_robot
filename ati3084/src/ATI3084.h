#ifndef ATI3084_H
#define ATI3084_H

#include <string>
#include <comedilib.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"

const double FORCE_CONSTRAINTS[6] = { 65.0, 65.0, 130.0, 5.0, 5.0, 5.0 };

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

#define WRENCH_BUFFER_SIZE 4


class ATI3084 : public RTT::TaskContext {
 public:
  ATI3084(const std::string &name);

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();

 protected:
  RTT::OutputPort<geometry_msgs::Wrench> wrench_port_;
  RTT::Property<std::string> device_prop_;
  RTT::Property<KDL::Wrench> offset_prop_;
 private:
  comedi_t *device_;
  lsampl_t raw_ADC_[6];

  Vector6d voltage_ADC_;
  Vector6d bias_;
  Matrix6d conversion_matrix;  // F/T conversion matrix
  Vector6d conversion_scale;  // F/T scaling
  lsampl_t maxdata_;
  comedi_range *rangetype_;

  KDL::Wrench wrench_;
  KDL::Wrench computed_wrench_;
  std::vector<KDL::Wrench> wrench_buffer_;

  bool initSensor();
  void readData();
  void voltage2FT();
  void computeWrench();

};

#endif // ATI3084_H
