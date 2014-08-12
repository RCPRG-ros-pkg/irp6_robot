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

const double FORCE_CONSTRAINTS[6] = { 65.0, 65.0, 130.0, 5.0, 5.0, 5.0 };

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

#define FAST_WRENCH_BUFFER_SIZE 2
#define USLEEP_MUX 50


class ATI3084 : public ForceSensor {
 public:
  ATI3084(const std::string &name);

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();

 protected:
  RTT::Property<std::string> device_prop_;

 private:

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
