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

class ForceSensor : public RTT::TaskContext {

 public:
  ForceSensor(const std::string &name);
  bool startHook();
  void updateHook();
  virtual bool configureHook();

 protected:
  Matrix6d conversion_matrix;  // F/T conversion matrix
  Vector6d conversion_scale;  // F/T scaling

  KDL::Wrench wrench_;
  KDL::Wrench valid_wrench_;

  RTT::OutputPort<geometry_msgs::Wrench> raw_wrench_output_port_;
  RTT::OutputPort<geometry_msgs::Wrench> fast_filtered_wrench_output_port_;
  RTT::OutputPort<geometry_msgs::Wrench> slow_filtered_wrench_output_port_;

  void WrenchKDLToMsg(const KDL::Wrench &in, geometry_msgs::Wrench &out);
  void voltage2FT();
  virtual void readData() = 0;
  virtual bool configureParticularSensorHook() = 0;

  comedi_t *device_;

  lsampl_t raw_ADC_[6];
  Vector6d voltage_ADC_;
  Vector6d bias_;

  std::vector<KDL::Wrench> slow_buffer_;
  std::vector<KDL::Wrench> fast_buffer_;

  int slow_buffer_index_;
  int fast_buffer_index_;

  KDL::Wrench slow_filtered_wrench_;
  KDL::Wrench fast_filtered_wrench_;

  // properties
  std::vector<double> force_limits_;
  RTT::Property<KDL::Wrench> offset_prop_;
  int slow_buffer_size_;
  int fast_buffer_size_;
  bool test_mode_;

};

#endif // FORCESENSOR_H
