#ifndef ForceTransformation_H_
#define ForceTransformation_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Wrench.h"

#include <Eigen/Dense>

#include "ForceTrans.h"

class ForceTransformation : public RTT::TaskContext {
 public:
  ForceTransformation(const std::string& name);
  virtual ~ForceTransformation();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

 private:

  RTT::InputPort<geometry_msgs::Pose> port_current_wrist_pose_;

  RTT::InputPort<geometry_msgs::Wrench> port_current_sensor_wrench_;
  RTT::OutputPort<geometry_msgs::Wrench> port_output_wrist_wrench_;
  RTT::InputPort<geometry_msgs::Pose> port_tool_;

  geometry_msgs::Pose current_wrist_pose_;

  ForceTrans *gravity_transformation;  // klasa likwidujaca wplyw grawitacji na czujnik

  KDL::Wrench force_offset;

};

#endif /* ForceTransformation_H_ */
