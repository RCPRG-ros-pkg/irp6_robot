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

  RTT::InputPort<geometry_msgs::Wrench> port_current_wrench_;
  RTT::OutputPort<geometry_msgs::Wrench> port_output_wrench_;

  geometry_msgs::Pose current_wrist_pose_;

  // MRROC++

  /*!
   * \brief Info if the force sensor test mode is active.
   *
   * It is taken from configuration data.
   */
  bool force_sensor_test_mode;

  bool is_reading_ready;  // czy jakikolwiek odczyt jest gotowy?

  // is sensor_frame right turn
  bool is_right_turn_frame;

  // force_sensor_frame related to wrist frame
  KDL::Frame force_sensor_frame;

  ForceTrans *gravity_transformation;  // klasa likwidujaca wplyw grawitacji na czujnik

  // ft_table used in get_reading and get_particualr_reading
  KDL::Wrench ft_table;

  KDL::Wrench force_offset;

  KDL::Frame tool_mass_center_translation;

  //! czy czujnik skonfigurowany?
  bool is_sensor_configured;

};

#endif /* ForceTransformation_H_ */
