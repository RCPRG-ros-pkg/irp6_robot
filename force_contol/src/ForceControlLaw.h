#ifndef ForceControlLaw_H_
#define ForceControlLaw_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Wrench.h"

#include <Eigen/Dense>
#include "ForceTrans.h"


class ForceControlLaw : public RTT::TaskContext {
 public:
  ForceControlLaw(const std::string& name);
  virtual ~ForceControlLaw();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

 private:

  RTT::InputPort<geometry_msgs::Pose> port_current_pose_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_pose_;

  RTT::InputPort<geometry_msgs::Wrench> port_current_wrench_;

  geometry_msgs::Pose current_pose_;

  // MRROC++

  /*!
   * \brief Info if the force sensor test mode is active.
   *
   * It is taken from configuration data.
   */
  bool force_sensor_test_mode;

  bool is_reading_ready;  // czy jakikolwiek odczyt jest gotowy?

  // nazwa czujnika
  short force_sensor_name;

  // is sensor_frame right turn
  bool is_right_turn_frame;

  // force_sensor_frame related to wrist frame
  KDL::Frame force_sensor_frame;

  ForceTrans *gravity_transformation;  // klasa likwidujaca wplyw grawitacji na czujnik

  // ft_table used in get_reading and get_particualr_reading
  KDL::Wrench ft_table;

  KDL::Wrench force_constraints;

  KDL::Frame tool_mass_center_translation;

  //! czy czujnik skonfigurowany?
  bool is_sensor_configured;

};

#endif /* ForceControlLaw */
