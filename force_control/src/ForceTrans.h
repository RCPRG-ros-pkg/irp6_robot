#ifndef __FORCETRANS_H
#define __FORCETRANS_H

#include "kdl/frames.hpp"

class ForceTrans {

 protected:
  bool initialized;
  double tool_weight;

  KDL::Wrench gravity_force_torque_in_base;
  KDL::Wrench reaction_force_torque_in_wrist;
  KDL::Vector gravity_arm_in_wrist;

  KDL::Frame sensor_frame;

  KDL::Frame tool_mass_center_translation;

  bool is_right_turn_frame;

 public:
  ForceTrans(const KDL::Frame & init_frame, const KDL::Frame & s_frame,
             const double weight, const KDL::Vector & point_of_gravity,
             bool _is_right_turn_frame);
  void synchro(const KDL::Frame & init_frame);
  void defineTool(const KDL::Frame & init_frame, const double weight,
                  const KDL::Vector & point_of_gravity);

  KDL::Wrench getForce(const KDL::Wrench _inputForceTorque,
                       const KDL::Frame curr_frame);

};

#endif
