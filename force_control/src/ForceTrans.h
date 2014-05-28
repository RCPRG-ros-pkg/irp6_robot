#ifndef __FORCETRANS_H
#define __FORCETRANS_H

#include "kdl/frames.hpp"

class ForceTrans {

 protected:
  bool initialized_;
  double tool_weight_;

  KDL::Wrench gravity_force_torque_in_base_;
  KDL::Wrench reaction_force_torque_in_wrist_;
  KDL::Vector gravity_arm_in_wrist_;

  KDL::Frame sensor_frame_;

  KDL::Frame tool_mass_center_translation_;

  bool is_right_turn_frame_;

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
