#ifndef __FORCETRANS_H
#define __FORCETRANS_H

#include "kdl/frames.hpp"

class ForceTrans
{

protected:

	const short force_sensor_name;

	bool initialized;
	double tool_weight;

	KDL::Wrench gravity_force_torque_in_base;
	KDL::Wrench reaction_force_torque_in_wrist;
	KDL::Vector gravity_arm_in_wrist;

	KDL::Frame sensor_frame;

	KDL::Frame ft_tool_mass_center_translation;

	KDL::Frame ft_tr_sensor_in_wrist;

	bool is_right_turn_frame;

public:
	ForceTrans(const short l_force_sensor_name, const KDL::Frame & init_frame, const KDL::Frame & s_frame, const double weight, const KDL::Vector & point_of_gravity, bool _is_right_turn_frame);
  void synchro(const KDL::Frame & init_frame);

	/*
	void defineTool(const lib::Homog_matrix & init_frame, const double weight, const lib::K_vector & point_of_gravity);
	lib::Ft_vector getForce(const lib::Ft_vector _inputForceTorque, const lib::Homog_matrix curr_frame);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	*/
};

#endif
