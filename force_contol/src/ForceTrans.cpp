#include "ForceTrans.h"

ForceTrans::ForceTrans(const short l_force_sensor_name,
                       const KDL::Frame & init_frame,
                       const KDL::Frame & s_frame, const double weight,
                       const KDL::Vector & point_of_gravity,
                       bool _is_right_turn_frame)
    : force_sensor_name(l_force_sensor_name),
      initialized(false),
      is_right_turn_frame(true) {

// ustalenie skretnosci wektora z odczytami z czujnika
  is_right_turn_frame = _is_right_turn_frame;

  // polozenie czujnika wzgledem nadgarstka
  sensor_frame = s_frame;

//  ft_tr_sensor_in_wrist = lib::Xi_f(sensor_frame);
  ft_tr_sensor_in_wrist = sensor_frame;

  tool_weight = weight;
  gravity_arm_in_wrist = point_of_gravity;
/* Temporary removal
 // synchro(init_frame);
 // defineTool(init_frame, weight, point_of_gravity);
  */

  initialized = true;

}

/*
 void ForceTrans::defineTool(const lib::Homog_matrix & init_frame, const double weight, const lib::K_vector & point_of_gravity)
 {
 tool_weight = weight;
 gravity_arm_in_wrist = point_of_gravity;

 gravity_force_torque_in_base = lib::Ft_vector(0.0, 0.0, -tool_weight, 0.0, 0.0, 0.0);

 // orientacja koncowki manipulatora bez narzedzia
 lib::Homog_matrix current_orientation(init_frame.return_with_with_removed_translation());

 // sila reakcji w ukladzie czujnika z orientacja bazy
 lib::Ft_vector gravity_force_torque_in_sensor(lib::Xi_f(!current_orientation) * gravity_force_torque_in_base);

 // macierz narzedzia wzgledem nadgarstka
 lib::Homog_matrix tool_mass_center_translation(point_of_gravity[0], point_of_gravity[1], point_of_gravity[2]);
 ft_tool_mass_center_translation = lib::Xi_f(tool_mass_center_translation);

 // sila reakcji w ukladzie nadgarstka z orientacja bazy
 reaction_force_torque_in_wrist = -(ft_tool_mass_center_translation * gravity_force_torque_in_sensor);

 }

 // zwraca sily i momenty sil w w ukladzie z orientacja koncowki manipulatory bez narzedzia
 lib::Ft_vector ForceTrans::getForce(const lib::Ft_vector _inputForceTorque, const lib::Homog_matrix curr_frame)
 {

 lib::Ft_vector inputForceTorque = _inputForceTorque;

 if (!is_right_turn_frame) {

 inputForceTorque[2] = -inputForceTorque[2];
 inputForceTorque[5] = -inputForceTorque[5];
 }

 if (initialized) {

 // sprowadzenie wejsciowych, zmierzonych sil i momentow sil z ukladu czujnika do ukladu nadgarstka
 lib::Ft_vector input_force_torque(ft_tr_sensor_in_wrist * inputForceTorque);

 // sprowadzenie odczytow sil do ukladu czujnika przy zalozeniu ze uklad chwytaka ma te sama orientacje
 // co uklad narzedzia
 lib::Ft_vector gravity_force_torque_in_sensor(lib::Xi_star(!curr_frame) * gravity_force_torque_in_base);

 // finalne przeksztalcenie (3.30 z doktoratu TW)
 lib::Ft_vector output_force_torque(input_force_torque
 - (ft_tool_mass_center_translation * gravity_force_torque_in_sensor) - reaction_force_torque_in_wrist);

 // sprowadzenie sily w ukladzie nadgarstka do orientacji ukladu bazowego
 output_force_torque = lib::Xi_star(curr_frame) * Ft_vector(-output_force_torque);

 return output_force_torque;
 }
 return 0;
 }

 void ForceTrans::synchro(const lib::Homog_matrix & init_frame)
 {
 //initialisation_frame = init_frame;
 if (initialized)
 defineTool(init_frame, tool_weight, gravity_arm_in_wrist);
 }

 */
