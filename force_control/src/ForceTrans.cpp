#include "ForceTrans.h"

ForceTrans::ForceTrans(const KDL::Frame & init_frame,
                       const KDL::Frame & s_frame, const double weight,
                       const KDL::Vector & point_of_gravity,
                       bool _is_right_turn_frame)
    : initialized_(false),
      is_right_turn_frame_(true) {

// ustalenie skretnosci wektora z odczytami z czujnika
  is_right_turn_frame_ = _is_right_turn_frame;

  // polozenie czujnika wzgledem nadgarstka
  sensor_frame_ = s_frame;

  tool_weight_ = weight;
  gravity_arm_in_wrist_ = point_of_gravity;
  synchro(init_frame);
  defineTool(init_frame, weight, point_of_gravity);

  initialized_ = true;

}

void ForceTrans::defineTool(const KDL::Frame & init_frame, const double weight,
                            const KDL::Vector & point_of_gravity) {

  tool_weight_ = weight;
  gravity_arm_in_wrist_ = point_of_gravity;

  gravity_force_torque_in_base_ = KDL::Wrench(
      KDL::Vector(0.0, 0.0, -tool_weight_), KDL::Vector(0.0, 0.0, 0.0));

// orientacja koncowki manipulatora bez narzedzia
  KDL::Frame current_orientation(init_frame.M, KDL::Vector(0.0, 0.0, 0.0));

// sila reakcji w ukladzie czujnika z orientacja bazy
  KDL::Wrench gravity_force_torque_in_sensor = current_orientation.Inverse()
      * gravity_force_torque_in_base_;

// macierz narzedzia wzgledem nadgarstka
  tool_mass_center_translation_ = KDL::Frame(KDL::Rotation(), point_of_gravity);

// sila reakcji w ukladzie nadgarstka z orientacja bazy
  reaction_force_torque_in_wrist_ = -(tool_mass_center_translation_
      * gravity_force_torque_in_sensor);

}

// zwraca sily i momenty sil w w ukladzie z orientacja koncowki manipulatory bez narzedzia
KDL::Wrench ForceTrans::getForce(const KDL::Wrench _inputForceTorque,
                                 const KDL::Frame curr_frame) {

  KDL::Wrench inputForceTorque = _inputForceTorque;

  if (!is_right_turn_frame_) {

    inputForceTorque[2] = -inputForceTorque[2];
    inputForceTorque[5] = -inputForceTorque[5];
  }

  if (initialized_) {

    // sprowadzenie wejsciowych, zmierzonych sil i momentow sil z ukladu czujnika do ukladu nadgarstka
    KDL::Wrench input_force_torque = sensor_frame_ * inputForceTorque;

    // sprowadzenie odczytow sil do ukladu czujnika przy zalozeniu ze uklad chwytaka ma te sama orientacje
    // co uklad narzedzia
    KDL::Wrench gravity_force_torque_in_sensor = (curr_frame.Inverse()).M
        * gravity_force_torque_in_base_;

    // finalne przeksztalcenie (3.30 z doktoratu TW)
    KDL::Wrench output_force_torque = input_force_torque
        - tool_mass_center_translation_ * gravity_force_torque_in_sensor
        - reaction_force_torque_in_wrist_;

    // sprowadzenie sily w ukladzie nadgarstka do orientacji ukladu bazowego
    output_force_torque = curr_frame.M * (-output_force_torque);

    return output_force_torque;
  }

  return KDL::Wrench();
}

void ForceTrans::synchro(const KDL::Frame & init_frame) {
  if (initialized_) {
    defineTool(init_frame, tool_weight_, gravity_arm_in_wrist_);
  }
}

