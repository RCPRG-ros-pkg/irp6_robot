#include <rtt/Component.hpp>

#include "ForceTransformation.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

ForceTransformation::ForceTransformation(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      force_sensor_test_mode(false),
      is_reading_ready(false),
      is_right_turn_frame(true),
      gravity_transformation(NULL),
      is_sensor_configured(false) {

  this->ports()->addPort("CurrentWristPose", port_current_wrist_pose_);
  this->ports()->addPort("Tool", port_tool_);

  this->ports()->addPort("CurrentWrench", port_current_wrench_);
  this->ports()->addPort("OutputWrench", port_output_wrench_);

}

ForceTransformation::~ForceTransformation() {

}

bool ForceTransformation::configureHook() {

  return true;
}

bool ForceTransformation::startHook() {
  if (port_current_wrist_pose_.read(current_wrist_pose_) == RTT::NoData) {
    return false;
  }

  is_sensor_configured = true;

  // force offset determination
  if (!force_sensor_test_mode) {
    // read current force
    geometry_msgs::Wrench current_wrench;
    port_current_wrench_.read(current_wrench);

    tf::wrenchMsgToKDL(current_wrench, force_offset);

  }
  /*
   // polozenie kisci bez narzedzia wzgledem bazy
   lib::Homog_matrix current_frame =
   master.servo_current_frame_wo_tool_dp.read();  // FORCE Transformation by Slawomir Bazant

   if (!gravity_transformation)  // nie powolano jeszcze obiektu
   {

   // zczytanie sil maksymalnych
   if (master.config.exists("force_constraints")) {
   char *tmp = strdup(
   master.config.value < std::string > ("force_constraints").c_str());
   char* toDel = tmp;
   for (int i = 0; i < 6; i++) {
   force_constraints[i] = strtod(tmp, &tmp);
   }
   free(toDel);
   }

   lib::Xyz_Angle_Axis_vector tab;
   //zczytanie polozenia czujnika sily wzgledem nadgarstka
   if (master.config.exists("force_sensor_in_wrist")) {
   char *tmp = strdup(
   master.config.value < std::string
   > ("force_sensor_in_wrist").c_str());
   char* toDel = tmp;
   for (int i = 0; i < 6; i++) {
   tab[i] = strtod(tmp, &tmp);
   }
   force_sensor_frame = lib::Homog_matrix(tab);
   free(toDel);
   }

   // zczytanie ciezaru narzedzia
   double weight = master.config.value<double>("weight");

   next_force_tool_weight = weight;

   // polozenie srodka ciezkosci narzedzia wzgledem nadgarstka
   double point[3];
   char *tmp = strdup(
   master.config.value < std::string
   > ("default_mass_center_in_wrist").c_str());
   char* toDel = tmp;
   for (int i = 0; i < 3; i++)
   point[i] = strtod(tmp, &tmp);
   free(toDel);

   lib::K_vector pointofgravity(point);

   tool_mass_center_translation = lib::Homog_matrix(point[0], point[1],
   point[2]);

   gravity_transformation = new lib::ForceTrans(force_sensor_name,
   current_frame,
   force_sensor_frame, weight,
   pointofgravity,
   is_right_turn_frame);
   } else {
   gravity_transformation->synchro(current_frame);
   }

   */

  return true;
}

void ForceTransformation::updateHook() {

  geometry_msgs::Wrench current_wrench;
  port_current_wrench_.read(current_wrench);
  port_output_wrench_.write(current_wrench);

}

ORO_CREATE_COMPONENT(ForceTransformation)

