#include <rtt/Component.hpp>

#include "Irp6pInverseKinematic.h"
#include "eigen_conversions/eigen_msg.h"

Irp6pInverseKinematic::Irp6pInverseKinematic(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("CurrentJointPosition", port_current_joint_position_);
  this->ports()->addPort("OutputJointPosition", port_output_joint_position_);

  this->ports()->addPort("Tool", port_tool_);

  this->ports()->addPort("InputWristPose", port_input_wrist_pose_);
  this->ports()->addPort("InputEndEffectorPose", port_input_end_effector_pose_);

}

Irp6pInverseKinematic::~Irp6pInverseKinematic() {

}

bool Irp6pInverseKinematic::configureHook() {

  /* -----------------------------------------------------------------------
   Dlugosci czlonow robota [m].
   ------------------------------------------------------------------------- */
  a2 = 0.455;
  a3 = 0.67;
  d5 = 0.19;
  d6 = 0.095;
  d7 = 0.20;

  local_desired_joints_.resize(6);
  local_current_joints_.resize(6);
  local_current_joints_tmp_.resize(6);

  if (port_tool_.read(tool_msgs_) == RTT::NewData) {

    return false;

  }

  return true;
}

void Irp6pInverseKinematic::updateHook() {

  if (port_input_wrist_pose_.read(wrist_pose_) == RTT::NewData) {

    Eigen::Affine3d trans;
    tf::poseMsgToEigen(wrist_pose_, trans);

    port_current_joint_position_.read(local_current_joints_);

    inverse_kinematics_single_iteration(local_current_joints_, trans,
                                        &local_desired_joints_);

    port_output_joint_position_.write(local_desired_joints_);

  } else if (port_input_end_effector_pose_.read(end_effector_pose_) == RTT::NewData) {

    port_tool_.read(tool_msgs_);

    Eigen::Affine3d tool;
    Eigen::Affine3d trans;
    Eigen::Affine3d wrist_pose;
    tf::poseMsgToEigen(end_effector_pose_, trans);
    tf::poseMsgToEigen(tool_msgs_, tool);


    wrist_pose = trans * tool.inverse();


    port_current_joint_position_.read(local_current_joints_);

    inverse_kinematics_single_iteration(local_current_joints_, wrist_pose,
                                        &local_desired_joints_);

    port_output_joint_position_.write(local_desired_joints_);
  }
}

void Irp6pInverseKinematic::inverse_kinematics_single_iteration(
    const Eigen::VectorXd& local_current_joints,
    const Eigen::Affine3d& local_desired_end_effector_frame,
    Eigen::VectorXd* local_desired_joints) {

  // poprawka w celu uwzglednienia konwencji DH
  local_current_joints_tmp_ = local_current_joints;

  local_current_joints_tmp_[2] += local_current_joints_tmp_[1] + M_PI_2;
  local_current_joints_tmp_[3] += local_current_joints_tmp_[2];

  // Stale
  const double EPS = 1e-10;

  // Zmienne pomocnicze.
  double Nx, Ox, Ax, Px;
  double Ny, Oy, Ay, Py;
  double Nz, Oz, Az, Pz;
  double s0, c0, s1, c1, s3, c3, s4, c4;
  double E, F, K, ro, G, H;
  double t5, t_ok;

  // Przepisanie zmiennych.
  Nx = local_desired_end_effector_frame(0, 0);
  Ny = local_desired_end_effector_frame(1, 0);
  Nz = local_desired_end_effector_frame(2, 0);
  Ox = local_desired_end_effector_frame(0, 1);
  Oy = local_desired_end_effector_frame(1, 1);
  Oz = local_desired_end_effector_frame(2, 1);
  Ax = local_desired_end_effector_frame(0, 2);
  Ay = local_desired_end_effector_frame(1, 2);
  Az = local_desired_end_effector_frame(2, 2);
  Px = local_desired_end_effector_frame(0, 3);
  Py = local_desired_end_effector_frame(1, 3);
  Pz = local_desired_end_effector_frame(2, 3);

  //  Wyliczenie Theta1.
  (*local_desired_joints)[0] = (atan2(Py, Px));
  s0 = sin((*local_desired_joints)[0]);
  c0 = cos((*local_desired_joints)[0]);

  // Wyliczenie Theta5.
  c4 = Ay * c0 - Ax * s0;
  // Sprawdzenie bledow numerycznych.
  if (fabs(c4 * c4 - 1) > EPS)
    s4 = sqrt(1 - c4 * c4);
  else
    s4 = 0;

  double cj_tmp;
  double dj_translation;
  // Sprawdzenie rozwiazania.
  if (local_current_joints_tmp_[4] > M_PI) {
    cj_tmp = local_current_joints_tmp_[4] - 2 * M_PI;
    dj_translation = 2 * M_PI;
  } else if (local_current_joints_tmp_[4] < -M_PI) {
    cj_tmp = local_current_joints_tmp_[4] + 2 * M_PI;
    dj_translation = -2 * M_PI;
  } else {
    cj_tmp = local_current_joints_tmp_[4];
    dj_translation = 0.0;
  }

  // Niejednoznacznosc - uzywamy rozwiazanie blizsze poprzedniemu.
  if (cj_tmp > 0)
    (*local_desired_joints)[4] = atan2(s4, c4);
  else
    (*local_desired_joints)[4] = atan2(-s4, c4);

  // Dodanie przesuniecia.
  (*local_desired_joints)[4] += dj_translation;

  // Wyliczenie Theta4 i Theta6.
  if (fabs(s4) < EPS) {
    printf("Osobliwosc\n");
    // W przypadku osobliwosci katowi theta4 przypisywana wartosc poprzednia.
    (*local_desired_joints)[3] = local_current_joints_tmp_[3];
    t5 = atan2(c0 * Nx + s0 * Ny, c0 * Ox + s0 * Oy);

    // Sprawdzenie warunkow.
    t_ok = t5 + (*local_desired_joints)[3];
    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(
            t5 - M_PI + (*local_desired_joints)[3]
                - (local_current_joints_tmp_[5])))
      t_ok = t5 - M_PI + (*local_desired_joints)[3];
    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(
            t5 + M_PI + (*local_desired_joints)[3]
                - (local_current_joints_tmp_[5])))
      t_ok = t5 + M_PI + (*local_desired_joints)[3];

    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(
            t5 - 2 * M_PI + (*local_desired_joints)[3]
                - (local_current_joints_tmp_[5])))
      t_ok = t5 - 2 * M_PI + (*local_desired_joints)[3];
    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(
            t5 + 2 * M_PI + (*local_desired_joints)[3]
                - (local_current_joints_tmp_[5])))
      t_ok = t5 + 2 * M_PI + (*local_desired_joints)[3];

    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(
            t5 - (*local_desired_joints)[3] - (local_current_joints_tmp_[5])))
      t_ok = t5 - (*local_desired_joints)[3];
    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(
            t5 - M_PI - (*local_desired_joints)[3]
                - (local_current_joints_tmp_[5])))
      t_ok = t5 - M_PI - (*local_desired_joints)[3];
    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(
            t5 + M_PI - (*local_desired_joints)[3]
                - (local_current_joints_tmp_[5])))
      t_ok = t5 + M_PI - (*local_desired_joints)[3];

    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(
            t5 - 2 * M_PI - (*local_desired_joints)[3]
                - (local_current_joints_tmp_[5])))
      t_ok = t5 - 2 * M_PI - (*local_desired_joints)[3];
    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(
            t5 + 2 * M_PI - (*local_desired_joints)[3]
                - (local_current_joints_tmp_[5])))
      t_ok = t5 + 2 * M_PI - (*local_desired_joints)[3];

    (*local_desired_joints)[5] = t_ok;
  } else {
    t5 = atan2(-s0 * Ox + c0 * Oy, s0 * Nx - c0 * Ny);
    t_ok = t5;

    // Sprawdzenie warunkow.
    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(t5 - M_PI - (local_current_joints_tmp_[5])))
      t_ok = t5 - M_PI;
    if (fabs(t_ok - local_current_joints_tmp_[5])
        > fabs(t5 + M_PI - (local_current_joints_tmp_[5])))
      t_ok = t5 + M_PI;

    (*local_desired_joints)[5] = t_ok;
    t_ok = atan2(c0 * Ax + s0 * Ay, Az);

    if (fabs(t_ok - local_current_joints_tmp_[3])
        > fabs(t_ok - M_PI - (local_current_joints_tmp_[3])))
      t_ok = t_ok - M_PI;
    if (fabs(t_ok - local_current_joints_tmp_[3])
        > fabs(t_ok + M_PI - (local_current_joints_tmp_[3])))
      t_ok = t_ok + M_PI;
    (*local_desired_joints)[3] = t_ok;
  }  //: else

  // Wyliczenie Theta2.
  c3 = cos((*local_desired_joints)[3]);
  s3 = sin((*local_desired_joints)[3]);

  E = c0 * Px + s0 * Py - c3 * d5;
  F = -Pz - s3 * d5;
  G = 2 * E * a2;
  H = 2 * F * a2;
  K = E * E + F * F + a2 * a2 - a3 * a3;
  ro = sqrt(G * G + H * H);

  (*local_desired_joints)[1] = atan2(K / ro, sqrt(1 - ((K * K) / (ro * ro))))
      - atan2(G, H);

  // Wyliczenie Theta3.
  s1 = sin((*local_desired_joints)[1]);
  c1 = cos((*local_desired_joints)[1]);
  (*local_desired_joints)[2] = atan2(F - a2 * s1, E - a2 * c1);

  // poprawka w celu dostosowania do konwencji DH
  (*local_desired_joints)[2] -= (*local_desired_joints)[1] + M_PI_2;
  (*local_desired_joints)[3] -= (*local_desired_joints)[2]
      + (*local_desired_joints)[1] + M_PI_2;

}

ORO_CREATE_COMPONENT(Irp6pInverseKinematic)

