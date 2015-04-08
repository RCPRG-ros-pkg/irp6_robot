/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SARKOFAGREGULATOR_H_
#define SARKOFAGREGULATOR_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <math.h>       /* pow */
#include <string>

class SarkofagRegulator : public RTT::TaskContext {
 public:
  explicit SarkofagRegulator(const std::string& name);
  ~SarkofagRegulator();

  int doServo(double, int);
  int doServo_friction_test(double, int);
  void reset();

 private:
  bool configureHook();
  void updateHook();

  RTT::InputPort<double> desired_position_;
  RTT::InputPort<double> deltaInc_in;
  RTT::InputPort<bool> synchro_state_in_;

  RTT::OutputPort<double> computedPwm_out;
  RTT::OutputPort<bool> emergency_stop_out_;

  double desired_position_increment_;
  double desired_position_old_, desired_position_new_;
  double deltaIncData;

  bool synchro_state_old_, synchro_state_new_;

  int64_t update_hook_iteration_number_;
  int64_t new_position_iteration_number_;

  // Properties
  int reg_number_;
  bool debug_;
  double A_;
  double BB0_;
  double BB1_;
  bool current_mode_;
  double max_output_current_;
  double current_reg_kp_;
  double eint_dif_;
  double max_desired_increment_;
  double enc_res_;

  std::string regulator_type_;

  enum reg_type {
    irp6,
    friction_test,
    pos_inc
  } type;

  double position_increment_old;  // przedosatnio odczytany przyrost polozenie (delta y[k-2]
  // -- mierzone w impulsach)
  double position_increment_new;  // ostatnio odczytany przyrost polozenie (delta y[k-1]
  // -- mierzone w impulsach)
  double step_old_pulse;  // poprzednia wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-2]
  // -- mierzone w radianach)
  double step_new;  // nastepna wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-1]
  // -- mierzone w radianach)
  double step_old;  // poprzednia wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-1]
  // -- mierzone w radianach)

  double set_value_new;  // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k])
  double set_value_old;  // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k-1])
  double set_value_very_old;  // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k-2])
  double delta_eint;  // przyrost calki uchybu
  double delta_eint_old;  // przyrost calki uchybu w poprzednim kroku

  double output_value;

  double a_, b0_, b1_;
};
#endif  // SARKOFAGREGULATOR_H_
