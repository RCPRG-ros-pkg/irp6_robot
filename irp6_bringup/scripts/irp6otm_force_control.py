#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Robot Control and Pattern Recognition Group, Warsaw University of Technology
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of the <organization> nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import tf
import actionlib

from controller_manager_msgs.srv import *
from std_msgs.msg import *
from diagnostic_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from force_control_msgs.msg import *
from tf.transformations import *

#from PyKDL import *
import PyKDL
import tf_conversions.posemath as pm

if __name__ == '__main__':
  rospy.init_node('irp6otm_force_control')

  rospy.wait_for_service('/irp6ot_manager/switch_controller')
  conmanSwitch = rospy.ServiceProxy('/irp6ot_manager/switch_controller', SwitchController)

      
  #
  # Deactivate all generators
  #
  
  conmanSwitch([], ['Irp6otmSplineTrajectoryGeneratorMotor','Irp6otmSplineTrajectoryGeneratorJoint','Irp6otmPoseInt','Irp6otmForceControlLaw','Irp6otmForceTransformation'], True)
  
     
  # 
  # Force controller parameters
  #
  
  pub = rospy.Publisher('/irp6ot_arm/fcl_param', ForceControl, queue_size=0)
  
  rospy.sleep(0.5)
  
  goal = ForceControl()
  goal.inertia = Inertia(Vector3(20.0, 20.0, 20.0), Vector3(0.5, 0.5, 0.5))
  goal.reciprocaldamping = ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.05, 0.05, 0.05))
  goal.wrench = Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
  goal.twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
  
  pub.publish(goal)
  
  
  conmanSwitch([], ['Irp6otmForceTransformation','Irp6otmForceControlLaw'], True)
  
  #
  # standard tool gravity param
  #
  
  pubtg = rospy.Publisher('/irp6ot_arm/tg_param', ToolGravityParam, queue_size=0)
  rospy.sleep(0.5)
  
  tg_goal = ToolGravityParam()
  tg_goal.weight = 10.8
  tg_goal.mass_center = Vector3(0.004, 0.0, 0.156)

 
  pubtg.publish(tg_goal)
   
  conmanSwitch(['Irp6otmForceTransformation','Irp6otmForceControlLaw'], [], True)
   
  print 'finish'
  
  
  
  
  