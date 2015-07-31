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
  rospy.init_node('irp6ot_tfg')
  

  rospy.wait_for_service('/irp6ot_manager/switch_controller')
  conmanSwitch = rospy.ServiceProxy('/irp6ot_manager/switch_controller', SwitchController)

  
  #
  # Motor coordinates motion
  #
  
  conmanSwitch(['Irp6ottfgSplineTrajectoryGeneratorMotor'], [], True)
  
  motor_client = actionlib.SimpleActionClient('/irp6ot_tfg/spline_trajectory_action_motor', FollowJointTrajectoryAction)
  motor_client.wait_for_server()

  print 'server ok'

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1']
  goal.trajectory.points.append(JointTrajectoryPoint([0.0], [0.0], [], [], rospy.Duration(3.0)))
  goal.trajectory.points.append(JointTrajectoryPoint([500.0], [0.0], [], [], rospy.Duration(6.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  motor_client.send_goal(goal)

  motor_client.wait_for_result()
  command_result = motor_client.get_result()
    
  conmanSwitch([], ['Irp6ottfgSplineTrajectoryGeneratorMotor'], True)  
    

  #
  # Joint coordinates motion
  #
  
  conmanSwitch(['Irp6ottfgSplineTrajectoryGeneratorJoint'], [], True)
  
  joint_client = actionlib.SimpleActionClient('/irp6ot_tfg/spline_trajectory_action_joint', FollowJointTrajectoryAction)
  joint_client.wait_for_server()

  print 'server ok'

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1']
  goal.trajectory.points.append(JointTrajectoryPoint([0.06], [0.0], [], [], rospy.Duration(3.0)))
  goal.trajectory.points.append(JointTrajectoryPoint([0.08], [0.0], [], [], rospy.Duration(6.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  joint_client.send_goal(goal)

  joint_client.wait_for_result()
  command_result = joint_client.get_result()
     
  conmanSwitch([], ['Irp6ottfgSplineTrajectoryGeneratorJoint'], True)
  
  print 'finish'
  
  
  
  
  