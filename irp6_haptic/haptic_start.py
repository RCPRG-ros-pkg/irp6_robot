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
  rospy.init_node('haptic_start')
  rospy.wait_for_service('/irp6p_manager/switch_controller')
  conmanSwitchIrp6p = rospy.ServiceProxy('/irp6p_manager/switch_controller', SwitchController)
  
  print "servers irp6p ok"
  
  rospy.wait_for_service('/irp6ot_manager/switch_controller')
  conmanSwitchIrp6ot = rospy.ServiceProxy('/irp6ot_manager/switch_controller', SwitchController)
  
  print "servers irp6ot ok"
  
  rospy.wait_for_service('/haptic_manager/switch_controller')
  conmanSwitchHaptic = rospy.ServiceProxy('/haptic_manager/switch_controller', SwitchController)
  
  print "haptic server ok"
  
  #
  # Deactivate all generators
  #
  
  conmanSwitchIrp6ot([], ['Irp6otmSplineTrajectoryGeneratorMotor','Irp6otmSplineTrajectoryGeneratorJoint','Irp6otmPoseInt','Irp6otmForceControlLaw','Irp6otmForceTransformation'], True)
  conmanSwitchIrp6p([], ['Irp6pmSplineTrajectoryGeneratorMotor','Irp6pmSplineTrajectoryGeneratorJoint','Irp6pmPoseInt','Irp6pmForceControlLaw','Irp6pmForceTransformation'], True)
  
  
  #
  # Aprroach to initial position
  #
  
  print 'Irp6 on track approach to initial position'
  
  
  #
  # Joint coordinates motion
  #
  
  conmanSwitchIrp6ot(['Irp6otmSplineTrajectoryGeneratorJoint'], [], True)
  
  joint_client = actionlib.SimpleActionClient('/irp6ot_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
  joint_client.wait_for_server()

  print 'server ok'

  # joint_client.cancel_all_goals()
  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
  goal.trajectory.points.append(JointTrajectoryPoint([0.0, 0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -1.57], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(6.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  joint_client.send_goal(goal)

  joint_client.wait_for_result()
  command_result = joint_client.get_result()
    
  conmanSwitchIrp6ot([], ['Irp6otmSplineTrajectoryGeneratorJoint'], True)
  
  
  #
  # Cartesian coordinates motion
  #
  
  conmanSwitchIrp6ot(['Irp6otmPoseInt'], [], True)
  
  pose_client = actionlib.SimpleActionClient('/irp6ot_arm/pose_trajectory', CartesianTrajectoryAction)
  pose_client.wait_for_server()
  
  print 'server ok'
 
  # pose_client.cancel_all_goals()
      
  goal = CartesianTrajectoryGoal()
  
 
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.905438961242, 0.0, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  pose_client.send_goal(goal)

  pose_client.wait_for_result()
  command_result = pose_client.get_result()
  
  conmanSwitchIrp6ot([], ['Irp6otmPoseInt'], True)
  
  print 'Irp6 postument approach to initial position'
  
  #
  # Joint coordinates motion
  #
  
  conmanSwitchIrp6p(['Irp6pmSplineTrajectoryGeneratorJoint'], [], True)
  
  joint_client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
  joint_client.wait_for_server()

  print 'server ok'

  # joint_client.cancel_all_goals()
  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  goal.trajectory.points.append(JointTrajectoryPoint([0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -1.57], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(6.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  joint_client.send_goal(goal)

  joint_client.wait_for_result()
  command_result = joint_client.get_result()
  
  
  
  conmanSwitchIrp6p([], ['Irp6pmSplineTrajectoryGeneratorJoint'], True)
  
  #
  # Cartesian coordinates motion
  #
  
  conmanSwitchIrp6p(['Irp6pmPoseInt'], [], True)
  
  pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
  pose_client.wait_for_server()
  
  print 'server ok'
  
  # pose_client.cancel_all_goals()   
  goal = CartesianTrajectoryGoal()
  
  
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.905438961242, 0.0, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  pose_client.send_goal(goal)

  pose_client.wait_for_result()
  command_result = pose_client.get_result()
  
  conmanSwitchIrp6p([], ['Irp6pmPoseInt'], True)
  
  
  print 'Irp6 on track force control parameters'   
  
  # 
  # Force controller parameters
  #
  
  pub = rospy.Publisher('/irp6ot_arm/fcl_param', ForceControl, queue_size=0)
  
  rospy.sleep(0.5)
  
  goal = ForceControl()
  goal.inertia = Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.5, 0.5, 0.5))
  goal.reciprocaldamping = ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.05, 0.05, 0.05))
  goal.wrench = Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
  goal.twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
  
  pub.publish(goal)
  
  #
  # standard tool gravity param
  #
  
  pubtg = rospy.Publisher('/irp6ot_arm/tg_param', ToolGravityParam, queue_size=0)
  rospy.sleep(0.5)
  
  tg_goal = ToolGravityParam()
  tg_goal.weight = 10.8
  tg_goal.mass_center = Vector3(0.004, 0.0, 0.156)

 
  pubtg.publish(tg_goal)
   
  print 'Irp6 postument force control parameters'     
  
    
  #
  # standard tool gravity param
  #
  
  pubtg = rospy.Publisher('/irp6p_arm/tg_param', ToolGravityParam, queue_size=0)
  rospy.sleep(0.5)
  
  tg_goal = ToolGravityParam()
  tg_goal.weight = 10.8
  tg_goal.mass_center = Vector3(0.004, 0.0, 0.156)

 
  pubtg.publish(tg_goal)
   
   
   
  conmanSwitchIrp6ot(['Irp6otmForceTransformation','Irp6otmForceControlLaw'], [], True)
  conmanSwitchIrp6p(['Irp6pmForceTransformation'], [], True)
  conmanSwitchHaptic(['Irp6Haptic'], [], True)  
    
  print 'finish'
  
  
  
  
  