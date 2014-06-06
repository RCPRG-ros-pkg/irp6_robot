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
  rospy.init_node('multi_trajectory')
  rospy.wait_for_service('/controller_manager/switch_controller')
  conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
  
  #
  # Motor coordinates motion
  #
  
  conmanSwitch(['SplineTrajectoryGeneratorMotor'], [], True)
  
  motor_client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_motor', FollowJointTrajectoryAction)
  motor_client.wait_for_server()

  print 'server ok'

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  goal.trajectory.points.append(JointTrajectoryPoint([0.4, -1.5418065817051163, 0.0, 1.57, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)))
  goal.trajectory.points.append(JointTrajectoryPoint([10.0, 10.0, 0.0, 10.57, 10.57, -20.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(12.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  motor_client.send_goal(goal)

  motor_client.wait_for_result()
  command_result = motor_client.get_result()
  
    
  #
  # Joint coordinates motion
  #
  
  conmanSwitch(['SplineTrajectoryGeneratorJoint'], ['SplineTrajectoryGeneratorMotor'], True)
  
  joint_client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
  joint_client.wait_for_server()

  print 'server ok'

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  goal.trajectory.points.append(JointTrajectoryPoint([0.4, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(3.0)))
  goal.trajectory.points.append(JointTrajectoryPoint([0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(6.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  joint_client.send_goal(goal)

  joint_client.wait_for_result()
  command_result = joint_client.get_result()
  
  
  
  conmanSwitch(['PoseInt'], ['SplineTrajectoryGeneratorJoint'], True)
  
  #
  # Cartesian coordinates motion
  #
  
  pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
  pose_client.wait_for_server()
  
  print 'server ok'
     
  goal = CartesianTrajectoryGoal()
  
  rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.18029263241))
  
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()))
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(6.0), pm.toMsg(rot), Twist()))
  rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.3, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(9.0), pm.toMsg(rot), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  pose_client.send_goal(goal)

  pose_client.wait_for_result()
  command_result = pose_client.get_result()
  
  conmanSwitch([], ['PoseInt'], True)
  
  #
  # Tool motion
  #
  
  tool_client = actionlib.SimpleActionClient('/irp6p_arm/tool_trajectory', CartesianTrajectoryAction)
  tool_client.wait_for_server()
  
  print 'server ok'
     
  goal = CartesianTrajectoryGoal()
  
  
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(0.0), Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)
  
  tool_client.send_goal(goal)

  tool_client.wait_for_result()
  command_result = tool_client.get_result()
  

  #
  # Cartesian coordinates motion
  #
  
  conmanSwitch(['PoseInt'], [], True)
  
  pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
  pose_client.wait_for_server()
  
  print 'server ok'
     
  goal = CartesianTrajectoryGoal()
  
  rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
  
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()))
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(6.0), pm.toMsg(rot), Twist()))
  rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.3, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
  #goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(9.0), pm.toMsg(rot), Twist()))
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(9.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.63691, 0.096783, 0.75634, -0.11369)), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  pose_client.send_goal(goal)

  pose_client.wait_for_result()
  command_result = pose_client.get_result()
  
  conmanSwitch([], ['PoseInt'], True)
  
  #
  # Tool motion
  #
  
  tool_client = actionlib.SimpleActionClient('/irp6p_arm/tool_trajectory', CartesianTrajectoryAction)
  tool_client.wait_for_server()
  
  print 'server ok'
     
  goal = CartesianTrajectoryGoal()
  
  
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(0.0), Pose(Point(0.0, 0.0, 0.25), Quaternion(0.0, 0.0, 0.0, 1.0)), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)
  
  tool_client.send_goal(goal)

  tool_client.wait_for_result()
  command_result = tool_client.get_result()
  
  print 'finish'
  
  
  
  
  