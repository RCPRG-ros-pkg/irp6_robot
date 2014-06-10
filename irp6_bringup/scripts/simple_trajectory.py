#! /usr/bin/env python

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

from tf.transformations import *

from PyKDL import Rotation

if __name__ == '__main__':
  rospy.init_node('simple_trajectory')
  rospy.wait_for_service('/controller_manager/switch_controller')
  conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
  
  conmanSwitch(['SplineTrajectoryGeneratorJoint'], [], True)
    
  client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
  client.wait_for_server()

  print 'server ok'


 

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  goal.trajectory.points.append(JointTrajectoryPoint([1.57, -1.5418065817051163, 0.0, 1.57, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  client.send_goal(goal)

  client.wait_for_result()
  command_result = client.get_result()
  
  conmanSwitch([], ['SplineTrajectoryGeneratorJoint'], True)
  
  print 'finish'

